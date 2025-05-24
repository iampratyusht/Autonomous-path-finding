#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "src/CircularBufferQueue/CircularBufferQueue.h"
#include <EEPROM.h>
#include "Encoder.h"
#include "debug.cpp"

#define rows 6
#define cols 6

// Matrix macros
#define linearise(row, col) row*cols + col
#define delineariseRow(location) location / cols
#define delineariseCol(location) location % cols

// Wall macros
#define distance(loc1, loc2) abs(delineariseRow(loc1) - delineariseRow(loc2)) + abs(delineariseCol(loc1) - delineariseCol(loc2))
#define markWall(location, direction) floodArray[location].neighbours |= 1 << direction
#define wallExists(location, direction) bitRead(floodArray[location].neighbours, direction)

// Neighbour macros
#define getNeighbourLocation(location, direction) (byte)((short)location + cellDirectionAddition[direction])  // Calculates the location of neighbour
#define getNeighbourDistanceIfAccessible(location, direction) floodArray[getNeighbourLocation(location, direction)].flood
#define getNeighbourDistance(location, direction) wallExists(location, direction) ? 255 : getNeighbourDistanceIfAccessible(location, direction)

// Direction macros
#define updateDirection(currentDirection, turn) *currentDirection = (*currentDirection + turn) % 4  // Updates the passed direction

#define north 0
#define east 1
#define south 2
#define west 3

#define rightTurn 1
#define uTurn 2
#define leftTurn 3

#define leftSensor 0
#define centreSensor 1
#define rightSensor 2

#define encoderStepsMenu 30

#define wallThreshold 8

float sensorValue[3];
int sensorPins[3] = {A2, A1, A0}; // Left, Front, Right
Encoder myEnc1(3, 7);
Encoder myEnc2(2, 4);
#define button 13

struct cell {
  byte flood;
  byte neighbours;
  byte visited;
};

cell floodArray[rows * cols];  // This array stores the flood value and neighbour data for all the cells

byte targetCells[4] = {35,35,35,35};  // This array stores the target cells

CircularBufferQueue floodQueue(512);  // This queue stores the cells that need to be flooded

byte startCell = 1;
byte currentCell, targetCell;                                           // startCell is the starting position
byte startDir = 1;
byte leftDir, currentDir, rightDir, nextLeftDir, nextDir, nextRightDir;  // startDir is the initial orientation of the bot

short cellDirectionAddition[4] = { rows, 1, -rows, -1 };  // The location of a neighbouring cell can be obtained using the values in this dictionary
byte updateDirectionTurnAmount[4] = { 0, rightTurn, uTurn, leftTurn };
byte targetScoreFromDirection[4] = { 0, 1, 2, 1 };

byte readingCellLoc, readingCellDistance, readingCellScore, minNeighbourDistance, targetRelativeDirection, targetScore;
byte distanceFromTarget = 1;

byte resetMazeEEPROM = 0;  // This determines whether or not the maze data stored in the EEPROM should be reset
byte menu = 0;             // This determines which value the encoder updates
short change;
byte* values[7] = { &startCell, &(targetCells[0]), &(targetCells[1]), &(targetCells[2]), &(targetCells[3]), &startDir, &resetMazeEEPROM };

long newPosition1, newPosition2, oldPosition1, oldPosition2;


void flood() {
  floodQueue.enqueue(currentCell);
  while (!floodQueue.isEmpty()) {
    readingCellLoc = *floodQueue.dequeue();
    if (isEnclosed(readingCellLoc)) continue;
    readingCellDistance = floodArray[readingCellLoc].flood;
    minNeighbourDistance = 255;
    for (byte i = 0; i < 4; i++) {
      minNeighbourDistance = min(minNeighbourDistance, getNeighbourDistance(readingCellLoc, i));
    }
    if (minNeighbourDistance != readingCellDistance - 1) {
      floodArray[readingCellLoc].flood = minNeighbourDistance + 1;
      for (byte i = 0; i < 4; i++) {
        if (isNeighbourValid(readingCellLoc, i)) {
          if (!isDestination(getNeighbourLocation(readingCellLoc, i))) {
            floodQueue.enqueue(getNeighbourLocation(readingCellLoc, i));
          }
        }
      } 
    }
  }
}

void updateTargetCell() {
  minNeighbourDistance = getNeighbourDistance(currentCell, 0);
  targetScore = 3;
  for (byte i = 0; i < 4; i++) {
    if (!wallExists(currentCell, i)) {
      readingCellLoc = getNeighbourLocation(currentCell, i);
      readingCellDistance = getNeighbourDistance(currentCell, i);
      readingCellScore = targetScoreFromDirection[getTargetRelativeDirection(readingCellLoc)];
      if ((readingCellDistance < minNeighbourDistance) || ((readingCellDistance == minNeighbourDistance) && (readingCellScore < targetScore))) {
        minNeighbourDistance = readingCellDistance;
        targetScore = readingCellScore;
        targetCell = readingCellLoc;
      }
    }
  }

  targetRelativeDirection = getTargetRelativeDirection(targetCell);

  updateDirection(&nextLeftDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&nextDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&nextRightDir, updateDirectionTurnAmount[targetRelativeDirection]);

  distanceFromTarget = 1;

  while (isNeighbourValid(targetCell, nextDir)) {
    readingCellLoc = getNeighbourLocation(targetCell, nextDir);
    if (isTunnel(readingCellLoc) && floodArray[readingCellLoc].flood == floodArray[targetCell].flood - 1) {
      targetCell = readingCellLoc;
      distanceFromTarget++;
    } else break;
  }
}

void goToTargetCell() {
  if (targetRelativeDirection == north) {
  } else if (targetRelativeDirection == east) {
    turn(-90, 10);
  } else if (targetRelativeDirection == south) {
    turn(180, 10);
  } else if (targetRelativeDirection == west) {
    turn(90, 10);
  }
  moveForward(distanceFromTarget, 100);

  updateDirection(&leftDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&currentDir, updateDirectionTurnAmount[targetRelativeDirection]);
  updateDirection(&rightDir, updateDirectionTurnAmount[targetRelativeDirection]);
}

void updateWalls() {
  readWall();
  if (sensorValue[leftSensor] < wallThreshold) {
    markWall(currentCell, leftDir);
    if (isNeighbourValid(currentCell, leftDir)) {
      markWall(getNeighbourLocation(currentCell, leftDir), (leftDir + 3) % 4);
    }
  }
  if (sensorValue[centreSensor] < wallThreshold) {
    markWall(currentCell, currentDir);
    if (isNeighbourValid(currentCell, currentDir)) {
      markWall(getNeighbourLocation(currentCell, currentDir), (currentDir + 3) % 4);
    }
    //alignFront();
  }
  if (sensorValue[rightSensor] < wallThreshold) {
    markWall(currentCell, rightDir);
    if (isNeighbourValid(currentCell, rightDir)) {
      markWall(getNeighbourLocation(currentCell, rightDir), (rightDir + 3) % 4);
    }
  }
}

bool isNeighbourValid(byte location, byte direction) {
  if (direction == north) return delineariseRow(location) > 0;
  else if (direction == east) return delineariseCol(location) < (cols - 1);
  else if (direction == south) return delineariseRow(location) < (rows - 1);
  else if (direction == west) return delineariseCol(location) > 0;
}

byte getTargetAbsoluteDirection(byte target) {
  short diff = (short)target - (short)currentCell;
  if (diff == -rows) return north;
  if (diff == 1) return east;
  if (diff == rows) return south;
  if (diff == -1) return west;
}

byte getTargetRelativeDirection(byte target) {
  return (getTargetAbsoluteDirection(target) + 4 - currentDir) % 4;
}

bool isDestination(byte location) {
  return floodArray[location].flood == 0;
}

bool isEnclosed(byte location) {
  // 15 is 00001111 in binary, which means that there are walls in 4 all 4 directions of the cell
  return floodArray[location].neighbours == 15;
}

bool isTunnel(byte location) {
  return (!wallExists(location, nextDir)) && wallExists(location, nextLeftDir) && wallExists(location, nextRightDir) && floodArray[location].visited;
}

void initialiseDirections() {
  currentDir = startDir;
  leftDir = (currentDir + 3) % 4;
  rightDir = (currentDir + 1) % 4;
  nextLeftDir = leftDir;
  nextDir = currentDir;
  nextRightDir = rightDir;
}

//////////////////////////////////
/////////////EEPROM//////////////
////////////////////////////////

void updateMazeValuesFromEEPROM() {
  for (byte i = 0; i < (rows * cols); i++) {
    floodArray[i].flood = EEPROM.read(i);
    if (i == 35) break;
  }
  for (byte i = 0; i < (rows * cols); i++) {
    floodArray[i].neighbours = EEPROM.read((rows * cols) + (short)i);
    if (i == 35) break;
  }
  for (byte i = 0; i < (rows * cols); i++) {
    floodArray[i].visited = EEPROM.read((2 * rows * cols) + (short)i);
    if (i == 35) break;
  }
  for (byte i = 0; i < 6; i++) {
    *(values[i]) = EEPROM.read((3 * rows * cols) + (short)i);
  }
}

void updateMazeValuesInEEPROM() {
  for (byte i = 0; i < (rows * cols); i++) {
    EEPROM.write(i, floodArray[i].flood);
    if (i == 35) break;
  }
  for (byte i = 0; i < (rows * cols); i++) {
    EEPROM.write((rows * cols) + (short)i, floodArray[i].neighbours);
    if (i == 35) break;
  }
  for (byte i = 0; i < (rows * cols); i++) {
    EEPROM.write((2 * rows * cols) + (short)i, floodArray[i].visited);
    if (i == 35) break;
  }
  for (byte i = 0; i < 6; i++) {
    EEPROM.write((3 * rows * cols) + (short)i, *(values[i]));
  }
}

  void resetMazeValuesInEEPROM() {
  for (byte i = 0; i < (rows * cols); i++) {
    floodArray[i].flood = 255;
    for (byte j = 0; j < 4; j++) floodArray[i].flood = min(floodArray[i].flood, distance(i, targetCells[j]));
    floodArray[i].neighbours = 0;
    floodArray[i].visited = 0;
    if (delineariseRow(i) == 0) markWall(i, north);
    if (delineariseCol(i) == 0) markWall(i, west);
    if (delineariseRow(i) == (rows - 1)) markWall(i, south);
    if (delineariseCol(i) == (cols - 1)) markWall(i, east);
    if (i == 35) break;
  }
  updateMazeValuesInEEPROM();
}

//////////////////////////////////
///////////Serial SETUP////////////
////////////////////////////////

void updateEncoder() {
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();
  if (abs(newPosition1 - oldPosition1) >= encoderStepsMenu) {
    menu += (newPosition1 - oldPosition1) / encoderStepsMenu;
    oldPosition1 = newPosition1;
    if (menu > 100) menu = 0;
    if (menu > 6) menu = 6;
    displayMenu();
  }
  if (abs(newPosition2 - oldPosition2) >= encoderStepsMenu) {
    change = (newPosition2 - oldPosition2) / encoderStepsMenu;
    oldPosition2 = newPosition2;
    if (0 <= menu && menu <= 4) {
      *(values[menu]) = (*(values[menu]) + change) % (rows * cols);
    } else if (menu == 5) {
      *(values[menu]) = (*(values[menu]) + change) % 4;
    } else if (menu == 6) {
      *(values[menu]) = (*(values[menu]) + change);
      if(*(values[menu]) > 200) *(values[menu]) = 0;
      else if (*(values[menu]) > 1) *(values[menu]) = 1;
    }
    displayMenu();
  }
}

void displayMenu() {
  
  if (menu == 0) {
    Serial.println("Start");
    Serial.print(delineariseRow(*(values[menu])));
    Serial.print(", ");
    Serial.println(delineariseCol(*(values[menu])));
  } else if (menu == 1) {
    Serial.println("End 1");
    Serial.print(delineariseRow(*(values[menu])));
    Serial.print(", ");
    Serial.println(delineariseCol(*(values[menu])));
  } else if (menu == 2) {
    Serial.println("End 2");
    Serial.print(delineariseRow(*(values[menu])));
    Serial.print(", ");
    Serial.println(delineariseCol(*(values[menu])));
  } else if (menu == 3) {
    Serial.println("End 3");
    Serial.print(delineariseRow(*(values[menu])));
    Serial.print(", ");
    Serial.println(delineariseCol(*(values[menu])));
  } else if (menu == 4) {
    Serial.println("End 4");
    Serial.print(delineariseRow(*(values[menu])));
    Serial.print(", ");
    Serial.println(delineariseCol(*(values[menu])));
  } else if (menu == 5) {
    Serial.println("Direction");
    Serial.println((*(values[menu]) == north) ? "North" : (*(values[menu]) == east)  ? "East"
                                                      : (*(values[menu]) == south) ? "South"
                                                                                   : "West");
  } else if (menu == 6) {
    Serial.println("Reset Maze");
    Serial.println((*(values[menu])) ? "Yes" : "No");
  }
}



//////////////////////////////////
//////////////Debug//////////////
/////////////////////////////////


void printFloodArray() {
  for (byte i = 0; i < rows; i++) {
    for (byte j = 0; j < cols; j++) {
      byte flood = floodArray[linearise(i, j)].flood;
      Serial.print(flood);
      if (flood < 10) Serial.print(" , ");
      else Serial.print(", ");
    }
    Serial.println();
  }
}

void testQueue(CircularBufferQueue q) {
  Serial.print("Is empty? ");
  Serial.println(q.isEmpty() ? "Empty" : "Not Empty");
  for (byte i = 0; i < 256; i++) {
    Serial.print(i);
    Serial.print(", ");
    Serial.println(q.enqueue(i) ? "Enqueued" : "Failed");
    if (i == 255) break;
  }
  Serial.print("Does enqueue? ");
  Serial.println(q.enqueue(1) ? "Enqueued" : "Failed");
  Serial.print("Is empty? ");
  Serial.println(q.isEmpty() ? "Empty" : "Not Empty");
  while (q.peek()) {
    Serial.print(*q.dequeue());
    Serial.println(" dequeued");
  }
  Serial.print("Is empty? ");
  Serial.println(q.isEmpty() ? "Empty" : "Not Empty");
}
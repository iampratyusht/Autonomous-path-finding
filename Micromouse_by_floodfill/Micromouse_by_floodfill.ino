#include "TA6586.h"
#include "floodfill.cpp"


#define D0 10
#define D1 9
#define D2 6
#define D3 5
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(D0, D1, offsetA);
Motor motor2 = Motor(D3, D2, offsetB);

int P, D, I, previousError, PIDvalue, error;
int pos1,pos2;
int lsp = 0;
int rsp = 0;
double encStart1 = 0;
double encStart2 = 0;

const float WHEEL_DIAMETER = 34.0; // in mm
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER; // in mm
const int CPR = 810; // Counts Per Revolution at output shaft
const float CELL_SIZE = 150.0;
const float TRACK_WIDTH = 90.0; // Distance between wheels in mm 


void moveForward(long blocks, int speed) {
  restartEnc();
  float Kp = 0.5;
  float Kd = 0.3;
  float Ki = 0.0;  

  // Compute steps required for given blocks
  long stepsPerBlock = (CPR / WHEEL_CIRCUMFERENCE) * CELL_SIZE;
  long steps = blocks * stepsPerBlock;
  float currentSpeed = 100;
  pos1 = 0;
  pos2 = 0;
  encStart1 = abs(myEnc1.read());
  encStart2 = abs(myEnc2.read());

  while (pos1 < steps && pos2 < steps) {
    encUpdate();
    if (abs(pos1 + steps) < 2000 || abs(pos2 + steps) < 2000) {
      if (currentSpeed > 60) currentSpeed = currentSpeed - 5;
    } else if (currentSpeed < speed) {
      currentSpeed = currentSpeed + 5;
    }
    long error = abs(pos1 - pos2);
    P = error;
    I = I + error;
    D = error - previousError;
    
    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    rsp = currentSpeed - PIDvalue;
    lsp = currentSpeed + PIDvalue;

    if (rsp > 200) rsp = 200;
    if (rsp < 60) rsp = 60;
    if (lsp > 200) lsp = 200;
    if (lsp < 60) lsp = 60;
    motor1.drive(rsp);
    motor2.drive(lsp);
  }
  motor1.drive(0);
  motor2.drive(0);
}
void turn(int angle, int speed) {
  restartEnc();
  float Kp = 0.7;
  float Kd = 0.9;
  float ks = 0.9;
  // Calculate steps for the given angle
  float arcLength = 3.14159 * TRACK_WIDTH * abs(angle) / 360.0;
  float steps = (arcLength * CPR) / WHEEL_CIRCUMFERENCE;
  float currentSpeed = speed;
  pos1 = 0;
  pos2 = 0;
  encStart1 = abs(myEnc1.read());
  encStart2 = abs(myEnc2.read());

  while (pos1 <= ks*steps && pos2 <= ks*steps) {
    encUpdate();
    if (abs(encStart1 + steps - newPosition1) < 2000){ 
      if (currentSpeed > 135) currentSpeed = currentSpeed - 5;
    } else {
      currentSpeed = currentSpeed + 5;
      if (currentSpeed > speed) currentSpeed = speed;
    }
    long error = abs(pos1 - pos2);
    PIDvalue = (Kp * error) + (Kd * (error - previousError));
    previousError = error;

    lsp = currentSpeed + PIDvalue;
    rsp = currentSpeed - PIDvalue;

    if (lsp > 200) lsp = 200;
    if (lsp < 60) lsp = 60;
    if (rsp > 200) rsp = 200;
    if (rsp < 60) rsp = 60;

    if (angle > 0) {
      motor1.drive(-rsp);
      motor2.drive(lsp);
    } else {
      motor1.drive(rsp);
      motor2.drive(-lsp);
    }
  }
  // for(int i=0;i<100;)

  motor1.drive(0);
  motor2.drive(0);  
}

void encUpdate() {
  newPosition1 = abs(myEnc1.read());
  newPosition2 = abs(myEnc2.read());
  pos1 = newPosition1 - encStart1;
  pos2 = newPosition2 - encStart2;
  if (newPosition1 != oldPosition1) oldPosition1 = newPosition1;
  if (newPosition2 != oldPosition2) oldPosition2 = newPosition2;
}

void readWall() {
  for (int i = 0; i < 3; i++) {
    int rawValue = analogRead(sensorPins[i]);
    float voltage = rawValue * (5.0 / 1023.0);
    sensorValue[i] = 12.08 / (voltage - 0.2);
    if((sensorValue[i])>30) sensorValue[i] = 30;
    if(abs(sensorValue[i])<4) sensorValue[i] = 4;
  }
}

void resetEnc() {
  newPosition1 = 0;
  oldPosition1 = -999;
  newPosition2 = 0;
  oldPosition2 = -999;
}

void restartEnc(){
  myEnc1.write(0);
  myEnc2.write(0);
}

void alignFront() {
  float speedFactor = 0.4;
  int alignUpperSpeed = 60;

  for (int i = 0; i < 1500; i++) {
    readWall();
    int frontError = 15 - sensorValue[1]; // Target front distance is 15 cm
    int lrError = sensorValue[0] - sensorValue[2];

    int leftSpeed = speedFactor * (frontError - lrError);
    int rightSpeed = speedFactor * (frontError + lrError);

    if (leftSpeed < -alignUpperSpeed) leftSpeed = -alignUpperSpeed;
    if (leftSpeed > alignUpperSpeed) leftSpeed = alignUpperSpeed;
    if (rightSpeed < -alignUpperSpeed) rightSpeed = -alignUpperSpeed;
    if (rightSpeed > alignUpperSpeed) rightSpeed = alignUpperSpeed;

    motor1.drive(leftSpeed);
    motor2.drive(rightSpeed);
  }
  motor1.drive(0);
  motor2.drive(0);
}
void setup() {
  Serial.begin(9600);
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  for (int i = 0; i < 3; i++) pinMode(sensorPins[i], INPUT);
  pinMode(button, INPUT_PULLUP); 
  //updateMazeValuesFromEEPROM();  
  Serial.println("Configure");

  while (!digitalRead(button)){}
  delay(500);
  newPosition1 = myEnc1.read(), newPosition2 = myEnc2.read(), oldPosition1 = myEnc1.read(), oldPosition2 = myEnc2.read();
  displayMenu();

  while (!digitalRead(button)) updateEncoder();  
  Serial.println("Saving");

  if (resetMazeEEPROM) resetMazeValuesInEEPROM();
  else updateMazeValuesInEEPROM();
  resetEnc();
  
  // Serial.println("CALLIBRATE");
  // while (digitalRead(11)) {};
  // delay(1000);
  // calibrate();
}

void loop() {
  currentCell = startCell;
  initialiseDirections();  
  
  while (!digitalRead(button)) {}  
  Serial.println("Running");
  delay(1000);
  

  while (currentCell != targetCells[0] && currentCell != targetCells[1] && currentCell != targetCells[2] && currentCell != targetCells[3]){
    updateWalls();
    flood();
    updateTargetCell();
    Serial.println(targetCell);
    goToTargetCell();
    currentCell = targetCell;
    floodArray[currentCell].visited = 1;
    delay(1000);
  }
  updateMazeValuesInEEPROM();
}



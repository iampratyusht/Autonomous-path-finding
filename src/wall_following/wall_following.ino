#include "TA6586.h"
#include "Encoder.h"

#define D0 10
#define D1 9
#define D2 6
#define D3 5
#define button 13
Encoder myEnc1(3, 7);
Encoder myEnc2(2, 4);

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(D0, D1, offsetA);
Motor motor2 = Motor(D3, D2, offsetB);

long newPosition1, newPosition2, oldPosition1, oldPosition2;
int pos1, pos2;
int previousError, P, I, D, PIDvalue, error;
int lsp = 0;
int rsp = 0;
int sensorPins[3] = { A2, A1, A0 };  // Left, Front, Right
float sensorValue[3];
float thresh = 20.0;

double encStart1 = 0;
double encStart2 = 0;

const float WHEEL_DIAMETER = 34.0;                           // in mm
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;  // in mm
const int CPR = 810;                                         // Counts Per Revolution at output shaft
const float CELL_SIZE = 200.0;
const float TRACK_WIDTH = 90.0;  // Distance between wheels in mm
int speed1 = 100;
int speed2 = 100;

void moveForward(long blocks, int speed) {
  restartEnc();
  // float Kp = 0.7;
  // float Kd = 0.9;
  float Kp = 0.7;
  float Kd = 0.9;
  float Kw = 0.0;

  // Compute steps required for given blocks
  long stepsPerBlock = (CPR / WHEEL_CIRCUMFERENCE) * CELL_SIZE;
  long steps = blocks * stepsPerBlock;
  pos1 = 0;
  pos2 = 0;
  float currentSpeed = speed;
  encStart1 = abs(myEnc1.read());
  encStart2 = abs(myEnc2.read());

  while (pos1 < steps) {
    encUpdate();
    readWall();
    if (abs(pos1 + steps) < 2000) {
      if (currentSpeed > 150) currentSpeed = currentSpeed - 5;
    } else if (currentSpeed < speed) {
      currentSpeed = currentSpeed + 5;
    }

    long error = abs(pos1 - pos2);
    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Kd * D);
    previousError = error;
    long wallError;


    wallError = Kw * (sensorValue[0] - sensorValue[2]);  // 50 is offset between left and right side- right being greater
    //if (sensorValue[0] < 8  || sensorValue[2] < 300) wallError = 0;
    lsp = currentSpeed + PIDvalue - wallError;
    rsp = currentSpeed - PIDvalue + wallError;

    rsp = currentSpeed - PIDvalue;
    lsp = currentSpeed + PIDvalue;

    if (rsp > 200) rsp = 200;
    if (rsp < 60) rsp = 60;
    if (lsp > 200) lsp = 200;
    if (lsp < 60) lsp = 60;
    //Serial.println(error);
    //Serial.println(lsp);
    // Serial.println(lsp);
    motor1.drive(rsp);
    motor2.drive(lsp);
  }

  motor1.drive(0);
  motor2.drive(0);
}

void turn(int angle, int speed) {
  restartEnc();
  float Kp = 0.9;
  float Kd = 0.7;
  float ks = 0.7;
  // Calculate steps for the given angle
  float arcLength = 3.14159 * TRACK_WIDTH * abs(angle) / 360.0;
  float steps = (arcLength * CPR) / WHEEL_CIRCUMFERENCE;
  float currentSpeed = speed;
  pos1 = 0;
  pos2 = 0;
  encStart1 = abs(myEnc1.read());
  encStart2 = abs(myEnc2.read());

  while (pos1 <= ks * steps && pos2 <= ks * steps) {
    encUpdate();
    if (abs(encStart1 + steps - newPosition1) < 2000) {
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
  motor1.drive(0);
  motor2.drive(0);
}
void readWall() {
  for (int i = 0; i < 3; i++) {
    int rawValue = analogRead(sensorPins[i]);
    float voltage = rawValue * (5.0 / 1023.0);
    sensorValue[i] = abs(12.08 / (voltage - 0.2));
    // sensorValue[i] = constrain(sensorValue[i], 4, 30);
    if ((sensorValue[i] >= 4.0) && (sensorValue[i]) <= 30.0) {
      sensorValue[i] = sensorValue[i];
    } else {
      sensorValue[i] = 100.0;
    }
  }
}
void encUpdate() {
  newPosition1 = abs(myEnc1.read());
  newPosition2 = abs(myEnc2.read());

  pos1 = newPosition1 - encStart1;
  pos2 = newPosition2 - encStart2;

  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
  }
  if (newPosition2 != oldPosition2) {
    oldPosition2 = newPosition2;
  }
}
void resetEnc() {
  newPosition1 = 0;
  oldPosition1 = -999;
  newPosition2 = 0;
  oldPosition2 = -999;
}
void restartEnc() {
  myEnc1.write(0);
  myEnc2.write(0);
}
void setup() {
  Serial.begin(9600);
  newPosition1 = myEnc1.read(), newPosition2 = myEnc2.read(), oldPosition1 = myEnc1.read(), oldPosition2 = myEnc2.read();
  for (int i = 0; i < 3; i++) pinMode(sensorPins[i], INPUT);
  pinMode(button, INPUT_PULLUP);
  resetEnc();
}

int numJunction = 0;
void loop() {
  delay(200);
  readWall();
  delay(100);
  if ((sensorValue[0] > thresh) && (sensorValue[2] > thresh) && (sensorValue[1] > thresh) && (numJunction < 2)) {
    moveForward(1.3, speed2);  //INTER MAZE TUNNEL
    delay(200);
    turn(20, 60);
    delay(200);
    numJunction++;
  } else if (sensorValue[2] > thresh) {
    turn(72, speed1);  //Right TURN
    delay(200);
    moveForward(1, speed2);
    delay(200);
    turn(20, 60);
    delay(200);
  } else if (sensorValue[2] < thresh && sensorValue[1] > thresh) {
    moveForward(1, speed2);  //forward turn
    delay(200);
    turn(20, 60);
    delay(200);
  } else if (sensorValue[0] > thresh && sensorValue[1] < thresh && sensorValue[2] < thresh) {
    turn(-72, speed1);
    delay(200);
    moveForward(1, speed2);  //left TURN
    delay(200);
    turn(20, 60);
    delay(200);
  } else if (sensorValue[0] < thresh && sensorValue[2] < thresh && sensorValue[1] < thresh) {
    delay(200);
    turn(180, speed1);
    delay(200);
    moveForward(1, speed2);  //TURN AROUND
    delay(200);
    turn(20, 60);
    delay(200);
  }
}

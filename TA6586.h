#ifndef TA6586_h
#define TA6586_h

#include <Arduino.h>

#define DEFAULTSPEED 255

class Motor
{
  public:
    Motor(int In1pin, int In2pin, int offset);      
    void drive(int speed);  
    void drive(int speed, int duration);  
    void brake();
  private:
	int In1, In2, Offset;
	void fwd(int speed);
	void rev(int speed);
};

void forward(Motor motor1, Motor motor2, int speed);
void forward(Motor motor1, Motor motor2);
void back(Motor motor1, Motor motor2, int speed);
void back(Motor motor1, Motor motor2);
void left(Motor left, Motor right, int speed);
void right(Motor left, Motor right, int speed);
void brake(Motor motor1, Motor motor2);
#endif
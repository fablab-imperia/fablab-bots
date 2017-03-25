#include "AFMotor.h"

//Circa 1 giro/s a 100
//Circa 20 cm di circonferenza ruota
//Circa 20 cm/s a 200
//Circa 10 cm/s a 100
//Circa 10 cm raggio macchina


#define LINEAR_SPEED 25
#define DESIRED_POWER_TO_MOTORS 100

#define CAR_RADIUS 8.0f

#define ANGULAR_SPEED (((float)LINEAR_SPEED) / (CAR_RADIUS*2.3))
#define RIGHT_SIDE_SLOWNESS_COMPENSATION 1.14f 

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);


void setup() {

 // allForward();
   int leftSideSpeed = DESIRED_POWER_TO_MOTORS;
   int rightSideSpeed = (int)((float)leftSideSpeed * RIGHT_SIDE_SLOWNESS_COMPENSATION);
   
   leftSideSetSpeed(leftSideSpeed);
   rightSideSetSpeed(rightSideSpeed);
 
 //rotate90DegreesLeft();
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
}

void loop() {

  goForward(100);
  delay(1000);
  rotateRight(90);
  delay(1000);
  goForward(100);
  delay(1000);
  rotateRight(90);
  delay(1000);
  goForward(100);
  delay(1000);
  rotateRight(90);
  delay(1000);
  goForward(100);
  allStop();
  delay(10000);
  
}

float angularSpeedInRadiansPerSec(int desiredLinearSpeed)  {
  float angularSpeed = ((float)desiredLinearSpeed) / CAR_RADIUS;
  return angularSpeed;
}

int millisToWaitForRotationDegrees(int degs, float angularSpeedInRadsPerSec) {
   float rads = (3.14 / 180) * degs;
   float timeInSecs = rads / angularSpeedInRadsPerSec;
   return (int) (timeInSecs * 1000);
}

void goForward(int distanceInCm) {
  allForward();
  int millisToWait = millisToWaitForLinearDistance(distanceInCm);
  delay(millisToWait);
  allStop();
}

int millisToWaitForLinearDistance(int distanceInCm) {
  float t = (float)distanceInCm / LINEAR_SPEED;
  int millisecs = (int)(t * 1000); 
  return millisecs;
}

void rotateLeft(int degs) {
  rotateLeft();
  int millisToWait = millisToWaitForRotationDegrees(degs, ANGULAR_SPEED);
  delay(millisToWait);
  allStop();
}


void rotateRight(int degs) {
  rotateRight();
  int millisToWait = millisToWaitForRotationDegrees(degs, ANGULAR_SPEED);
  delay(millisToWait);
  allStop();
}


void rotateLeft() {

  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
   
}

void rotateRight() {

  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
   
}


void leftSideSetSpeed(unsigned int speed){
 
  motor4.setSpeed(speed);
  motor1.setSpeed(speed);

}


void rightSideSetSpeed(unsigned int speed){
 
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);

}

void allSetSpeed(unsigned int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

void allForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}


void allBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void allStop(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


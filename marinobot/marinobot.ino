/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->	http://www.adafruit.com/products/1438
  https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>

#define SIGNAL_TIMEOUT 3000
#define MAX_SPEED 150

int rxPin = 3;
int txPin = 2;
SoftwareSerial bluetooth(rxPin, txPin);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
// You can also make another motor on port M2
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);

unsigned long last_signal = 0;
uint8_t current_speed = MAX_SPEED;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  bluetooth.begin(9600);
  Serial.println("Adafruit Motorshield v2");

  AFMS.begin();  // create with the default frequency 1.6KHz

  rightMotor->setSpeed(current_speed);
  leftMotor->setSpeed(current_speed);
  /*
    // Set the speed to start, from 0 (off) to 255 (max speed)
    rightMotor->setSpeed(0);
    leftMotor->setSpeed(255);
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);

    delay(5000);

    rightMotor->setSpeed(255);
    leftMotor->setSpeed(0);

    delay(5000);

    // turn on motor
    rightMotor->run(RELEASE);
    leftMotor->run(RELEASE);*/
}

void loop() {
  char c = bluetooth.read();
  switch (c) {
    case 'S':
      Serial.println("STOP");
      rightMotor->run(RELEASE);
      leftMotor->run(RELEASE);
      last_signal = millis();
      break;
    case 'F':
      Serial.println("FORWARD");
      rightMotor->run(FORWARD);
      leftMotor->run(FORWARD);
      last_signal = millis();
      break;
    case 'B':
      Serial.println("BACKWARD");
      rightMotor->run(BACKWARD);
      leftMotor->run(BACKWARD);
      last_signal = millis();
      break;
    case 'R':
      Serial.println("RIGHT");
      rightMotor->run(BACKWARD);
      leftMotor->run(FORWARD);
      last_signal = millis();
      break;
    case 'L':
      Serial.println("LEFT");
      rightMotor->run(FORWARD);
      leftMotor->run(BACKWARD);
      last_signal = millis();
      break;
    default:
      // Set speed
      if (c > '0' && c < '9') {
        current_speed = MAX_SPEED * (c - 0x30) / 10;  // 0x30 is hex for ascii code of 0, 0x31 for 1, etc...
        Serial.print("Current speed: ");
        Serial.println(current_speed);
        rightMotor->setSpeed(current_speed);
        leftMotor->setSpeed(current_speed);
      } else if (c == 'q') {
        current_speed = MAX_SPEED;                    // For some reason q is the max speed
        Serial.print("Current speed: ");
        Serial.println(current_speed);
        rightMotor->setSpeed(current_speed);
        leftMotor->setSpeed(current_speed);
      } else if (c != -1) {
        Serial.println(c);
      }
      break;
  }

  if (millis() - last_signal > SIGNAL_TIMEOUT) {
    Serial.println("SIGNAL LOST");
    rightMotor->run(RELEASE);
    leftMotor->run(RELEASE);
  }
  /*
    uint8_t i;

    Serial.print("tick");

    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);
    for (i=0; i<255; i++) {
    rightMotor->setSpeed(i);
    leftMotor->setSpeed(i);
    delay(10);
    }
    for (i=255; i!=0; i--) {
    rightMotor->setSpeed(i);
    leftMotor->setSpeed(i);
    delay(10);
    }

    Serial.print("tock");

    rightMotor->run(BACKWARD);
    leftMotor->run(BACKWARD);
    for (i=0; i<255; i++) {
    rightMotor->setSpeed(i);
    leftMotor->setSpeed(i);
    delay(10);
    }
    for (i=255; i!=0; i--) {
    rightMotor->setSpeed(i);
    leftMotor->setSpeed(i);
    delay(10);
    }

    Serial.print("tech");
    rightMotor->run(RELEASE);
    leftMotor->run(RELEASE);
    delay(1000);*/
}

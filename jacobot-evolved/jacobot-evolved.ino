git#include <AFMotor.h>


AF_DCMotor M1(1);

AF_DCMotor M2(2);

AF_DCMotor M3(3);

//AF_DCMotor M4(4);


void setup() {
    M1.setSpeed(255);
    M2.setSpeed(255);
    M3.setSpeed(255);
  //  M4.setSpeed(255);
}

void loop() {
    M1.run(FORWARD);
    M2.run(FORWARD);
    M3.run(FORWARD);
    //M4.run(FORWARD);

    delay(5000);  

    M1.run(RELEASE);
    M2.run(RELEASE);
    M3.run(RELEASE);
   // M4.run(RELEASE);
    
    delay(5000);

    M1.run(BACKWARD);
    M2.run(BACKWARD);
    M3.run(BACKWARD);
    //M4.run(BACKWARD);

    delay(5000);

    M1.run(RELEASE);
    M2.run(RELEASE);
    M3.run(RELEASE);
    //M4.run(RELEASE);
    
    delay(5000);


}

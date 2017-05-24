#include "NewPing.h"
#include "AFMotor.h"
#include <SoftwareSerial.h>

/* BEHAVIOURS*/
#define ROOMBA 0
#define SQUARE 1
#define FOLLOWER 2
#define LISTEN 3


int rxPin = 2;
int txPin = A0;

SoftwareSerial bluetooth(rxPin, txPin);

int last_behaviour = -1;


/***********************************************
 * Robot specific parameters                   
 *    Circa 1 giro/s a 100
 *     Circa 20 cm di circonferenza ruota
 *    Circa 20 cm/s a 200
 *     Circa 10 cm/s a 100
 *     Circa 10 cm raggio macchina*
 ***********************************************/
#define MOTOR_TO_SPEED 4
#define DESIRED_POWER_TO_MOTORS 180
#define CAR_RADIUS 8.0f
#define RIGHT_SIDE_SLOWNESS_COMPENSATION 1.18f 

// --- Pins for ULTRSONIC SENSOR
#define ECHO_PIN A5
#define TRIGGER_PIN A4


/***********************************************
 *    LED MATRIX PARAMETERS 
 ***********************************************/
#define DIN A3
#define CLK A1
#define LOAD A2
// --- MAX7219 COMMANDS ---
#define CMD_NOOP 0x0000
#define CMD_DECODE_MODE 0x0900
#define CMD_INTENSITY 0x0A00
#define CMD_SCAN_LIMIT 0x0B00
#define CMD_SHUTDOWN 0x0C00
#define CMD_DISPLAY_TEST 0x0F00

/******************************************
 ***          GLOBALS                   ***
/******************************************/

//--MOTORS
AF_DCMotor motor1(1, MOTOR12_8KHZ);
AF_DCMotor motor2(2, MOTOR12_8KHZ);
AF_DCMotor motor3(3, MOTOR12_8KHZ);
AF_DCMotor motor4(4, MOTOR12_8KHZ);

//--- SONAR
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 400);

// --- CURRENT STATE
int jacobot_current_state;

// --- SPEED (LINEAR - ANGULAR)
unsigned long LINEAR_SPEED = (unsigned long)((float)DESIRED_POWER_TO_MOTORS)/MOTOR_TO_SPEED;
float ANGULAR_SPEED = (float)(((float)LINEAR_SPEED) / (CAR_RADIUS*2.8f));




/***********************************************
 * BEHAVIOURS AND Behaviour specific parameters                   
 ***********************************************/

  #define ROOMBA_OBSTACLE_THRESHOLD_DISTANCE_CM 20

  //--- STATES FOR ROOMBA
  #define STATE_WANDERING 1
  #define STATE_AVOID_CRASH 2 

  

  bool isCrashing() {

   unsigned int distanceCm = distanceToObstacle();
  
   if (distanceCm > 0 && distanceCm < ROOMBA_OBSTACLE_THRESHOLD_DISTANCE_CM)  {
      Serial.print("CRASH: distance ");
      Serial.println(distanceCm);
      return true;
   }     

   Serial.print("NO CRASH: distance ");
   Serial.println(distanceCm);
   
   return false;
}

void roomba_style(){
   Serial.print("CURRENT STATE IS: ");
   Serial.println((jacobot_current_state == STATE_WANDERING)? "WANDERING" : "AVOID CRASH");
   
   switch (jacobot_current_state) {
    case STATE_WANDERING:
        if (isCrashing()) 
        {
          
            jacobot_current_state = STATE_AVOID_CRASH;
            show_sad_face();
            allStop();
            delay(1000);
        } 
        
    break;
    case STATE_AVOID_CRASH:
        int back = random(10,40);
        goBackward(back);
        delay(500);
        
        int dir = random(0,2);
        int angle = random(70,181);
        
        //change direction randomly
        if (dir == 0)
          rotateLeft(angle);
        else 
          rotateRight(angle);
        
        delay(1000);
        jacobot_current_state = STATE_WANDERING;
        show_smile_face();
        allForward();
    break;
    
  }
}



  #define FOLLOWER_DISTANCE_CM 15
  #define FOLLOWER_ERROR_CM 2

  //--- STATES FOR FOLLOWER
  #define STATE_IDLE 1
  #define STATE_KEEP_DISTANCE 2 

  //Global: how much to move in cm
  int travelling_distance_cm = 0;
  
  void follow_me_but_keep_distance(unsigned int targetDistanceCm){
    

    unsigned int current_distance_cm = 0;

    Serial.print("CURRENT STATE IS: ");
    Serial.println((jacobot_current_state == STATE_IDLE)? "IDLE" : "KEEP_DISTANCE");
    
    switch (jacobot_current_state) {
        case STATE_IDLE:
            current_distance_cm =  distanceToObstacle();   
            Serial.print("DISTANCE TO TARGET (CM): ");
            Serial.println(current_distance_cm);

            Serial.print("DISTANCE TO KEEP: ");
            Serial.println(targetDistanceCm);
            
            if ( 
                  current_distance_cm > targetDistanceCm + FOLLOWER_ERROR_CM 
                  ||
                  current_distance_cm < targetDistanceCm - FOLLOWER_ERROR_CM
                ) 
            {
                 show_sad_face(); 
                 jacobot_current_state = STATE_KEEP_DISTANCE;
                 travelling_distance_cm = current_distance_cm - targetDistanceCm;
                  Serial.print("DISTANCE TO TRAVEL (CM): ");
                  Serial.println(travelling_distance_cm);
            }
           
        break;
        case STATE_KEEP_DISTANCE:
             if (travelling_distance_cm > 0) 
             {
                goForward(travelling_distance_cm);
                 Serial.print("FORWARD (CM): ");
                 Serial.println(travelling_distance_cm);
             }
             else if (travelling_distance_cm < 0) 
             {
                goBackward(-travelling_distance_cm);
                Serial.print("BACKWARD (CM): ");
                Serial.println(travelling_distance_cm);
             }
              show_smile_face(); 
              jacobot_current_state = STATE_IDLE;
              travelling_distance_cm = 0;
                 
        break;
    }

 }


void make_a_square(unsigned int long sideInCm) {
  goForward(sideInCm);
  delay(1000);
  rotateLeft(90);
  delay(1000);
  goForward(sideInCm);
  delay(1000);
  rotateLeft(90);
  delay(1000);
  goForward(sideInCm);
  delay(1000);
  rotateLeft(90);
  delay(1000);
  goForward(sideInCm);
  delay(1000);
  rotateLeft(90);
  delay(5000);
}






// --- LED MATRIX Data ---

char smile[] = 
{ 
  B00100000,
  B01000000,
  B10000000,
  B10000000,
  B10000000,
  B10000000,
  B01000000,
  B00100000
};

char sad[] = 
{
  B10000000,
  B01000000,
  B00100000,
  B00100000,
  B00100000,
  B00100000,
  B01000000,
  B10000000
};


/*********************************************************************
 *           LED MATRIX  FUNCTIONS
 *********************************************************************/


void clk() {
  digitalWrite(CLK, HIGH);
  digitalWrite(CLK, LOW);
}

void load() {
  digitalWrite(LOAD, LOW);
  digitalWrite(LOAD, HIGH);
}

void matrix_init() {
  matrix_write(CMD_SHUTDOWN | 0x01); // Normal operation
  matrix_write(CMD_DISPLAY_TEST | 0x00); // Normal operation
  matrix_write(CMD_DECODE_MODE | 0x00); // No decode
  matrix_write(CMD_INTENSITY | 0x0F); // Highest
  matrix_write(CMD_SCAN_LIMIT | 0x07); // Display 8 lines
}

void matrix_write(word data) {
  word mask = 0x8000;
  for (char i = 0; i < sizeof(word) * 8; i++) {
    digitalWrite(DIN, (data & mask) != 0x0000);
    clk();
    mask >>= 1;
  }
  load();
}

void matrix_img(char *img) {
  word cmd;
  for (word i = 0; i < 8; i++) {
    cmd = i + 1;
    cmd <<= 8;
    cmd &= 0xFF00;
    cmd |= 0x00FF & img[i];

    matrix_write(cmd);
  }

}

void dim() {
  word intensity = CMD_INTENSITY | 0x0F;
  for (char i = 0; i < 0x0F; i++) {
    matrix_write(intensity--);
    delay(80);
  }
}

void light() {
  word intensity = CMD_INTENSITY | 0x00;
  for (char i = 0; i < 0x0F; i++) {
    matrix_write(intensity++);
    delay(80);
  }
}


void show_sad_face(){
  matrix_img(sad);
}

void show_smile_face(){
  matrix_img(smile);
}

/**********************************************************
 *  MOVEMENT FUNCTIONS
 **********************************************************/

float angularSpeedInRadiansPerSec(int desiredLinearSpeed)  {
  float angularSpeed = ((float)desiredLinearSpeed) / CAR_RADIUS;
  return angularSpeed;
}

int millisToWaitForRotationDegrees(int degs, float angularSpeedInRadsPerSec) {
   float rads = (3.14 / 180) * degs;
   float timeInSecs = rads / angularSpeedInRadsPerSec;
   return (int) (timeInSecs * 1000);
}

void goForward(unsigned long distanceInCm) {
  allForward();
  Serial.print("Distanza da percorrere: ");
  Serial.println(distanceInCm);
  unsigned long millisToWait = millisToWaitForLinearDistance(distanceInCm);
  Serial.print("Millisecs di attesa: ");
  Serial.println(millisToWait);
  delay(millisToWait);
  allStop();
}

void goBackward(unsigned long distanceInCm) {
  allBackward();
  unsigned long millisToWait = millisToWaitForLinearDistance(distanceInCm);
  delay(millisToWait);
  allStop();
}

unsigned long millisToWaitForLinearDistance(unsigned long distanceInCm) {
  Serial.print("Linear speed ");
  Serial.println(LINEAR_SPEED);
  unsigned long t = (distanceInCm * 1000L);
  t = t / LINEAR_SPEED;

  return t;
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

unsigned int distanceToObstacle() {
    //retrieves flight time in microsecs
    unsigned long timeMicrosecs = sonar.ping();
    //convert flight time in cm
    unsigned int distanceCm  = NewPing::convert_cm(timeMicrosecs);
    
    return distanceCm;
}  



void setup() {
   bluetooth.begin(9600);
   pinMode(DIN, OUTPUT);
   pinMode(CLK, OUTPUT);
    pinMode(LOAD, OUTPUT);
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(ECHO_PIN,INPUT);
    digitalWrite(CLK, LOW);
    digitalWrite(LOAD, HIGH);
    Serial.begin(9600);
    matrix_init();
    randomSeed(analogRead(2));
  
 // allForward();
   int leftSideSpeed = DESIRED_POWER_TO_MOTORS;
   int rightSideSpeed = (int)((float)leftSideSpeed * RIGHT_SIDE_SLOWNESS_COMPENSATION);
   
   leftSideSetSpeed(leftSideSpeed);
   rightSideSetSpeed(rightSideSpeed);
   
   //Pin setup for working with sensor
    show_smile_face();

    
  //  allForward();
  //  jacobot_current_state = STATE_WANDERING;

    jacobot_current_state = STATE_IDLE;


}






void loop() {
  
  String message = "";

  
  while(bluetooth.available()){
    message+=char(bluetooth.read());
  }

  message.trim();

  if (message == "*avanti#") 
  {
      last_behaviour =  LISTEN;
      allForward();
      delay(2000);
      allStop();
  } 
  else if (message == "*indietro#") 
  {
      last_behaviour =  LISTEN;
     allBackward();  
     delay(2000);
     allStop();
  } 
  else if ( message == "*destra#" ) 
  {
      last_behaviour =  LISTEN;
      rotateRight(90);
  }
  else if ( message == "*sinistra#" ) 
  {
      last_behaviour =  LISTEN;
      rotateLeft(90);
  }
  else if ( message == "*stop#" ) 
  {
      last_behaviour =  LISTEN;
      allStop();
  }
  else if ( message == "*rumba#" ) 
  {
      allForward();
      jacobot_current_state = STATE_WANDERING;
      last_behaviour =  ROOMBA;
      roomba_style();
  } 
  else if ( message == "*segui#" ) 
  {
      jacobot_current_state = STATE_IDLE;
      last_behaviour =  FOLLOWER;
      follow_me_but_keep_distance(60);
  } 
  else 
  {
    if (last_behaviour == ROOMBA)
      roomba_style();
    else if (last_behaviour == FOLLOWER)
      follow_me_but_keep_distance(60);
    else if (last_behaviour== LISTEN)
      allStop();
  }

 delay(500);
 
}





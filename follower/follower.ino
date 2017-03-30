#include "LineDetector.h"
#include "AFMotor.h"


#define MAX_SPEED 250
#define MIN_SPEED 0

#define NORMAL_SPEED 90
#define MOVE_BACK_SPEED 120

/************************
          SCHEMA POSIZIONE MOTORI

              ------
                ||
            M4 (    ) M3
                |  |
    M1 (____) M2
        
*************************/

//VARIABILI CONTROLLO MOTORI
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

//Rilevatore di linee
LineDetector sensorbar;


//VARIABILI DI CONTROLLO

float error = 0.0f; //errore attuale nella posizione
float lastError = 0.0f; //errore ottenuto al giro precedente 
float kp = 0.1f;
float kd = 1.0f;    //termine proporzionalef;    //termine proporzionale alla variazione dell'errore
float PV = 0.0f;    //termine usato per calcolare quanta spinta dare ai motori in base all'errore

int leftSide_motor_speed = 0;  //valore usato per impostare la velocità sul lato sinistro
int rightSide_motor_speed = 0; //valore usato per impostare la velocità sul lato destro

//buffer last position
unsigned int lastPosition = 0;


void setup() {
    Serial.begin(9600);
  
}

void loop() {

    //LEGGI POSIZIONE LINEA
    unsigned int line_position = sensorbar.readLinePosition();

    //SEGUI LINEA
    followLine(line_position, lastPosition);
    
    lastPosition = line_position;

    delay(80);
}


void stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


void turnLeft() {

 //Setta velocita default per rotazione
 motor1.setSpeed(200);
 motor4.setSpeed(200);
 motor2.setSpeed(200);
 motor3.setSpeed(200);

  //per girare a SX motori 2 e 3 avanti e 1 e 4 indietro
  motor1.run(BACKWARD);
  motor4.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);

}


void moveBack() {
 //Setta velocita default per rotazione
 motor1.setSpeed(MOVE_BACK_SPEED);
 motor4.setSpeed(MOVE_BACK_SPEED);
 motor2.setSpeed(MOVE_BACK_SPEED);
 motor3.setSpeed(MOVE_BACK_SPEED);

 //per girare a SX motori 2 e 3 indietro e 1 e 4 avanti
 motor1.run(BACKWARD);
 motor4.run(BACKWARD);
 motor2.run(BACKWARD);
 motor3.run(BACKWARD);

}

void turnRight() {
 //Setta velocita default per rotazione
 motor1.setSpeed(200);
 motor4.setSpeed(200);
 motor2.setSpeed(200);
 motor3.setSpeed(200);

 //per girare a SX motori 2 e 3 indietro e 1 e 4 avanti
 motor1.run(FORWARD);
 motor4.run(FORWARD);
 motor2.run(BACKWARD);
 motor3.run(BACKWARD);

}

void setLeftSideSpeed(unsigned int speed) {

   motor1.setSpeed(speed);
   motor4.setSpeed(speed);
  motor1.run(FORWARD);
   motor4.run(FORWARD);
}

void setRightSideSpeed(unsigned int speed) {

   motor2.setSpeed(speed);
   motor3.setSpeed(speed);
  motor2.run(FORWARD);
   motor3.run(FORWARD);
}

void findLineMovingRight() 
{
  

  unsigned int line =0;
  do {
     line = sensorbar.readLinePosition();
     Serial.print("Valore linea in do while: ");
     Serial.println(line);
     turnRight();
  } while (line > 7000);
  Serial.println("Fuori Do While");

}


void findLineMovingLeft() 
{
  

  unsigned int line =0;
  do {
     line = sensorbar.readLinePosition();
     turnLeft();
  } while (line < 2000);

}

void findLineMovingBack() 
{
  

  unsigned int line =0;
  do {
     line = sensorbar.readLinePosition();
     moveBack();
  } while (line < 1000 || line > 8000);
}


void followLine(unsigned int position, unsigned int lastPosition) {

  Serial.print("POSITION ");
  Serial.println(position);
   switch(position) {

      //LINEA SULL'ESTREMO SX DELLA BARRA DI SENSORI => IL ROBOT DEVE GIRARE A SX DI BRUTTO
      case 0:

        //SONO NEL BIANCO MA PRIMA AVEVO LINEA A SX: VADO A SX
        if (lastPosition > 0 && lastPosition < 4500)
           findLineMovingLeft();
        else if (lastPosition > 4500 )
           findLineMovingRight();
        else 
          stop();
      break;

     
      //CASO NORMALE: CONTROLLO PD PER MANTENERE IL MIO OBIETTIVO (LINEA AL CENTRO DELLA BARRA DI SENSORI)
      default:
        Serial.println("PERCORSO NORMALE");
        //Calcolo quanto disto dalla posizione obiettivo (3500)
        error = (int) position - 4500;

        
        
        Serial.print("ERRORE ");
        Serial.println(error);

        //Formula per il controllo PD: calcolo un valore di correzione per i motori che dipende da quanto
        //ora (error) e anche da quanto ho sbagliato prima (error - lastError) 
        PV = kp * error + kd * (error - lastError);


        //metto da parte l'errore che ho avuto ora per il prossimo giro
        lastError = error;

        //il valore da dare ai motori va da 0 a 255: devo tarare le costanti sopra e i valori qui sotto
        //in modo da poterlo sommare direttamente. Assumo che PV vari tra -55 e +55: siccome non devo superare
        // 255, lo sommo a 200, cosi il valore che passo ai motori sta tra 145 e 255 
        
        //Limito PV verso alto
        Serial.print("PV: ");
        Serial.println(PV);

        leftSide_motor_speed = NORMAL_SPEED + PV;
        rightSide_motor_speed = NORMAL_SPEED - PV;

        if (leftSide_motor_speed > MAX_SPEED)
          leftSide_motor_speed = MAX_SPEED;

        if (rightSide_motor_speed > MAX_SPEED)
          rightSide_motor_speed = MAX_SPEED;

       if (leftSide_motor_speed < MIN_SPEED)
          leftSide_motor_speed = MIN_SPEED;

        if (rightSide_motor_speed < MIN_SPEED)
          rightSide_motor_speed = MIN_SPEED;
          


        Serial.print("LEFT SIDE MOTOR SPEED ");
        Serial.println(leftSide_motor_speed);


        Serial.print("RIGHT SIDE MOTOR SPEED ");
        Serial.println(rightSide_motor_speed);

        setLeftSideSpeed(leftSide_motor_speed);
        setRightSideSpeed(rightSide_motor_speed);

      break;

   }

}

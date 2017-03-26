#include "LineDetector.h"
#include "AFMotor.h"

#define REFERENCE_SPEED 100 
#define MAX_SPEED_RANGE 90

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
float kp = 2.0f;    //termine proporzionale all'errore
float kd = 0.0f;    //termine proporzionale alla variazione dell'errore
float PV = 0.0f;    //termine usato per calcolare quanta spinta dare ai motori in base all'errore

unsigned int leftSide_motor_speed = 0;  //valore usato per impostare la velocità sul lato sinistro
unsigned int rightSide_motor_speed = 0; //valore usato per impostare la velocità sul lato destro



void setup() {
    Serial.begin(9600);
  
}

void loop() {

    sensorbar.printRawSensors();
    //LEGGI POSIZIONE LINEA
    unsigned int line_position = sensorbar.readLinePosition();

    //SEGUI LINEA
    followLine(line_position);
    
    //ATTENDI 
    //delay(5);

}


void stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


void turnLeft() {

 //Setta velocita default per rotazione
 motor1.setSpeed(150);
 motor4.setSpeed(150);
 motor2.setSpeed(150);
 motor3.setSpeed(150);

  //per girare a SX motori 2 e 3 avanti e 1 e 4 indietro
  motor1.run(BACKWARD);
  motor4.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);

}

void turnRight() {
 //Setta velocita default per rotazione
 motor1.setSpeed(150);
 motor4.setSpeed(150);
 motor2.setSpeed(150);
 motor3.setSpeed(150);

 //per girare a SX motori 2 e 3 indietro e 1 e 4 avanti
 motor1.run(FORWARD);
 motor4.run(FORWARD);
 motor2.run(BACKWARD);
 motor3.run(BACKWARD);

}

void setLeftSideSpeed(unsigned int speed) {
   motor1.run(FORWARD);
   motor4.run(FORWARD);
   motor1.setSpeed(speed);
   motor4.setSpeed(speed);
}

void setRightSideSpeed(unsigned int speed) {
   motor2.run(FORWARD);
   motor3.run(FORWARD);
   motor2.setSpeed(speed);
   motor3.setSpeed(speed);
}

void followLine(unsigned int position) {

  Serial.print("POSITION ");
  Serial.println(position);
   switch(position) {

      //LINEA SULL'ESTREMO SX DELLA BARRA DI SENSORI => IL ROBOT DEVE GIRARE A SX DI BRUTTO
      case 0:
        Serial.println("LINEA TUTTO A SINISTRA: GIRO A SINISTRA DI BRUTTO!");
        turnLeft();
      break;

      //LINEA ALL'ESTREMO DESTRO DELLA BARRA DI SENSORI => IL ROBOT DEVE GIRARE A DX DI BRUTTO
      case 7000:

        Serial.println("LINEA TUTTO A DESTRA: GIRO A DESTRA DI BRUTTO!");
        turnRight();
      break;
      //CASO NORMALE: CONTROLLO PD PER MANTENERE IL MIO OBIETTIVO (LINEA AL CENTRO DELLA BARRA DI SENSORI)
      default:
        Serial.println("PERCORSO NORMALE");
        //Calcolo quanto disto dalla posizione obiettivo (3500)
        error = (int) position - 3500;

        
        
        Serial.print("ERRORE ");
        Serial.println(error);

        //Formula per il controllo PD: calcolo un valore di correzione per i motori che dipende da quanto
        //ora (error) e anche da quanto ho sbagliato prima (error - lastError) 
        PV = kp * error + kd * (error - lastError);

        PV = map(PV, -2500, 2500, -MAX_SPEED_RANGE, MAX_SPEED_RANGE);
  
        Serial.print("PV ");
        Serial.println(PV);


        //metto da parte l'errore che ho avuto ora per il prossimo giro
        lastError = error;

        //il valore da dare ai motori va da 0 a 255: devo tarare le costanti sopra e i valori qui sotto
        //in modo da poterlo sommare direttamente. Assumo che PV vari tra -55 e +55: siccome non devo superare
        // 255, lo sommo a 200, cosi il valore che passo ai motori sta tra 145 e 255 
        
        //Limito PV verso alto
        if (PV > MAX_SPEED_RANGE)
        {
          PV = MAX_SPEED_RANGE;
        }

        //Limito PV verso basso
        if (PV < -MAX_SPEED_RANGE)
        {
          PV = -MAX_SPEED_RANGE;
        }

        leftSide_motor_speed = REFERENCE_SPEED + PV;
        rightSide_motor_speed = REFERENCE_SPEED - PV;

        Serial.print("LEFT SIDE MOTOR SPEED ");
        Serial.println(leftSide_motor_speed);


        Serial.print("RIGHT SIDE MOTOR SPEED ");
        Serial.println(rightSide_motor_speed);

        setLeftSideSpeed(leftSide_motor_speed);
        setRightSideSpeed(rightSide_motor_speed);

      break;

   }

}

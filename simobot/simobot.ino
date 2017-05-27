#include <Wire.h>

#define IR_L 7
#define IR_C 6
#define IR_R 5
#define STERZO 3
#define MOT_L_BA 10
#define MOT_L_FO 9
#define MOT_R_BA 11
#define MOT_R_FO 12

#define NO_DIRECTION 0
#define LEFT_DIRECTION 1
#define RIGHT_DIRECTION 2
#define CENTER_DIRECTION 3

#define DEBUG 0

int IR_L_Value = 0;
int IR_C_Value = 0;
int IR_R_Value = 0;
int STERZO_Value;

int last_direction = NO_DIRECTION;

void setup() {
  Serial.begin(9600); 

  pinMode(IR_L, INPUT);
  pinMode(IR_C, INPUT);
  pinMode(IR_R, INPUT); 
  pinMode(STERZO, OUTPUT); 
  pinMode(MOT_L_BA, OUTPUT);
  pinMode(MOT_L_FO, OUTPUT);
  pinMode(MOT_R_BA, OUTPUT);
  pinMode(MOT_R_FO, OUTPUT); 

  digitalWrite(MOT_L_BA, LOW);
  digitalWrite(MOT_L_FO, LOW);
  digitalWrite(MOT_R_BA, LOW);
  digitalWrite(MOT_R_FO, LOW);
}

void loop() {
 IR_L_Value = digitalRead(IR_L); 
 IR_C_Value = digitalRead(IR_C); 
 IR_R_Value = digitalRead(IR_R); 

#if DEBUG == 1
Serial.print("L=");
Serial.print(IR_L_Value);
Serial.print("  C=");
Serial.print(IR_C_Value); 
Serial.print("  R=");
Serial.print(IR_R_Value);  */
#endif

if(IR_L_Value==1 && IR_C_Value==0 && IR_R_Value==1){// vai dritto
STERZO_Value=165;
#if DEBUG == 1
Serial.print("  DRITTO=");
Serial.print(STERZO_Value);
#endif
analogWrite(STERZO, STERZO_Value);
  digitalWrite(MOT_L_BA, LOW);
  digitalWrite(MOT_L_FO, HIGH);
  digitalWrite(MOT_R_BA, LOW);
  digitalWrite(MOT_R_FO, HIGH);
  last_direction = CENTER_DIRECTION;
  
}

else if(IR_L_Value==0 && IR_C_Value==1 && IR_R_Value==1){// gira a sinistra
STERZO_Value=130;
#if DEBUG == 1
Serial.print("  SINISTRA=");
Serial.print(STERZO_Value); 
#endif
analogWrite(STERZO, STERZO_Value);
  digitalWrite(MOT_L_BA, LOW);
  digitalWrite(MOT_L_FO, HIGH);
  digitalWrite(MOT_R_BA, LOW);
  digitalWrite(MOT_R_FO, HIGH);
  last_direction = LEFT_DIRECTION;
}

else if(IR_L_Value==0 && IR_C_Value==0 && IR_R_Value==1){// gira stretto a sinistra
STERZO_Value=0;
#if DEBUG == 1
Serial.print("  SINISTRA=");
Serial.print(STERZO_Value); 
#endif
analogWrite(STERZO, STERZO_Value);
  digitalWrite(MOT_L_BA, HIGH);
  digitalWrite(MOT_L_FO, LOW);
  digitalWrite(MOT_R_BA, LOW);
  digitalWrite(MOT_R_FO, HIGH);
  last_direction = LEFT_DIRECTION;
}

else if(IR_L_Value==1 && IR_C_Value==1 && IR_R_Value==0){// gira a destra
STERZO_Value=200;
#if DEBUG == 1
Serial.print("  DESTRA=");
Serial.print(STERZO_Value); 
#endif
analogWrite(STERZO, STERZO_Value);
  digitalWrite(MOT_L_BA, LOW);
  digitalWrite(MOT_L_FO, HIGH);
  digitalWrite(MOT_R_BA, LOW);
  digitalWrite(MOT_R_FO, HIGH);
  last_direction = RIGHT_DIRECTION;
}

else if(IR_L_Value==1 && IR_C_Value==0 && IR_R_Value==0){// gira stretto a destra
STERZO_Value=200;
#if DEBUG == 1
Serial.print("  DESTRA=");
Serial.print(STERZO_Value); 
#endif
analogWrite(STERZO, STERZO_Value);
  digitalWrite(MOT_L_BA, LOW);
  digitalWrite(MOT_L_FO, HIGH);
  digitalWrite(MOT_R_BA, HIGH);
  digitalWrite(MOT_R_FO, LOW);
  last_direction = RIGHT_DIRECTION;
}

else if(IR_L_Value==1 && IR_C_Value==1 && IR_R_Value==1){// ferma e torna indietro
#if DEBUG == 1
Serial.print("  FERMA=");
Serial.print(STERZO_Value); 
#endif

  if (last_direction == CENTER_DIRECTION)
  {
    Serial.println("BACK CENTER");
    STERZO_Value=165;
  }
  else if (last_direction == LEFT_DIRECTION)
  {
    Serial.println("BACK LEFT");
    STERZO_Value=200;
  }
  else if (last_direction == RIGHT_DIRECTION) 
  {
      Serial.println("BACK RIGHT");
      STERZO_Value=130;
  } 
  else 
  {
      Serial.println("BACK CENTER");
      STERZO_Value=165; 
  }
    
    analogWrite(STERZO, STERZO_Value);  
    digitalWrite(MOT_L_BA, HIGH);
    digitalWrite(MOT_L_FO, LOW);
    digitalWrite(MOT_R_BA, HIGH);
    digitalWrite(MOT_R_FO, LOW);
}

else if(IR_L_Value==0 && IR_C_Value==0 && IR_R_Value==0){// appoggia la macchinina
STERZO_Value=165;
#if DEBUG == 1
Serial.print("  APPOGGIA=");
Serial.print(STERZO_Value); 
#endif
analogWrite(STERZO, STERZO_Value);
  digitalWrite(MOT_L_BA, LOW);
  digitalWrite(MOT_L_FO, LOW);
  digitalWrite(MOT_R_BA, LOW);
  digitalWrite(MOT_R_FO, LOW);
  
}
#if DEBUG == 1
Serial.println("  ---");
#endif
delay(100);
}

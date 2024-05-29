#include <Servo.h>  //include servo.h library
#include "SoftwareSerial.h"
Servo myservo;
SoftwareSerial HC06(8, 9);
uint8_t pos;
int motor_speed = 70;
boolean fire = false;
char temp;


#define Left 2     // left sensor
#define Right 4   // right sensor
#define Forward 3  //front sensor
#define LM1 13      // left motor
#define LM2 12      // left motor
#define RM1 11      // right motor
#define RM2 10     // right motor
#define pump 6

void setup() {
  Serial.begin(9600);
  HC06.begin(9600);
  pinMode(Left, INPUT);
  pinMode(Right, INPUT);
  pinMode(Forward, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(pump, OUTPUT);
  myservo.attach(5);
}
void dung(){
  digitalWrite(LM2, LOW);
  digitalWrite(LM1, LOW);
  digitalWrite(RM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(pump, LOW);
  myservo.write(90);
}

void put_off_fire() {
  delay(100);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
  if(digitalRead(Forward) == 0){
    digitalWrite(pump, HIGH);
    delay(500);

    for (pos = 50; pos <= 130; pos += 1) {
      myservo.write(pos);
      delay(10);
    }
    for (pos = 130; pos >= 50; pos -= 1) {
      myservo.write(pos);
      delay(10);
    }
  } else if (digitalRead(Forward) == 0){
    digitalWrite(pump, LOW);
    myservo.write(90);
  } 
}


void loop() {
  Serial.println(temp);

  if (HC06.available() > 0){
    temp = HC06.read();
    switch(temp)
    {
    case 'F':
      digitalWrite(LM2, HIGH);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, HIGH);
      digitalWrite(RM1, LOW);
      myservo.write(90);
      digitalWrite(pump, LOW);
      break;
    case 'B':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, HIGH);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, HIGH);
      myservo.write(90);
      digitalWrite(pump, LOW);
      break;
    case 'G':
      digitalWrite(LM2, HIGH);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, LOW);
      myservo.write(90);
      digitalWrite(pump, LOW);
      break;
    case 'I':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, HIGH);
      digitalWrite(RM1, LOW);
      myservo.write(90);
      digitalWrite(pump, LOW);
      break;
    case 'H':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, HIGH);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, LOW);
      myservo.write(90);
      digitalWrite(pump, LOW);
      break;
    case 'J':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, HIGH);
      myservo.write(90);
      digitalWrite(pump, LOW);
      break;
    case 'L':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, LOW);
      digitalWrite(pump, HIGH);
      for(pos = 90; pos < 130; pos++)
      {
        myservo.write(pos);
      }delay(15);
      break;
    case 'R':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, LOW);
      digitalWrite(pump, HIGH);
      for(pos = 90; pos > 50; pos--)
      {
        myservo.write(pos);
      }delay(15);
      break;
    case 'S':
      digitalWrite(LM2, LOW);
      digitalWrite(LM1, LOW);
      digitalWrite(RM2, LOW);
      digitalWrite(RM1, LOW); 
      myservo.write(90);
      digitalWrite(pump, LOW);
      if (digitalRead(Forward) == 0 && digitalRead(Right) != 0 && digitalRead(Left) != 0) 
      {
        digitalWrite(LM2, HIGH);
        digitalWrite(LM1, LOW);
        digitalWrite(RM2, HIGH);
        digitalWrite(RM1, LOW);
        delay(10);
        put_off_fire();
      } else if (digitalRead(Left) == 0 && digitalRead(Right) == 1) {
        digitalWrite(LM2, HIGH);
        digitalWrite(LM1, LOW);
        digitalWrite(RM1, LOW);
        digitalWrite(RM2, LOW);
        digitalWrite(pump, LOW);
        delay(100);
        dung();
      } else if (digitalRead(Right) == 0 && digitalRead(Left) == 1) {
        digitalWrite(LM1, LOW);
        digitalWrite(LM2, LOW);
        digitalWrite(RM2, HIGH);
        digitalWrite(RM1, LOW);
        digitalWrite(pump, LOW);
        delay(100);
        dung();
      }
      break;
    }
    delay(10);

  } else {
    
    if (digitalRead(Forward) == 0 && digitalRead(Right) != 0 && digitalRead(Left) != 0) 
      {
        digitalWrite(LM2, HIGH);
        digitalWrite(LM1, LOW);
        digitalWrite(RM2, HIGH);
        digitalWrite(RM1, LOW);
        digitalWrite(pump, LOW);
        delay(10);
        put_off_fire();
      } else if (digitalRead(Left) == 0 && digitalRead(Right) == 1) {
        digitalWrite(LM2, HIGH);
        digitalWrite(LM1, LOW);
        digitalWrite(RM1, LOW);
        digitalWrite(RM2, LOW);
        digitalWrite(pump, LOW);
        delay(100);
        dung();
      } else if (digitalRead(Right) == 0 && digitalRead(Left) == 1) {
        digitalWrite(LM1, LOW);
        digitalWrite(LM2, LOW);
        digitalWrite(RM2, HIGH);
        digitalWrite(RM1, LOW);
        digitalWrite(pump, LOW);
        delay(100);
        dung();
      }
  }
   digitalWrite(pump, LOW);
}
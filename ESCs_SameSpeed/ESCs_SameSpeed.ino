#include <Servo.h>


int value = 1000; // set values you need to zero

Servo firstESC, secondESC, thirdESC, fourthESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time


void setup() {

  firstESC.attach(13);   // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(12);
  thirdESC.attach(11);
  fourthESC.attach(10);
  Serial.begin(9600);    // start serial at 9600 baud
  delay(3000);

}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  firstESC.writeMicroseconds(value);
  secondESC.writeMicroseconds(value);
  thirdESC.writeMicroseconds(value);
  fourthESC.writeMicroseconds(value);
  
  if(Serial.available()) 
    value = Serial.parseInt();    // Parse an Integer from Serial

}

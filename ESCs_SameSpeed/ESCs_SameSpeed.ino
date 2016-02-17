#include <Servo.h>


int value = 1000; // set values you need to zero

Servo firstESC, secondESC, thirdESC, fourthESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time


void setup() {

  firstESC.attach(13);   // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(12);
  thirdESC.attach(11);
  fourthESC.attach(10);
  firstESC.writeMicroseconds(1000);
  secondESC.writeMicroseconds(1000);
  thirdESC.writeMicroseconds(1000);
  fourthESC.writeMicroseconds(1000);
  Serial.begin(9600);    // start serial at 9600 baud
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  Serial.println("Ready to parse:");

}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions

  //int value = 1200;
  
  //firstESC.writeMicroseconds(value);
  secondESC.writeMicroseconds(value);
  //thirdESC.writeMicroseconds(value);
  fourthESC.writeMicroseconds(value);
  
  if(Serial.available()) {
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);
  }
  
}

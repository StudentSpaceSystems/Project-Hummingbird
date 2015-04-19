#include <SoftwareSerial.h>
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
SoftwareSerial XBee(2, 3);
int led = 13;

int n = 0;
int m = 0;

String s;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  XBee.begin(9600);
  Serial.begin(9600);  
}

bool flag = false;
// the loop routine runs over and over again forever:
void loop() {
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    XBee.write(Serial.read());
  }
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.write(XBee.read());
    n = XBee.read().toInt();
  }
  
  if (n != 0) 
  {
    while (n <= m)
    { 
       digitalWrite(13,HIGH);
       delay(500);
       digitalWrite(13,LOW);
       delay(500);
       m += 1;
       
       Serial.print(n);Serial.print("  ");Serial.print(m);Serial.println();
    }
    if (n == m)
    {
       n = 0;
       m = 0;
    }
  }
  
}

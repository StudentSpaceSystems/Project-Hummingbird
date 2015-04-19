#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial XBee(2, 3); // RX, TX

const byte commandSize = 2;
char commandBuffer[commandSize * 4 - 1];
const char delimiter = ',';
const char start = '<';
const char finish = '>';
boolean writing = false;
int x = 0;

int commands[] = {0, 0, 0, 0};

Servo esc1, esc2, esc3, esc4;

void setup()
{
  XBee.begin(9600);
  Serial.begin(9600);
  Serial.println("Attemtping to read from Xbee...");

  esc1.attach(9);
  esc2.attach(10);
  esc3.attach(11);
  esc4.attach(12);
}

void loop()
{
  if (Serial.available())
  {
    //XBee.write(s);
  }
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    char c = XBee.read();
    if (c == finish)
    {
      writing = false;
      x = 0;
      parseBuffer();
      executeCommands();
      clearCommands();
    }
    if (writing)
    {
      if (c != delimiter)
      {
        commandBuffer[x] = c;
        x += 1;
      }
    }
    if (c == start)
    {
      writing = true;
    }
  }
  char s = Serial.read();
    if (s == 'k')
    {
      Serial.println('Killing!');
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);
    }
}


void executeCommands()
{
  Serial.println();
  for (int i = 0; i < 4; i += 1)
  {
    commands[i] = ((commands[i] / (pow(10, commandSize))) * (1000.0)) + 1000.0;
    Serial.println(commands[i]);
  }

  esc1.writeMicroseconds(commands[0]);
  esc2.writeMicroseconds(commands[1]);
  esc3.writeMicroseconds(commands[2]);
  esc4.writeMicroseconds(commands[3]);
}

void clearCommands()
{
  for (int i = 0; i < sizeof(commands); i += 1)
  {
    commands[i] = 0;
  }
}

void parseBuffer() {
  for (int motor = 0; motor < 4; motor += 1)
  {
    for (int p = 0; p < commandSize; p += 1)
    {
      int index = (motor * commandSize) + p;
      int k = pow(10, 1 - p);
      int a = commandBuffer[index] - 48;
      commands[motor] += (a * k);
    }
  }
}


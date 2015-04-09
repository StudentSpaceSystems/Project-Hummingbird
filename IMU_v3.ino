#include "Wire.h"
#include "I2Cdev.h"
#include "Quaternion.h"

const float pi = 3.14159265358979323846264338327950;

//IMU Init
#include "MPU6050.h"
MPU6050 accelgyro;
float Ax, Ay, Az;
float Gx, Gy, Gz;
float Mx, My, Mz;
#define LED_PIN 13
bool blinkState = false;
float estGx,estGy,estGz;


//Baro Init
#include "SFE_BMP180.h"
SFE_BMP180 pressure;
double baseline;
double relativeAlt;


//Quaternion Init
// (v, theta) or (a, b, c, d)
float Ox, Oy, Oz;
Quaternion qk;
float t0 = 0;


void setup() {
  Wire.begin();
  Serial.begin(38400);


  //IMU Setup
  accelgyro.initialize();
  Serial.println("Testing IMU connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(LED_PIN, OUTPUT);
  estGx = 0;
  estGy = 0;
  estGz = 0;
  
  
  float v0[3] = {0,0,1};
  qk = Quaternion(v0,0);

  //Baro Setup
  pressure.begin();
  baseline = getPressure();
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");
  
}

void loop() {
  
  getIMU();
  getAlt();
  float t = millis()/1000;
  float dt = t - t0;
  t0 = t;


  float l = sqrt(pow(Gx,2)+pow(Gy,2)+pow(Gz,2));
  float theta = l*dt;
  float v[3] = {Gx/l, Gy/l, Gz/l};
  Quaternion q (v, theta);
  Quaternion qf = qk.multiplyBy(q);
  qk = qf;
  
  Serial.print(qf.getTheta());Serial.print("\t");
  Serial.print(qf.geti());Serial.print("\t");
  Serial.print(qf.getj());Serial.print("\t");
  Serial.print(qf.getk());Serial.print("\t");
  Serial.println();
  
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}


void getIMU()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Ax = ax * 2.0f / 32768.0f;
  Ay = ay * 2.0f / 32768.0f;
  Az = az * 2.0f / 32768.0f;

  //Radians/second
  Gx = (gx * 250.0f / 32768.0f)*(pi/180);
  Gy = (gy * 250.0f / 32768.0f)*(pi/180);
  Gz = (gz * 250.0f / 32768.0f)*(pi/180);
  
  Gx = (0.5*Gx)+(0.5*estGx);
  Gy = (0.5*Gy)+(0.5*estGy);
  Gz = (0.5*Gz)+(0.5*estGz);
  estGx = Gx;
  estGy = Gy;
  estGz = Gz;

  Mx = mx * 10.0f * 1229.0f / 4096.0f + 18.0f;
  My = my * 10.0f * 1229.0f / 4096.0f + 18.0f;
  Mz = mz * 10.0f * 1229.0f / 4096.0f + 18.0f;
}


void getAlt()
{
  double P = getPressure();
  relativeAlt = pressure.altitude(P,baseline);
}


double getPressure()
{
  char status;
  double T, P, p0, a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}


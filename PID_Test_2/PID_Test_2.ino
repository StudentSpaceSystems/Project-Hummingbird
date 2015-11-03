#include "Wire.h"
#include "I2Cdev.h"
#include "Servo.h"
#include <PID_v1.h>

const float pi = 3.14159265358979323846264338327950;

//IMU Init
#include "MPU6050.h"
MPU6050 accelgyro;
float Ax,Ay,Az;
float Gx,Gy,Gz;
float Mx,My,Mz;
#define LED_PIN 13
bool blinkState = false;

//Servo Init
Servo esc1,esc2,esc3,esc4;

//PID Setup
double Setpoint, Input, Output;
PID Xaxis(&Input, &Output, &Setpoint,6,3,2, DIRECT);

//Kalman setup
struct kalman_state {
  double q; //process noise covariance
  double r; //measurement noise covariance
  double value; //value
  double p; //estimation error covariance
  double k; //kalman gain
};

struct kalman_state AxState;
struct kalman_state AyState;
struct kalman_state AzState;

struct kalman_state GxState;
struct kalman_state GyState;
struct kalman_state GzState;

struct kalman_state MxState;
struct kalman_state MyState;
struct kalman_state MzState;

struct kalman_state altState;

double axq, axr, axp, gxq, gxr, gxp, mxq, mxr, mxp;
double ayq, ayr, ayp, gyq, gyr, gyp, myq, myr, myp;
double azq, azr, azp, gzq, gzr, gzp, mzq, mzr, mzp;
float axoff, ayoff, azoff, gxoff, gyoff, gzoff, mxoff, myoff, mzoff, altoff;

struct kalman_state kalman_init(double q, double r, double p, double intial_value)
{
  struct kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.value = intial_value;

  return result;
}

void kalman_update(struct kalman_state* state, double measurement)
{
  //prediction update
  //omit x = x
  state->p = state->p + state->q;

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->value = state->value + state->k * (measurement - state->value);
  state->p = (1 - state->k) * state->p;
}


void setup() {
  //ESC Setup
  esc1.attach(13);
  esc2.attach(12);
  esc3.attach(11);
  esc4.attach(10);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  
  Wire.begin();
  Serial.begin(38400);

  //PID Begin
  Xaxis.SetMode(AUTOMATIC);
  Input = Gx;
  Setpoint = 0;
  Xaxis.SetOutputLimits(-1,1);

  //IMU Setup
  accelgyro.initialize();
  Serial.println("Testing IMU connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(LED_PIN, OUTPUT);
  
  // initialize the state values for the various kalman filters
  axq=ayq=azq=1;
  axr=ayr=azr=1;
  axp=ayp=azp=1;
  
  gxq=gyq=gzq=0.1;
  gxr=gyr=gzr=4;
  gxp=gyp=gzp=1;
  
  mxq=myq=mzq=1;
  mxr=myr=mzr=1;
  mxp=myp=mzp=1;
 
  // initialize the offsets for kalman states
  //gxoff = 0.264666051;
  //gyoff = 0.883565366;
  //gzoff = -0.820515335;
  
  gxoff = 0;
  gyoff = 0;
  gzoff = 0;
  
  // initialize the kalman states
  Serial.println("Creating kalman states");
  AxState = kalman_init(axq, axr, axp, 0);
  AyState = kalman_init(ayq, ayr, ayp, 0);
  AzState = kalman_init(azq, azr, azp, 0);
  
  GxState = kalman_init(axq, axr, axp, 0);
  GyState = kalman_init(ayq, ayr, ayp, 0);
  GzState = kalman_init(azq, azr, azp, 0);
  
  MxState = kalman_init(axq, axr, axp, 0);
  MyState = kalman_init(ayq, ayr, ayp, 0);
  MzState = kalman_init(azq, azr, azp, 0);
  
  Serial.print("Finding offset -- do not touch\n");
  findOffset(&gxoff, &gyoff, &gzoff);
  Serial.print("Offsets found:\t"+String(gxoff)+"\t"+String(gyoff)+"\t"+String(gzoff)+"\n");
  digitalWrite(LED_PIN, LOW);
  delay(5000);
}

float t0 = 0;
float dtheta = 0;
int speed2 = 1200;
int speed4 = 1200;

void loop() {
  
  getIMU();
  float t = millis();
  float dt = t-t0;

  Input = Gx;

  if (dt > 20) {
    Xaxis.Compute();
    int sf = 5;
    speed2 -= Output * sf;
    speed4 += Output * sf;

    if (speed2 > 1500)
    {
      speed2 = 1500;
    }
    if (speed2 < 1200)
    {
      speed2 = 1200;
    }
    if (speed4 > 1500)
    {
      speed4 = 1500;
    }
    if (speed4 < 1200)
    {
      speed4 = 1200;
    }
    esc2.writeMicroseconds(speed2);
    esc4.writeMicroseconds(speed4);
    
    //Serial.println(speed2);
    //Serial.println(speed4);

    t0 = t;
  }
  
  
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void findOffset(float *gxoffptr, float *gyoffptr, float *gzoffptr)
{
  double gxavg=0, gyavg=0, gzavg=0;
  long numValues = 0;
  unsigned long start = millis();
  while (millis() - start <= 10000) {
    getIMU();
    gxavg = (gxavg*numValues - GxState.value) / (numValues + 1);
    gyavg = (gyavg*numValues - GyState.value) / (numValues + 1);
    gzavg = (gzavg*numValues - GzState.value) / (numValues + 1); 
  }
  *gxoffptr = gxavg;
  *gyoffptr = gyavg;
  *gzoffptr = gzavg;
}

void getIMU()
{
  float gcutoff = 0.01;
  
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

  Mx = mx * 10.0f * 1229.0f / 4096.0f + 18.0f;
  My = my * 10.0f * 1229.0f / 4096.0f + 18.0f;
  Mz = mz * 10.0f * 1229.0f / 4096.0f + 18.0f;
  
  // adjust gyro values w/ filter
  Gx = Gx<gcutoff&&Gx>-gcutoff?0:Gx;
  Gy = Gy<gcutoff&&Gy>-gcutoff?0:Gy;
  Gz = Gz<gcutoff&&Gz>-gcutoff?0:Gz;
  
  kalman_update(&AxState,Ax);
  kalman_update(&AyState,Ay);
  kalman_update(&AzState,Az);
  
  kalman_update(&GxState,Gx);
  kalman_update(&GyState,Gy);
  kalman_update(&GzState,Gz);
  
  kalman_update(&MxState,Mx);
  kalman_update(&MyState,My);
  kalman_update(&MzState,Mz);
}

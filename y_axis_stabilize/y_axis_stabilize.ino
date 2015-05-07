#include "Wire.h"
#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include <Servo.h>

#include <PID_v1.h>

const float pi = 3.14159265358979323846264338327950;

Servo esc1, esc2, esc3, esc4;

//IMU Init
#include "MPU6050.h"
MPU6050 accelgyro;
float Ax, Ay, Az;
float Gx, Gy, Gz;
float Mx, My, Mz;

//Baro Init
#include "SFE_BMP180.h"
SFE_BMP180 pressure;
double baseline;
double relativeAlt;

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
double altq, altr, altp;
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

double fInput, fOutput, fSetpoint;
double wInput, wOutput, wSetpoint;
PID fPID (&fInput, &fOutput, &fSetpoint, 1, 3, 1, DIRECT);
PID wPID (&wInput, &wOutput, &wSetpoint, 1, 1, 1, DIRECT);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Attemtping to read from Xbee...");
  esc1.attach(13);
  esc2.attach(12);
  esc3.attach(11);
  esc4.attach(10);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  //IMU Setup
  accelgyro.initialize();
  Serial.println("Testing IMU connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Baro Setup
  pressure.begin();
  baseline = getPressure();
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");

  // initialize the state values for the various kalman filters
  axq = ayq = azq = 1;
  axr = ayr = azr = 1;
  axp = ayp = azp = 1;

  gxq = gyq = gzq = 0.1;
  gxr = gyr = gzr = 4;
  gxp = gyp = gzp = 1;

  mxq = myq = mzq = 1;
  mxr = myr = mzr = 1;
  mxp = myp = mzp = 1;

  altq = 1;
  altr = 1;
  altp = 1;

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

  altState = kalman_init(altq, altr, altp, 0);

  Serial.print("Finding offset -- do not touch\n");
  findOffset(&gxoff, &gyoff, &gzoff);
  Serial.print("Offsets found:\t" + String(gxoff) + "\t" + String(gyoff) + "\t" + String(gzoff) + "\n");
  delay(5000);

  wSetpoint = 0.00;

  //turn the PID on
  fPID.SetMode(AUTOMATIC);
  fPID.SetSampleTime(50);
  fPID.SetOutputLimits(-100, 100);
  wPID.SetMode(AUTOMATIC);
  wPID.SetSampleTime(50);
  wPID.SetOutputLimits(-100, 100);
}

long tL = 0;
long t0 = 0;
float xRot = 0.0;

void loop() {
  getIMU();
  getAlt();
  digitalWrite(13, HIGH);

  long t = millis();
  long dt = t - t0;
  t0 = t;
  
  xRot += (GxState.value + gxoff) * (dt/1000.0);
  
  if (xRot > 0.01)
  {
    fSetpoint = -1;
  }
  else if (xRot < -0.01)
  {
    fSetpoint = 1; 
  }
  else
  {
    fSetpoint = 0.00;
  }

  fInput = GxState.value + gxoff;
  fPID.Compute();
  Serial.println(wOutput);

  
  if (t - tL > 20)
  {
    tL = t;
    int base = 1300;
    int speed2;
    int speed4;
    if (fOutput < 0)
    {
      speed2 = base - 10 * fOutput;
      speed4 = base;
    }
    else if (fOutput > 0)
    {
      speed2 = base;
      speed4 = base + 10 * fOutput;
    }
    else
    {
      speed2 = base;
      speed4 = base;
    }
    esc2.writeMicroseconds(speed2);
    esc4.writeMicroseconds(speed4);
  }
}

void findOffset(float *gxoffptr, float *gyoffptr, float *gzoffptr)
{
  double gxavg = 0, gyavg = 0, gzavg = 0;
  long numValues = 0;
  unsigned long start = millis();
  while (millis() - start <= 10000) {
    digitalWrite(13, HIGH);
    getIMU();
    gxavg = (gxavg * numValues - GxState.value) / (numValues + 1);
    gyavg = (gyavg * numValues - GyState.value) / (numValues + 1);
    gzavg = (gzavg * numValues - GzState.value) / (numValues + 1);
  }
  digitalWrite(13, LOW);
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
  Gx = (gx * 250.0f / 32768.0f) * (pi / 180);
  Gy = (gy * 250.0f / 32768.0f) * (pi / 180);
  Gz = (gz * 250.0f / 32768.0f) * (pi / 180);

  Mx = mx * 10.0f * 1229.0f / 4096.0f + 18.0f;
  My = my * 10.0f * 1229.0f / 4096.0f + 18.0f;
  Mz = mz * 10.0f * 1229.0f / 4096.0f + 18.0f;

  // adjust gyro values w/ filter
  Gx = Gx < gcutoff && Gx > -gcutoff ? 0 : Gx;
  Gy = Gy < gcutoff && Gy > -gcutoff ? 0 : Gy;
  Gz = Gz < gcutoff && Gz > -gcutoff ? 0 : Gz;

  kalman_update(&AxState, Ax);
  kalman_update(&AyState, Ay);
  kalman_update(&AzState, Az);

  kalman_update(&GxState, Gx);
  kalman_update(&GyState, Gy);
  kalman_update(&GzState, Gz);

  kalman_update(&MxState, Mx);
  kalman_update(&MyState, My);
  kalman_update(&MzState, Mz);
}


void getAlt()
{
  double P = getPressure();
  relativeAlt = pressure.altitude(P, baseline);
  kalman_update(&altState, relativeAlt);
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


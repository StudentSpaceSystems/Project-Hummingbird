#include "Wire.h"
#include "I2Cdev.h"
#include "Quaternion.h"
#include <SoftwareSerial.h>
#include <Servo.h>

const float pi = 3.14159265358979323846264338327950;

//XBee + Motor Stuff
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


//Quaternion Init
// (v, theta) or (a, b, c, d)
float Ox, Oy, Oz;
Quaternion qk;
float t0 = 0;


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


void setup() {
  Wire.begin();
  Serial.begin(9600);

  XBee.begin(9600);
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


  float v0[3] = {0, 0, 1};
  qk = Quaternion(v0, 0);

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

  altState = kalman_init(altq, altr, altp, 0);

  Serial.print("Finding offset -- do not touch\n");
  findOffset(&gxoff, &gyoff, &gzoff);
  Serial.print("Offsets found:\t" + String(gxoff) + "\t" + String(gyoff) + "\t" + String(gzoff) + "\n");
  delay(5000);
}

float tLast = 0.0;
float yRot = 0.0;
int level1 = 20;
int level3 = 20;

void loop() {
  float t = millis() / 1000.0;
  float dt = t - t0;
  t0 = t;
  float dyRot = (GyState.value + gyoff) * dt;
  yRot += dyRot;

  int increment = 1;

  getIMU();
  getAlt();
  readXBee();

  if (t - tLast > 0.05)
  {
    tLast = t;

    if (yRot > 0.05)
    {
      if (GyState.value + gyoff > 0)
      {
        level1 += increment;
        level3 -= increment;
      }
    }
     else if (yRot < -0.05)
    {
      if (GyState.value + gyoff < 0)
      {
        level3 += increment;
        level1 -= increment;
      }
    }
    else
    {
      level1 = 20;
      level3 = 20;
    }
    
    if (level1 < 20)
    {
      level1 = 20;
    }
    if (level1 > 50)
    {
      level1 = 50;
    }
    if (level3 < 20)
    {
      level3 = 20;
    }
    if (level3 > 50)
    {
      level3 = 50;
    }
   
    esc1.writeMicroseconds(1000 + (level1*10));
    esc3.writeMicroseconds(1000 + (level3*10));
  }
 
  digitalWrite(13, HIGH);

}

void readXBee()
{
  if (XBee.available())
  {
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

void findSolution(Quaternion qf, float dt)
{
  float vTarget[] = {1, 0, 0};
  Quaternion qTarget = Quaternion(vTarget, 0);
  Quaternion z = Quaternion(0, 0, 0, 0); //blank object
  Quaternion qDerrivative = (z.differenceBetween(qf, qTarget)).multiplyScalar(1 / dt);
  Quaternion qInv = (qf.multiplyScalar(0.5)).inverse();
  Quaternion solution = qInv.multiplyBy(qDerrivative);
  Serial.print(solution.geta()); Serial.print(solution.getb()); Serial.print(solution.getc()); Serial.print(solution.getd());
  Serial.println();
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


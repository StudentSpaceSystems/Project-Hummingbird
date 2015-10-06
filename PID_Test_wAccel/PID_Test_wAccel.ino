#include "Wire.h"
#include "I2Cdev.h"
#include "PID_v1.h"
#include "Servo.h"

const float pi = 3.14159265358979323846264338327950;
float t = 0;

//Servo Init
Servo esc1,esc2,esc3,esc4;
int speed1,speed2,speed3,speed4;

//IMU Init
#include "MPU6050.h"
MPU6050 accelgyro;
float Ax,Ay,Az;
float Gx,Gy,Gz;
float Mx,My,Mz;
#define LED_PIN 13
bool blinkState = false;


//Baro Init
#include "SFE_BMP180.h"
SFE_BMP180 pressure;
double baseline;
double relativeAlt;

//PID setup
double Xsetpoint, Xinput, Xoutput;
PID Xbalance(&Xinput, &Xoutput, &Xsetpoint, 7,10,3, DIRECT);

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
  Serial.begin(38400);


  //IMU Setup
  accelgyro.initialize();
  Serial.println("Testing IMU connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(LED_PIN, OUTPUT);
  
  //Baro Setup
  pressure.begin();
  baseline = getPressure();
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");
  
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
  
  altq=1;
  altr=1;
  altp=1;
  
  // initialize the offsets for kalman states  
  gxoff = 0;
  gyoff = 0;
  gzoff = 0;
  axoff = 0;
  ayoff = 0;
  azoff = 0;
  
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

  //Motor Activation
  esc1.attach(13);
  esc2.attach(12);
  esc3.attach(11);
  esc4.attach(10);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  
  Serial.print("Finding offset -- do not touch\n");
  findOffsetg(&gxoff, &gyoff, &gzoff);
  findOffseta(&axoff, &ayoff, &azoff);
  Serial.print("Offsets found:\t"+String(gxoff)+"\t"+String(gyoff)+"\t"+String(gzoff)+"\n");

  //PID Activation
  Xbalance.SetMode(AUTOMATIC);
  Xbalance.SetOutputLimits(-0.7,0.7);

  delay(5000);
  speed2 = 1300;
  speed4 = 1300;
  esc2.writeMicroseconds(speed2);
  esc4.writeMicroseconds(speed4);
}

void loop() {
  int t1 = millis();
  
  getIMU();
  getAlt();

  Gx = ((int)(10000000*Gx))/10000000.0;
  //Serial.print("Gx:\t"); Serial.print(Gx,10); Serial.print("\t");
  //Serial.print("Gy:\t"); Serial.print(Gy); Serial.print("\t");
  //Serial.print("Gz:\t"); Serial.print(Gz); Serial.print("\n");

  Xinput = Ay; // CHANGED -- set Ax to 0 minimize horizontal acceleration
  Xsetpoint = 0;
  Xbalance.Compute();

  int step = 1000 * Xoutput;
  int idle  = 1300;

  if ((t1 - t) > 0.02*1000);
  {
    t = t1;
    if (Xoutput > 0.0)
    {
      speed2 = idle;
      speed4 = idle + step;
    }
    else if (Xoutput < 0.0)
    {
      speed2 = idle - step;
      speed4 = idle;
    }
    else
    {
      speed2 = idle;
      speed4 = idle;
    }
  }

  Serial.print("Ax:\t"); Serial.print(Ax); Serial.print("\t");
  Serial.print("Ay:\t"); Serial.print(Ay); Serial.print("\t");
  Serial.print("Az:\t"); Serial.print(Az); Serial.print("\n");
  Serial.print("XOutput:\t");Serial.println(Xoutput,10);
  Serial.print("Speed2:\t"); Serial.print(speed2); Serial.print("\t");
  Serial.print("Speed4:\t"); Serial.print(speed4); Serial.print("\n");
  Serial.println("");
  esc2.writeMicroseconds(speed2);
  esc4.writeMicroseconds(speed4);

  //Serial.print("Speed 2:");Serial.print(speed2);Serial.print("\t Speed 4:");Serial.println(speed4);

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void findOffsetg(float *gxoffptr, float *gyoffptr, float *gzoffptr)
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

void findOffseta(float *axoffptr, float *ayoffptr, float *azoffptr)
{
  double axavg=0, ayavg=0, azavg=0;
  long numValues = 0;
  unsigned long start = millis();
  while (millis() - start <= 10000) {
    getIMU();
    axavg = (axavg*numValues - AxState.value) / (numValues + 1);
    ayavg = (ayavg*numValues - AyState.value) / (numValues + 1);
    azavg = (azavg*numValues - AzState.value) / (numValues + 1); 
  }
  *axoffptr = axavg;
  *ayoffptr = ayavg;
  *azoffptr = azavg - 1.0;
}

void getIMU()
{
  float gcutoff = 0.01;
  float acutoff = 0.01;
  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
  
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  //Radians/second
  Gx = (gx * 250.0f / 32768.0f)*(pi/180);
  Gy = (gy * 250.0f / 32768.0f)*(pi/180);
  Gz = (gz * 250.0f / 32768.0f)*(pi/180);
  
  // adjust gyro values w/ filter
  Gx = Gx<gcutoff&&Gx>-gcutoff?0:Gx;
  Gy = Gy<gcutoff&&Gy>-gcutoff?0:Gy;
  Gz = Gz<gcutoff&&Gz>-gcutoff?0:Gz;

  Ax = Ax<acutoff&&Ax>-acutoff?0:Ax;
  Ay = Ay<acutoff&&Ay>-acutoff?0:Ay;
  Az = Az<acutoff&&Az>-acutoff?0:Az;

  Ax = ax * 2.0f / 32768.0f;
  Ay = ay * 2.0f / 32768.0f;
  Az = az * 2.0f / 32768.0f;

  Mx = mx * 10.0f * 1229.0f / 4096.0f + 18.0f;
  My = my * 10.0f * 1229.0f / 4096.0f + 18.0f;
  Mz = mz * 10.0f * 1229.0f / 4096.0f + 18.0f;

  kalman_update(&AxState,Ax);
  kalman_update(&AyState,Ay);
  kalman_update(&AzState,Az);

  kalman_update(&MxState,Mx);
  kalman_update(&MyState,My);
  kalman_update(&MzState,Mz);
  
  kalman_update(&GxState,Gx);
  kalman_update(&GyState,Gy);
  kalman_update(&GzState,Gz);
  
  Gx = GxState.value+gxoff;
  Gy = GyState.value+gyoff;
  Gz = GzState.value+gzoff;

  Ax = AxState.value+axoff;
  Ay = AyState.value+ayoff;
  Az = AzState.value+azoff;

}


void getAlt()
{
  double P = getPressure();
  relativeAlt = pressure.altitude(P,baseline);
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


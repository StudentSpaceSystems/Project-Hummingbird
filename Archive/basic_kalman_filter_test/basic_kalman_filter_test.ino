#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float GxOffset = 0.264666051;
float GyOffset = 0.883565366;
float GzOffset = -0.820515335;

float Gx,Gy,Gz;
float Ax,Ay,Az;

#define LED_PIN 13
bool blinkState = false;

// struct for the state of the kalman filter
struct kalman_state {
  double q; //process noise covariance
  double r; //measurement noise covariance
  double value; //value
  double p; //estimation error covariance
  double k; //kalman gain
};

struct kalman_state GxState;
struct kalman_state GyState;
struct kalman_state GzState;

struct kalman_state AxState;
struct kalman_state AyState;
struct kalman_state AzState;

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
  //omit value = value
  state->p = state->p + state->q;

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->value = state->value + state->k * (measurement - state->value);
  state->p = (1 - state->k) * state->p;
}

void setup() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    // initialize the kalman state
    Serial.println("Creating kalman state");
    GxState = kalman_init(0.1, 4, 1, 0);
    GyState = kalman_init(0.1, 4, 1, 0);
    GzState = kalman_init(0.1, 4, 1, 0);

    AxState = kalman_init(0.1, 4, 1, 0);
    AyState = kalman_init(0.1, 4, 1, 0);
    AzState = kalman_init(0.1, 4, 1, 0);

}

void loop() {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    
    Gx = (gx * 250.0f / 32768.0f);
    Gy = (gy * 250.0f / 32768.0f);
    Gz = (gz * 250.0f / 32768.0f);

    Ax = ax * 2.0f / 32768.0f;
    Ay = ay * 2.0f / 32768.0f;
    Az = az * 2.0f / 32768.0f;
    
    kalman_update(&GxState,Gx);
    kalman_update(&GyState,Gy);
    kalman_update(&GzState,Gz);
    
    kalman_update(&AxState,Ax);
    kalman_update(&AyState,Ay);
    kalman_update(&AzState,Az);

    long t = micros();

    Serial.print("Ax: ");Serial.print(AxState.value);Serial.print("\t");
    Serial.print("Ay: ");Serial.print(AyState.value);Serial.print("\t");
    Serial.print("Az: ");Serial.print(AzState.value);Serial.print("\t");
    
    Serial.println();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

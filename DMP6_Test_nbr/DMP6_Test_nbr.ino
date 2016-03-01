// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

//PID Include + Setup
#include <PID_v1.h>

double rollSetpoint, rollInput, rollOutput;
double Kp=0, Ki=0, Kd=0;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

//ESC Connect
#include <Servo.h>
int speed1,speed2,speed3,speed4;
Servo firstESC, secondESC, thirdESC, fourthESC; 

//Non-blocking reader variables
boolean NBR_readFlag = false;
int NBR_intBuffer[3] = {-1,-1,-1};
String NBR_stringBuffer = "";

//
boolean GLOBAL_PAUSE_STATE = false;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


// uncomment "rollOutput_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define rollOutput_READABLE_QUATERNION

// uncomment "rollOutput_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define rollOutput_READABLE_EULER

// uncomment "rollOutput_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define rollOutput_READABLE_YAWPITCHROLL

// uncomment "rollOutput_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us rollOutput_READABLE_WORLDACCEL instead.
//#define rollOutput_READABLE_REALACCEL

// uncomment "rollOutput_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define rollOutput_READABLE_WORLDACCEL


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Serial.begin(115200);
    
    firstESC.attach(13);
    secondESC.attach(12);
    thirdESC.attach(11);
    fourthESC.attach(10);
    firstESC.writeMicroseconds(1000);
    secondESC.writeMicroseconds(1000);
    thirdESC.writeMicroseconds(1000);
    fourthESC.writeMicroseconds(1000);
    speed1 = 1000;
    speed2 = 1000;
    speed3 = 1000;
    speed4 = 1000;
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


    //PID Activation
    rollPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-100.0,100.0);
    rollSetpoint = 0.0;
    
    // configure LED for rollOutput
    pinMode(LED_PIN, rollOutput);
}


int baseline = 1400;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    Serial.println("1");
    // non-blocking read from serial per-byte manner
    if (NBR_readFlag and Serial.available()) {
      char NBR_c;
      NBR_c = Serial.read();
      if (NBR_c == 59) {   // Semicolon
        int NBR_commaIndex = NBR_stringBuffer.indexOf(',');
        int NBR_secondCommaIndex = NBR_stringBuffer.indexOf(',', NBR_commaIndex+1);
        NBR_intBuffer[0] = NBR_stringBuffer.substring(0, NBR_commaIndex).toInt();
        NBR_intBuffer[1] = NBR_stringBuffer.substring(NBR_commaIndex+1, NBR_secondCommaIndex).toInt();
        NBR_intBuffer[2] = NBR_stringBuffer.substring(NBR_secondCommaIndex+1).toInt();
        Serial.print(":: ");Serial.print(NBR_intBuffer[0]);Serial.print(" ");Serial.print(NBR_intBuffer[1]);Serial.print(" ");Serial.println(NBR_intBuffer[2]);
        NBR_stringBuffer = "";
        NBR_readFlag = false;
        if (NBR_intBuffer[0] != -1) {
          Serial.print("Pushing values to motors.");
        }
        else  {
          NBR_intBuffer[0] = -1;
        }
      }
      else  {
        NBR_stringBuffer += NBR_c;
      }
    }
    else if (Serial.available()) {
      if (Serial.peek() == 33)  {   // Exclamation mark
        NBR_stringBuffer = "";
        Serial.read();
        NBR_readFlag = true;
      }
    }
    // end of non-blocking read-system
    if (NBR_intBuffer[0] == 0 && NBR_intBuffer[1] == 0 && NBR_intBuffer[2] == 0) {
      firstESC.writeMicroseconds(1000);
      secondESC.writeMicroseconds(1000);
      thirdESC.writeMicroseconds(1000);
      fourthESC.writeMicroseconds(1000);
      Serial.println("Moving to pause state.");
      while (true)  {}
    }
    else if (NBR_intBuffer[0] >= 0) {
      rollPID.SetTunings(NBR_intBuffer[0], NBR_intBuffer[1], NBR_intBuffer[2]);
    }
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available

     //PID Calculate
    rollInput = ypr[2];
    rollPID.Compute();
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print("\t");
    Serial.println(rollOutput);

    speed2 = baseline - (int)rollOutput*10/2;
    speed4 = baseline + (int)rollOutput*10/2;
    
    if (speed2 < 1200) speed2 = 1200;
    if (speed4 < 1200) speed4 = 1200;
    if (speed2 > 1800) speed2 = 1800;
    if (speed4 > 1800) speed4 = 1800;
    
    //firstESC.writeMicroseconds(value);
    secondESC.writeMicroseconds(speed2);
    //thirdESC.writeMicroseconds(value);
    fourthESC.writeMicroseconds(speed4);
    
    while (!mpuInterrupt && fifoCount < packetSize) {
       
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x01) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

       
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        /*Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI); */
              
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

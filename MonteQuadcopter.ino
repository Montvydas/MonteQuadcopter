#include <SoftwareSerial.h>
SoftwareSerial mySerial (7, 8); //RX, TX
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <EEPROM.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file


#define ADDR_PITCH_OFFSET 0
#define ADDR_ROLL_OFFSET 1
#define FL_MOTOR 3
#define FR_MOTOR 9
#define BR_MOTOR 10
#define BL_MOTOR 11

//---------------------------------PID------------------------------------
//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

//Define the aggressive and conservative Tuning Parameters
double consKp = 0.5, consKi = 0.05, consKd = 0.05;

PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);
//------------------------------------------------------------------------


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

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
float ypr_cal[3];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


#define ALPHA 0.1
#define MULTIPLIER 6.67
float motorBattery;

int targetSpeed[4];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 3; i++) {
    ypr_cal[i] = 0.0;
  }

  //Enable internal reference of 1.1V
  //initialise battery level array with current battery level value
  pinMode(A0, INPUT);
  analogReference(INTERNAL);
//  float tmp = analogRead(A0) / 1023.0 * MULTIPLIER;
  motorBattery = 4.2;
  //------------------------------PID----------------------------------
  //initialize the variables we're linked to
  pitchInput = 0.0;
  rollInput = 0.0;

  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;

  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
  //-------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  }

  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize mySerial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  mySerial.begin(9600);
  while (!mySerial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  mySerial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  mySerial.println(F("Testing device connections..."));
  mySerial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  mySerial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  while (mySerial.available() && mySerial.read()); // empty buffer again

  // load and configure the DMP
  mySerial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mySerial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    mySerial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    mySerial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    mySerial.print(F("DMP Initialization failed (code "));
    mySerial.print(devStatus);
    mySerial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int myReading = 0;
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (mySerial.available()) {
    myReading = mySerial.parseInt();
    for (int i = 0; i < 4; i++) {
      targetSpeed[i] = myReading;
    }
    while (mySerial.available())  //flushing anything that wasn't read
      mySerial.read();
    //      runIndividual(myReading);
    //      checkMotor(myReading);
    //        checkIndividual(myReading);
  }

  // wait for MPU interrupt or extra packet(s) available
//      while (!mpuInterrupt && fifoCount < packetSize) {
  // if you are really paranoid you can frequently test in between other
  // stuff to see if mpuInterrupt is true, and if so, "break;" from the
  // while() loop to immediately process the MPU data
  //    }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //        mySerial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
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

    //----------------------------------------PID-----------------------------------------
    if (myReading == 0) {
      Serial.println(F("CALIBRATING"));
      ypr_cal[0] = ypr[0] * 180 / M_PI;
      ypr_cal[1] = ypr[1] * 180 / M_PI;
      ypr_cal[2] = ypr[2] * 180 / M_PI;

      //              ypr_cal[1] = 1.26;
      //              ypr_cal[2] = -0.55;
    }

    pitchInput = ypr[1] * 180 / M_PI - ypr_cal[1];
    rollInput = ypr[2] * 180 / M_PI - ypr_cal[2];

    //            pitchInput /= 2.0;
    //            rollInput /= 2.0;

    pitchPID.Compute();
    rollPID.Compute();

    int actSpeed[4];
    stabilise (targetSpeed, actSpeed, rollOutput, pitchOutput);
//    targetSpeed = actSpeed; // should this behere or not?

    Serial.print(F("pitchInput="));
    Serial.print(pitchInput);
    Serial.print(F("   pitchOutput="));
    Serial.print(pitchOutput);

    Serial.print(F("   rollInput="));
    Serial.print(rollInput);
    Serial.print(F("   rollOutput="));
    Serial.print(rollOutput);

    Serial.print(F("   mot[0]="));
    Serial.print(actSpeed[0]);
    Serial.print(F("   mot[1]="));
    Serial.print(actSpeed[1]);
    Serial.print(F("   mot[2]="));
    Serial.print(actSpeed[2]);
    Serial.print(F("   mot[3]="));
    Serial.println(actSpeed[3]);

    runIndividual (actSpeed);
    //            checkIndividual(myReading, actSpeed);
    //------------------------------------------------------------------------------------
    motorBattery = smoothBattery(motorBattery, analogRead(A0) / 1023.0 * MULTIPLIER, ALPHA);
    if (motorBattery < 2.0){
      mySerial.println (F("WARNING! LOW BATTERY!"));
    }
    mySerial.print(motorBattery);
    mySerial.print(F("   ypr   "));
    mySerial.print(ypr[0] * 180 / M_PI);
    mySerial.print("   ");
    mySerial.print(ypr[1] * 180 / M_PI);
    mySerial.print("   ");
    mySerial.println(ypr[2] * 180 / M_PI);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  delay(100);
}

float smoothBattery (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}

void setSpeed(int val) {
  analogWrite(FL_MOTOR, val);
  analogWrite(FR_MOTOR, val);
  analogWrite(BR_MOTOR, val);
  analogWrite(BL_MOTOR, val);
}

void checkMotor(int motor) {
  analogWrite(FL_MOTOR, 0);
  analogWrite(FR_MOTOR, 0);
  analogWrite(BR_MOTOR, 0);
  analogWrite(BL_MOTOR, 0);

  if (motor == 0)
    analogWrite(FL_MOTOR, 10);
  if (motor == 1)
    analogWrite(FR_MOTOR, 10);
  if (motor == 2)
    analogWrite(BR_MOTOR, 10);
  if (motor == 3)
    analogWrite(BL_MOTOR, 10);
}

void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  actSpeed[0] = (int) currSpeed[0] + (rollDiff) - (pitchDiff);  //each motor has actual Speed and speed at which we want them to fly...
  actSpeed[1] = (int) currSpeed[1] + (rollDiff) + (pitchDiff);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff) + (pitchDiff);  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[3] = (int) currSpeed[3] - (rollDiff) - (pitchDiff);

  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0 )
      actSpeed[i] = 0;
  }
}

void checkIndividual (int motor, int* actSpeed) {
  analogWrite(FL_MOTOR, 0);
  analogWrite(FR_MOTOR, 0);
  analogWrite(BR_MOTOR, 0);
  analogWrite(BL_MOTOR, 0);

  if (motor == 0)
    analogWrite(FL_MOTOR, actSpeed[0]);
  if (motor == 1)
    analogWrite(FR_MOTOR, actSpeed[1]);
  if (motor == 2)
    analogWrite(BR_MOTOR, actSpeed[2]);
  if (motor == 3)
    analogWrite(BL_MOTOR, actSpeed[3]);
}

void runIndividual (int* actSpeed) {
  analogWrite(FL_MOTOR, actSpeed[0]);
  analogWrite(FR_MOTOR, actSpeed[1]);
  analogWrite(BR_MOTOR, actSpeed[2]);
  analogWrite(BL_MOTOR, actSpeed[3]);
}

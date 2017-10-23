/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"
#include "Quadcopter.h"
#include "BatteryLevel.h"
#include "SoftwareSerial.h"
#include "ButterFilter.h"
#include "PinChangeInterrupt.h"

// ================================================================
// ===                    MPU6050 Stuff                         ===
// ================================================================

long prevTime = micros();
long currTime = micros();

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN A2  // use analogue pin A2 -> I know, very strange but easier to wire up
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
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yprOffset[3];    // offset to make ypr initially be all zero

// define which one to use, need to test which works best
#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL

// ================================================================
// ===                    Filter stuff                          ===
// ================================================================

ButterFilter pitchFilter;
ButterFilter rollFilter;

// ================================================================
// ===                    Motors stuff                          ===
// ================================================================
#define FL 3
#define FR 9
#define BL 11
#define BR 10

Quadcopter quad(FL, FR, BL, BR);
int targetSpeed[4];

// ================================================================
// ===                    PID stuff                             ===
// ================================================================
//Define Variables we'll be connecting to
int16_t gx, gy, gz;

double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=0.98, consKi=0.08, consKd=0.0;

PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);

// Look into option of using Proportional on Measurement instead of Proportional on Erro!
//PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
//PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, P_ON_M, DIRECT);

// ================================================================
// ===                 Battery Level stuff                      ===
// ================================================================

// ================================================================
// ===                 Communication stuff                      ===
// ================================================================

SoftwareSerial bluetooth (7, 8); //RX, TX

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
//  if (digitalRead(A0)==1)  Serial.println("A0");
//  if (digitalRead(A1)==1)  Serial.println("A1");
//  if (digitalRead(A2)==1)  {Serial.println("A2");}
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    bluetooth.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // Initialise settings for IMU
    initIMU();
    initPID();

    pitchFilter.begin(12, 200);
    rollFilter.begin(12, 200);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while ((!mpuInterrupt && fifoCount < packetSize) || (Serial.available() > 0 || bluetooth.available() > 0)) {
        if (bluetooth.available() > 0) 
        {
          /*
        }
          char mode = bluetooth.read();  
          switch(mode){
          case 'c':
            calibrateIMU();
            break;
          case 'a':
            quad.arm();
            break;
          case 'd':
            quad.disarm();
            break;
          case '0':
            setTargetSpeed(0);
            break;
          case '1':
            setTargetSpeed(50);
            break;          
          case '2':
            setTargetSpeed(100);
            break;            
          case '3':
            setTargetSpeed(120);
            break;    
          case '4':
            setTargetSpeed(150);
            break;                   
          case '5':
            setTargetSpeed(180);
            break;          
          case 'P':
             mode = Serial.read();  
             if (mode == '+')
               consKp += 0.1;
             if (mode == '-')
               consKp -= 0.1;
             pitchPID.SetTunings(consKp, consKi, consKd);
//          Serial.print("kp=");
//          Serial.println(consKp);
          break;
          case 'I':
             mode = Serial.read();  
             if (mode == '+')
               consKi += 0.01;
             if (mode == '-')
               consKi -= 0.01;
//             Serial.print("ki=");
//             Serial.println(consKi);
             pitchPID.SetTunings(consKp, consKi, consKd);
              break;
          case 'D':
             mode = Serial.read();  
             if (mode == '+')
               consKd += 0.001;
             if (mode == '-')
               consKd -= 0.001;
//             Serial.print("kd=");
//             Serial.println(consKd);
             pitchPID.SetTunings(consKp, consKi, consKd);
             break;
          } 
          */
          
//          while (!bluetooth.available());                 // wait for data
          while (Serial.available() && Serial.read());   // empty buffer
          while (bluetooth.available() && bluetooth.read());   // empty buffer
        }
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
//        processPID();
        processRatePID();
    }

    processRateIMU();
//    processIMU();
}

void setTargetSpeed(int one){
  for (int i = 0; i < 4; i++)
    targetSpeed[i] = one;
}


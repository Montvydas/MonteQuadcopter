#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial (7, 8); //RX, TX

#define FL_MOTOR 3
#define FR_MOTOR 9
#define BR_MOTOR 10
#define BL_MOTOR 11

int speed = 40;
void setup() {
  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);

  mySerial.begin(9600);
  //  Serial.begin(9600);
}


void loop() {
  if (mySerial.available()) {
    int myReading = mySerial.parseInt();
    setSpeedForAll(myReading);
    // checkIndividualMotor(myReading);     // Is used to check which motor is which
    while (mySerial.available())  //flushing anything that wasn't read
      mySerial.read();
  }
}

void setSpeedForAll(int val) {
  analogWrite(FL_MOTOR, val);
  analogWrite(FR_MOTOR, val);
  analogWrite(BR_MOTOR, val);
  analogWrite(BL_MOTOR, val);
}

void checkIndividualMotor(int motor) {
  analogWrite(FL_MOTOR, 0);
  analogWrite(FR_MOTOR, 0);
  analogWrite(BR_MOTOR, 0);
  analogWrite(BL_MOTOR, 0);

  switch(motor){
    case 0:
      analogWrite(FL_MOTOR, 10);
      break;
    case 1:
      analogWrite(FR_MOTOR, 10);
      break;
    case 2:
      analogWrite(BR_MOTOR, 10);
      break;
    case 3:
      analogWrite(BL_MOTOR, 10);
      break;
    default:
      break;
  }
}

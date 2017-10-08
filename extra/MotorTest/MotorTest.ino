#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Quadcopter.h>

#define FL 3
#define FR 9
#define BL 11
#define BR 10

Quadcopter quad(FL, FR, BL, BR);

SoftwareSerial mySerial (7, 8); //RX, TX

int mySpeed[] = {20, 20, 20, 20};

void printCurrentSpeed();

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
//  testMotors();
}


void loop() {
  if (mySerial.available()) {
    int myReading = mySerial.parseInt();
    
    quad.setConstantSpeed(myReading);
    Serial.print(myReading);
    while (mySerial.available() && mySerial.read());  //flushing anything that wasn't read
  }
}

void printCurrentSpeed(){
  int* currSpeed = quad.getSpeed();
  for (int i = 0; i < 4; i++){
    Serial.print(currSpeed[i]);
    Serial.print(" ");
    mySerial.print(currSpeed[i]);
    mySerial.print(" ");    
  }
  Serial.println();
  mySerial.println();
}

void testMotors(){
    Serial.println("setConstantSpeed");
  quad.setConstantSpeed(15);
  printCurrentSpeed();
  delay(2000);

  Serial.println("arm");
  quad.arm();
  printCurrentSpeed();
  delay(2000);

  Serial.println("disarm");
  quad.disarm();
  printCurrentSpeed();
  delay(2000);

  Serial.println("setSpeed");
  quad.setSpeed(mySpeed);
  printCurrentSpeed();
  delay(2000);

  Serial.println("setSingleMotor");
  quad.setSingleMotor(0);
  printCurrentSpeed();
  delay(2000);

  Serial.println("getStabilisedSpeed");
  quad.getStabilisedSpeed(mySpeed, 15.2, -10.6);
  quad.setSpeed(quad.getSpeed());
  printCurrentSpeed();
  delay(2000);

  Serial.println("disarm");
  quad.disarm();
  printCurrentSpeed();
  delay(2000);
}


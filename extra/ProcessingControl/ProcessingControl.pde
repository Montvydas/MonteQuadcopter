import processing.serial.*;

Serial myPort;  // The serial port
int value = 0;

void setup() {
  size(640, 360);
  // List all the available serial ports
  printArray(Serial.list());
  String portName = "/dev/tty.30-15-01-14-20-21-DevB";
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, portName, 9600);
}

void draw() {
  while (myPort.available() > 0) {
    float inByte = myPort.read();
    println(inByte);
  }
  
  fill(value);
  rect(25, 25, 50, 50);
}

void keyPressed() {
  if (value == 0) {
    value = 255;
  } else {
    value = 0;
  }
  myPort.write(value);
}
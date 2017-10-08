int battVolts;   // made global for wider avaliblity throughout a sketch if needed, example a low voltage alarm, etc

#define ALPHA 0.1
#define MULTIPLIER 6.67
float battery;

void setup(void)
{
  Serial.begin(9600);
  Serial.print("volts X 100");
  Serial.println( "\r\n\r\n" );
  delay(100);
  pinMode(A0, INPUT);
  analogReference(INTERNAL);
  battery = 4.2;
}

void loop(void)
{
  battery = smoothBattery(battery, analogRead(A0) / 1023.0 * MULTIPLIER, ALPHA);
  Serial.print("Battery Level: ");
  Serial.print(battery);
  Serial.println("V");
  delay(1000);
}

float smoothBattery (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}

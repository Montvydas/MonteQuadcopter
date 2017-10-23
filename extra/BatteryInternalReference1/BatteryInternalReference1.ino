#include <BatteryLevel.h>

BatteryLevel batteryLevel(0);    //this refers to anlog pin A0

void setup(void)
{
  Serial.begin(115200);
}

void loop(void)
{
  float battery = batteryLevel.getHardwareBatteryLevel();
  battery = batteryLevel.getBatteryLevel();
  
  Serial.print("Battery Level: ");
  Serial.print(battery);
  Serial.println("V");
  delay(500);
}

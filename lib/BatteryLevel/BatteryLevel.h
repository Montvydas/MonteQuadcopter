/*
 BatteryLevel.cpp - Library for getting battery level.
 Created by Montvydas Klumbys, 8 October, 2017.
 */

#ifndef BatteryLevel_h
#define BatteryLevel_h
#include "Arduino.h"

#define INIT_BATTERY 4.2
#define ALPHA 0.1
#define MULTIPLIER 6.67

class BatteryLevel
{
    public:
    BatteryLevel(int);
    float getBatteryLevel();
    float getHardwareBatteryLevel();
    float smooth(float, float, float);

    private:
      float batteryLevel;
      int pin;
};

#endif

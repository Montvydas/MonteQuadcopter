/*
 BatteryLevel.cpp - Library for getting battery level.
 Created by Montvydas Klumbys, 8 October, 2017.
 */

#include "Arduino.h"
#include "BatteryLevel.h"

// constructor
BatteryLevel::BatteryLevel(int pin)
{
  this->pin = pin;
  pinMode(this->pin, INPUT);
  analogReference(INTERNAL);
  this->batteryLevel = INIT_BATTERY;
}

float BatteryLevel::getHardwareBatteryLevel()
{
  this->batteryLevel = smooth(this->batteryLevel, analogRead(this->pin) / 1023.0 * MULTIPLIER, ALPHA);
  return this->batteryLevel;
}

float BatteryLevel::getBatteryLevel(){
  return this->batteryLevel;
}

float BatteryLevel::smooth (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}

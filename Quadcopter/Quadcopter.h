/*
 Quadcopter.cpp - Library for quadcopter control.
 Created by Montvydas Klumbys, 5 October, 2017.
 */

#ifndef Quadcopter_h
#define Quadcopter_h

#include "Arduino.h"

#define ARM_SPEED 10

class Quadcopter
{
    public:
    Quadcopter();
    void setConstantSpeed(int);
    void arm();
    void disarm();
    void setSpeed(int*);
    void setSingleMotor(int);
    int* getSpeed();
    int* getStabilisedSpeed();
    private:
      speed[4];
      motor[4];
};

#endif

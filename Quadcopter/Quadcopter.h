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
    Quadcopter(int, int, int, int);
    void setConstantSpeed(int);
    void arm();
    void disarm();
    void setSpeed(int*);
    void setSingleMotor(int);
    int* getSpeed();
    int* getStabilisedSpeed(int*, float, float);
    private:
      int speed[4];
      int motor[4];
};

#endif

/*
 ButterFilter.cpp - Library for 2nd Order butterworth filter.
 Created by Montvydas Klumbys, 5 October, 2017.
 */

#ifndef ButterFilter_h
#define ButterFilter_h

#include "Arduino.h"

class ButterFilter
{
    public:
    ButterFilter();
    void begin(float, float);
    float* getCoeffs();
    float filter(float);
    float getCutoff();
    float getFs();
    
    private:
    float v[3];
    float a[3];
    float cutoff;
    float fs;
};

#endif

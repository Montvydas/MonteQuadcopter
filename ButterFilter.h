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
    void setup(float, float);
    float* getCoeffs();
    float filter(float);
    
    private:
    float[] v = {0, 0, 0};
    float[] a = {0, 0, 0};
};

#endif

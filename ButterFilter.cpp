/*
 ButterFilter.cpp - Library for 2nd Order butterworth filter.
 Created by Montvydas Klumbys, 5 October, 2017.
 */

#include "Arduino.h"
#include "ButterFilter.h"
#include "math.h"

#define PI 3.141592

// constructor
ButterFilter::ButterFilter()
{
    v = [0, 0, 0];
}

// setup out filter
void ButterFilter::setup(float cutoff, float fs)
{
    this->cutoff = cutoff;
    this->fs = fs;
    
    float f = math.tan(PI * cutoff / fs);
    float f2 = f * f;
    sq2 = math.sqrt(2);
    
    float a0 = 1 + sq2 * f + f2;
    
    // Coefficients for 2nd order buttorworth filter
    this->a[1] = -2 * (f2 - 1) / a0;
    this->a[2] = -(1 - sq2 * f + f2) / a0;
    this->a[0] = f2 / a0;
}

// Used to get calculated coeeficients for testing
float* ButterFilter::getCoeffs()
{
    return a;
}

// Is used to apply a filter and get a new value
float ButterFilter::filter(float data)
{
    this->v[0] = this->v[1];
    this->v[1] = this->v[2];
    this->v[2] = this->a[0] * data + this->a[2] * this->v[0] + this->a[1] * this->v[1];
    
    return (this->v[0] + this->v[2])+ 2 * this->v[1];
}

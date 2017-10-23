/*
 Quadcopter.cpp - Library for quadcopter control.
 Created by Montvydas Klumbys, 5 October, 2017.
 */

#include "Arduino.h"
#include "Quadcopter.h"

// constructor
Quadcopter::Quadcopter(int m0, int m1, int m2, int m3)
{
  this->armed = false;
  // Front Left
  this->motor[0] = m0;
  // Front Right
  this->motor[1] = m1;
  // Back Left
  this->motor[2] = m2;
  // Back Right
  this->motor[3] = m3;

  // set all pins to be output
  for (int i = 0; i < 4; i++)
  {
    pinMode(this->motor[i], OUTPUT);
    this->speed[i] = 0;
  }
}

void Quadcopter::setConstantSpeed(int speed)
{
  for (int i = 0; i < 4; i++)
  {
    analogWrite(this->motor[i], speed);
    this->speed[i] = speed;
  }
}

void Quadcopter::arm()
{
  this->armed = true;
  this->setConstantSpeed(ARM_SPEED);
}

void Quadcopter::disarm()
{
  this->armed = false;
  this->setConstantSpeed(0);
}

void Quadcopter::setSpeed(int* speed)
{
  for (int i = 0; i < 4; i++)
  {
    analogWrite(this->motor[i], speed[i]);
    this->speed[i] = speed[i];
  }
}

void Quadcopter::setSingleMotor(int motor_index, int speed)
{
  // firstly disarm
  this->disarm();

  if (motor_index >= 0 && motor_index <= 3)
  {
    // this could be improved to use setSpeed instead of reinventing a function
    analogWrite(this->motor[motor_index], speed);
    this->speed[motor_index] = speed;
  }
}

int* Quadcopter::getSpeed()
{
  return this->speed;
}

bool Quadcopter::isArmed()
{
  return this->armed;
}

void Quadcopter::getStabilisedSpeed(int* speed, int* actSpeed, float rollDiff, float pitchDiff){
  //each motor has actual Speed and speed at which we want them to fly...
  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  this->speed[0] = (int) speed[0] + (rollDiff) - (pitchDiff);
  this->speed[1] = (int) speed[1] + (rollDiff) + (pitchDiff);
  this->speed[2] = (int) speed[2] - (rollDiff) - (pitchDiff);
  this->speed[3] = (int) speed[3] - (rollDiff) + (pitchDiff);


  for (int i = 0; i < 4; i++) {
    if (this->speed[i] < 0 )
      this->speed[i] = 0;
    if (this->speed[i] > 255)
      this->speed[i] = 255;

    actSpeed[i] = this->speed[i];
  }
  return this->speed;
}

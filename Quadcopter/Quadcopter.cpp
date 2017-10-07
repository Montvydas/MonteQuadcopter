/*
 Quadcopter.cpp - Library for quadcopter control.
 Created by Montvydas Klumbys, 5 October, 2017.
 */

#include "Arduino.h"
#include "Quadcopter.h"

// constructor
Quadcopter::Quadcopter(int m0, int m1, int m2, int m3)
{
  this->motor[0] = m0;
  this->motor[1] = m1;
  this->motor[2] = m2;
  this->motor[3] = m3;

  // set all pins to be output
  for (int i = 0; i < 4; i++)
    pinMode(this->motor[i], OUTPUT);
}

void uadcopter::setConstantSpeed(int speed)
{
  for (int i = 0; i < 4; i++)
    analogWrite(this->motor[i], speed);
}

void Quadcopter::arm()
{
  this->setConstantSpeed(ARM_SPEED);
}

void Quadcopter::disarm()
{
  this->setConstantSpeed(0);
}

void Quadcopter::setSpeed(int* speed)
{
  this->speed = speed;
  for (int i = 0; i < 4; i++)
    analogWrite(this->motor[i], speed[i]);
}

void Quadcopter::setSingleMotor(int motor_index)
{
  // firstly disarm
  this->disarm();

  if (motor_index >= 0 && motor_index <= 3)
    analogWrite(this->motor[motor_index], ARM_SPEED);
}

int* Quadcopter::getSpeed()
{
  return this->speed;
}

int* Quadcopter::getStabilisedSpeed(int* speed, float rollDiff, float pitchDiff){
  //each motor has actual Speed and speed at which we want them to fly...
  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  this->speed[0] = (int) speed[0] + (rollDiff) - (pitchDiff);
  this->speed[1] = (int) speed[1] + (rollDiff) + (pitchDiff);
  this->speed[2] = (int) speed[2] - (rollDiff) + (pitchDiff);
  this->speed[3] = (int) speed[3] - (rollDiff) - (pitchDiff);

  for (int i = 0; i < 4; i ++) {
    if (this->speed[i] < 0 )
      this->speed[i] = 0;
  }
  return this->speed;
}

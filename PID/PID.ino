#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

double ypr[3];
double yprOffset[3];

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);

// Look into option of using Proportional on Measurement instead of Proportional on Erro!
//PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, P_ON_M, DIRECT); 
//PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, P_ON_M, DIRECT);

void initPID();
void processPID();
void setTunings(PID, double, double, double);

void setup()
{
  initPID();
}

void loop()
{
  processPID();
}

void initPID()
{
  //initialize the variables we're linked to
  pitchInput = 0.0;
  rollInput = 0.0;

  // try to hover at 0 degrees at first
  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;

  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);

  for (int i = 0; i < 4; i++)
    yprOffset[i] = 0.0;
}

void processPID()
{
  // This peace of code is used to calibrate
  bool isCalibrating = false;
  if (isCalibrating)
    for (int i = 0; i < 4; i++)
      yprOffset[i] = ypr[i];

  // remove offset
  for (int i = 0; i < 4; i++)
    ypr[i] -= yprOffset[i];  

  // assign to PID variables -> in the future maybe jsut assign directly when creating PID?
//  yawInput = ypr[0] * 180 / PI;
  pitchInput = ypr[1] * 180 / PI;
  rollInput = ypr[2] * 180 / PI;
 
  setTunings(pitchPID, pitchSetpoint, pitchInput, 10);
  setTunings(rollPID, rollSetpoint, rollInput, 10);

  pitchPID.Compute();
  rollPID.Compute();
}

void setTunings(PID pid, double setpoint, double input, double gapOffset)
{
  double gap = abs(setpoint-input); //distance away from setpoint
  if (gap < gapOffset)
    pid.SetTunings(consKp, consKi, consKd);
  else
     pid.SetTunings(aggKp, aggKi, aggKd);
}



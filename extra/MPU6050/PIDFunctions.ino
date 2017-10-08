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

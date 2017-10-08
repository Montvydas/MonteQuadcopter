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
}

void processPID()
{

  // assign to PID variables -> in the future maybe jsut assign directly when creating PID?
  // Also remove offsets
//  yawInput = (ypr[0] - yprOffset[0]) * 180 / PI;
  pitchInput = (ypr[1] - yprOffset[1]) * 180 / PI;
  rollInput = (ypr[2] - yprOffset[2]) * 180 / PI;

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

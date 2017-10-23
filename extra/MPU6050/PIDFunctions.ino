#define PID_SIZE 5
#define PID_ROLL_RATE   0
#define PID_ROLL_ANGLE  1
#define PID_PITCH_RATE  2
#define PID_PITCH_ANGLE 3
#define PID_YAW_RATE    4

double pidInputs[5];
double pidOutputs[5];
double pidSetpoints[5];

PID pitchRatePID(&pidInputs[PID_PITCH_RATE], &pidOutputs[PID_PITCH_RATE],&pidSetpoints[PID_PITCH_RATE],1.86, 0.36, 0.0, REVERSE);

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

  pitchPID.SetOutputLimits(-50, 50);
  rollPID.SetOutputLimits(-50, 50);

  pitchPID.SetSampleTime(5);
  rollPID.SetSampleTime(5);
}

void processRatePID(){
  
}

void processPID()
{

  // assign to PID variables -> in the future maybe jsut assign directly when creating PID?
  // Also remove offsets
//  yawInput = (ypr[0] - yprOffset[0]) * 180 / PI;
  pitchInput = pitchFilter.filter((ypr[1] - yprOffset[1]) * 180 / PI);
  rollInput = rollFilter.filter((ypr[2] - yprOffset[2]) * 180 / PI);

//  setTunings(pitchPID, pitchSetpoint, pitchInput, 10);
//  setTunings(rollPID, rollSetpoint, rollInput, 10);

  bool isPitchCalculated = pitchPID.Compute();
  bool isRollCalculated = rollPID.Compute();

  if (!quad.isArmed()){
//    Serial.print("pitchOutput=");
//    Serial.print(pitchOutput);
//    Serial.print("   rollOutput=");
//    Serial.print(rollOutput);
//    Serial.println();
  }


  if (quad.isArmed() && (isPitchCalculated || isRollCalculated)){
    int mySpeed[4];
    quad.getStabilisedSpeed(targetSpeed, mySpeed, 0, pitchOutput);
    quad.setSpeed(mySpeed);
//    Serial.print("  1=");
//    Serial.print(mySpeed[0]);
//    Serial.print("  2=");
//    Serial.print(mySpeed[1]);
//    Serial.print("  3=");
//    Serial.print(mySpeed[2]);
//    Serial.print("  4=");
//    Serial.print(mySpeed[3]);
  } else {
//    Serial.print("  y=");
//    Serial.print((ypr[0] - yprOffset[0]) * 180/PI);
//    Serial.print("  p=");
//    Serial.print((ypr[1] - yprOffset[1]) * 180/PI);
//    Serial.print("  r=");
//    Serial.print((ypr[2] - yprOffset[2]) * 180/PI);
  }
//  Serial.println();
}

void setTunings(PID pid, double setpoint, double input, double gapOffset)
{
  double gap = abs(setpoint-input); //distance away from setpoint
  if (gap < gapOffset)
    pid.SetTunings(consKp, consKi, consKd);
  else
     pid.SetTunings(aggKp, aggKi, aggKd);
}

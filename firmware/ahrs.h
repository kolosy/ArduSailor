#ifndef ahrs_h
#define ahrs_h

#include "Arduino.h"

// reporting value
extern float current_pitch;
extern float current_roll;

float readSteadyHeading();
int mpuInit();

float toCircle(float value);
void writeCalibrationLine();
#endif

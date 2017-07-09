#ifndef ahrs_h
#define ahrs_h

#include "Arduino.h"

// reporting value
extern float current_pitch;
extern float current_roll;
extern float mag_offset;

float readSteadyHeading();
int mpuInit();

float toCircle(float value);
void writeCalibrationLine();

struct AngleCmp {
	float s;
	float c;
};

#endif

#ifndef ahrs_h
#define ahrs_h

#include "Arduino.h"

// reporting value
extern float current_pitch;
extern float current_roll;
extern float mag_offset;

float readSteadyHeading();
int mpuInit(int16_t settingsAddress);
void calibrateMag(bool waitForSetup);

float toCircle(float value);

struct AngleCmp {
	float s;
	float c;
};

#endif

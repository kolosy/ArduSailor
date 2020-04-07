#ifndef ahrs_h
#define ahrs_h

#include "Arduino.h"

#ifndef PILOT_DEBUG
// reporting value
extern float current_pitch;
extern float current_roll;
extern float mag_offset;

typedef unsigned char prog_uchar;

float readSteadyHeading();
int mpuInit(int16_t settingsAddress);
void calibrateMag(bool waitForSetup);

float toCircle(float value);
#endif 

struct AngleCmp {
	float s;
	float c;
};

#endif

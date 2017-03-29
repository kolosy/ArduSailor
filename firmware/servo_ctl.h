#ifndef __servo_ctl
#define __servo_ctl

#include "Arduino.h"

#define RUDDER_MIN 50
#define RUDDER_MAX 125

#define WINCH_MAX 56
#define WINCH_MIN 120

extern int heel_offset;
extern uint8_t current_rudder;
extern uint8_t current_winch;

void servoInit();
void centerWinch();
void centerRudder();
void winchTo(int value);
void normalizedWinchTo(int value);
void normalizedWinchTo(int value, int min, int max);
void rudderFromCenter(int value);
void rudderTo(int value);

#endif
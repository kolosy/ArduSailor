#include "servo_ctl.h"

#include <Servo.h>
#include "logger.h"

#define WINCH_PORT 11
#define RUDDER_PORT 10

#define WINCH_EN 27
#define RUDDER_EN 26

#define SP_EN 25

// degrees per second
#define RUDDER_SPEED 300.0
#define WINCH_SPEED 25.0

int heel_offset = 0;
uint8_t current_rudder = 0;
uint8_t current_winch = 0;

bool motor_running = false;

Servo sv_winch, sv_rudder;

void servoInit() {
	pinMode(SP_EN, OUTPUT);
	digitalWrite(SP_EN, LOW);

	pinMode(WINCH_EN, OUTPUT);
	digitalWrite(WINCH_EN, LOW);

	pinMode(RUDDER_EN, OUTPUT);
	digitalWrite(RUDDER_EN, LOW);

#ifndef NO_SAIL
	sv_winch.attach(WINCH_PORT);
#endif

	sv_rudder.attach(RUDDER_PORT);
}

#ifdef NO_SAIL
void runMotor() {
	if (motor_running)
		return;

	digitalWrite(SP_EN, HIGH);
	delay(10);
	digitalWrite(WINCH_EN, HIGH);
	motor_running = true;
}

void stopMotor() {
	if (!motor_running)
		return;

	digitalWrite(WINCH_EN, LOW);
	digitalWrite(SP_EN, LOW);

	motor_running = false;
}
#endif

void centerWinch() {
	winchTo(WINCH_MAX);
}

void centerRudder() {
	rudderTo(90 + heel_offset);
}

void winchTo(int value) {
#ifdef NO_SAIL
	return;
#endif

	int v = constrain(value, min(WINCH_MIN, WINCH_MAX), max(WINCH_MIN, WINCH_MAX));

	logln(F("Winch to %d, constrained to %d"), value, v);

	if (current_winch == v)
		return;

	digitalWrite(SP_EN, HIGH);
	delay(10);
	digitalWrite(WINCH_EN, HIGH);
	delay(10);
	sv_winch.write(v);
	delay(1000 * (abs((current_winch - v))/WINCH_SPEED) + 150);
	digitalWrite(WINCH_EN, LOW);
	digitalWrite(SP_EN, LOW);

	current_winch = v;
}

void normalizedWinchTo(int value) {
	winchTo(map(value, 0, 90, WINCH_MIN, WINCH_MAX));
}

void normalizedWinchTo(int value, int min, int max) {
	winchTo(map(value, min, max, WINCH_MIN, WINCH_MAX));
}

void rudderFromCenter(int value) {
	rudderTo(90 + value);
}

void rudderTo(int value) {
	logln(F("Rudder to %d"), value);

	int v = constrain(value + heel_offset, RUDDER_MIN, RUDDER_MAX);

	if (current_rudder == v)
		return;

	digitalWrite(SP_EN, HIGH);
	delay(10);
	digitalWrite(RUDDER_EN, HIGH);
	delay(10);
	sv_rudder.write(v);
	delay(1000 * (abs((current_rudder - v))/RUDDER_SPEED) + 150);
	digitalWrite(RUDDER_EN, LOW);

#ifndef NO_SAIL
	digitalWrite(SP_EN, LOW);
#else
	if (!motor_running)
		digitalWrite(SP_EN, LOW);
#endif

	current_rudder = v;
}

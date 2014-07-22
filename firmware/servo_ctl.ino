#define WINCH_PORT 5
#define RUDDER_PORT 6

#define WINCH_EN A2
#define RUDDER_EN A3

#define RUDDER_MIN 55
#define RUDDER_MAX 125
#define WINCH_MIN 60
#define WINCH_MAX 115

Servo sv_winch, sv_rudder;

// servo absolute limits:
// rudder - 90 (center) +/-40
// winch	- 60 -> 115 (center)
void servoInit() {
	pinMode(WINCH_EN, OUTPUT);
	digitalWrite(WINCH_EN, HIGH);

	pinMode(RUDDER_EN, OUTPUT);
	digitalWrite(RUDDER_EN, LOW);

	sv_winch.attach(WINCH_PORT);
	delay(100);
	digitalWrite(WINCH_EN, LOW);
	
	digitalWrite(RUDDER_EN, HIGH);
	sv_rudder.attach(RUDDER_PORT);	
	delay(100);
	digitalWrite(RUDDER_EN, LOW);
}

void centerWinch() {
	winchTo(115);
}

void centerRudder() {
	rudderTo(90);
}

void bothTo(int rudder, int winch) {
	int rv = constrain(rudder, RUDDER_MIN, WINCH_MAX);
	int wv = constrain(winch, WINCH_MIN, WINCH_MAX);
	
	int r_step = (current_rudder - rv) / 10;
	int w_step = (current_winch - wv) / 10;

	digitalWrite(RUDDER_EN, HIGH);
	digitalWrite(WINCH_EN, HIGH);
	delay(100);

	for (int i=0; i<10; i++) {
		current_rudder += r_step;
		current_winch += w_step;

		// split the waits in between... don't want to have both servos going at 
		// the same time if we can help it		
		sv_rudder.write(current_rudder);
		delay(100);
		sv_winch.write(current_winch);
		delay(100);
	}
	
	digitalWrite(RUDDER_EN, LOW);
	digitalWrite(WINCH_EN, LOW);
}

void winchTo(int value) {
	int v = constrain(value, WINCH_MIN, WINCH_MAX);
	
	digitalWrite(WINCH_EN, HIGH);
	delay(100);
	sv_winch.write(v);
	delay(2500);
	digitalWrite(WINCH_EN, LOW);
	
	current_winch = v;
}

void rudderTo(int value) {
	int v = constrain(value, RUDDER_MIN, RUDDER_MAX);
	
	digitalWrite(RUDDER_EN, HIGH);
	delay(100);
	sv_rudder.write(v);
	delay(1500);
	digitalWrite(RUDDER_EN, LOW);
	
	current_rudder = v;
}

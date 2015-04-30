#define WINCH_PORT 11
#define RUDDER_PORT 10

#define WINCH_EN 27
#define RUDDER_EN 26

#define SP_EN 25

#define RUDDER_MIN 50
#define RUDDER_MAX 125
#define WINCH_MIN 60
#define WINCH_MAX 115

// degrees per second
#define RUDDER_SPEED 300.0
#define WINCH_SPEED 277.0

Servo sv_winch, sv_rudder;

// servo absolute limits:
// rudder - 90 (center) +/-40
// winch    - 60 -> 115 (center)
void servoInit() {
    pinMode(SP_EN, OUTPUT);
    digitalWrite(SP_EN, LOW);
    
    pinMode(WINCH_EN, OUTPUT);
    digitalWrite(WINCH_EN, LOW);

    pinMode(RUDDER_EN, OUTPUT);
    digitalWrite(RUDDER_EN, LOW);

    sv_winch.attach(WINCH_PORT);
    sv_rudder.attach(RUDDER_PORT);  
}

void centerWinch() {
    winchTo(115);
}

void centerRudder() {
    rudderTo(90);
}

void winchTo(int value) {
    logln("Winch to %d", value);
  
    int v = constrain(value, WINCH_MIN, WINCH_MAX);

    digitalWrite(SP_EN, HIGH);
    delay(10);
    digitalWrite(WINCH_EN, HIGH);
    delay(10);
    sv_winch.write(v);
    delay(abs((current_winch - v))/WINCH_SPEED + 50);
    digitalWrite(WINCH_EN, LOW);
    digitalWrite(SP_EN, LOW);
    
    current_winch = v;
}

void rudderTo(int value) {
    logln("Rudder to %d", value);

    int v = constrain(value, RUDDER_MIN, RUDDER_MAX);
    
    digitalWrite(SP_EN, HIGH);
    delay(10);
    digitalWrite(RUDDER_EN, HIGH);
    delay(10);
    sv_rudder.write(v);
    delay(abs((current_rudder - v))/RUDDER_SPEED + 50);
    digitalWrite(RUDDER_EN, LOW);
    digitalWrite(SP_EN, LOW);
    
    current_rudder = v;
}

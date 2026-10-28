// Servo library: positions go to the simulation, which moves the servo only while its power is switched on.
#ifndef SIM_SERVO_H
#define SIM_SERVO_H

#include "Arduino.h"

class Servo {
public:
    uint8_t attach(int pin) { _pin = pin; sim_servo_attach(pin); return 0; }
    uint8_t attach(int pin, int, int) { return attach(pin); }
    void detach() { _pin = -1; }
    void write(int value) { if (_pin >= 0) sim_servo_write(_pin, value); }
    void writeMicroseconds(int us) { write((int)map(us, 544, 2400, 0, 180)); }
    int read() { return _pin >= 0 ? sim_servo_read(_pin) : 0; }
    bool attached() { return _pin >= 0; }

private:
    int _pin = -1;
};

#endif

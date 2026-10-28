// The only I2C device is the MPU-9250, which is simulated at the i2cdevlib level (MPU6050.h).
#ifndef SIM_WIRE_H
#define SIM_WIRE_H

#include "Arduino.h"

class TwoWire {
public:
    void begin() {}
    void setClock(uint32_t) {}
};

extern TwoWire Wire;

#endif

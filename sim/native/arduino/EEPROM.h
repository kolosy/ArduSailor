// EEPROM library over the simulation's 4KB EEPROM, which the runner saves to a file between runs.
#ifndef SIM_EEPROM_H
#define SIM_EEPROM_H

#include "Arduino.h"

struct EEPROMClass {
    uint8_t read(int idx) { return sim_eeprom_read(idx); }
    void write(int idx, uint8_t val) { sim_eeprom_write(idx, val); }
    void update(int idx, uint8_t val) { if (read(idx) != val) write(idx, val); }
    uint16_t length() { return 4096; }

    template <typename T> T &get(int idx, T &t) {
        uint8_t *p = (uint8_t *)&t;
        for (size_t i = 0; i < sizeof(T); i++) p[i] = read(idx + (int)i);
        return t;
    }
    template <typename T> const T &put(int idx, const T &t) {
        const uint8_t *p = (const uint8_t *)&t;
        for (size_t i = 0; i < sizeof(T); i++) update(idx + (int)i, p[i]);
        return t;
    }
};

extern EEPROMClass EEPROM;

#endif

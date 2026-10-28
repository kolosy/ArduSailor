// Just enough of the Arduino AVR core for the ArduSailor sketch to build and run natively. Time, pins,
// servos, UARTs and the EEPROM all go to the simulation engine through sim_api.h.
#ifndef SIM_ARDUINO_H
#define SIM_ARDUINO_H

// standard headers first: the Arduino macros below (min, max, abs, round, ...) break them if they come later
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#ifdef __cplusplus
#include <cmath>
#include <cstdlib>
#include <cstring>
#endif

#include "sim_api.h"

typedef bool boolean;
typedef uint8_t byte;
typedef unsigned int word;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#ifdef abs
#undef abs
#endif
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

// flash strings are ordinary strings here
class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);

long random(long howbig);
long random(long howsmall, long howbig);
void randomSeed(unsigned long seed);
long map(long x, long in_min, long in_max, long out_min, long out_max);

class Print {
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t c) = 0;
    size_t write(const char *str) { return str ? write((const uint8_t *)str, strlen(str)) : 0; }
    size_t write(const uint8_t *buffer, size_t size);

    size_t print(const __FlashStringHelper *s) { return write((const char *)s); }
    size_t print(const char s[]) { return write(s); }
    size_t print(char c) { return write((uint8_t)c); }
    size_t print(unsigned char n, int base = DEC) { return print((unsigned long)n, base); }
    size_t print(int n, int base = DEC) { return print((long)n, base); }
    size_t print(unsigned int n, int base = DEC) { return print((unsigned long)n, base); }
    size_t print(long n, int base = DEC);
    size_t print(unsigned long n, int base = DEC);
    size_t print(double n, int digits = 2);

    size_t println(void) { return write("\r\n"); }
    template <typename T> size_t println(T value) { size_t n = print(value); return n + println(); }
    template <typename T> size_t println(T value, int format) { size_t n = print(value, format); return n + println(); }
};

class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    void setTimeout(unsigned long timeout) { _timeout = timeout; }
    long parseInt();
    float parseFloat();

protected:
    unsigned long _timeout = 1000;
    int timedPeek();
};

class HardwareSerial : public Stream {
public:
    explicit HardwareSerial(int port) : _port(port) {}
    void begin(unsigned long baud) { sim_serial_begin(_port, (long)baud); }
    void end() {}
    int available() override { return sim_serial_available(_port); }
    int read() override { return sim_serial_read(_port); }
    int peek() override { return sim_serial_peek(_port); }
    void flush() {}
    size_t write(uint8_t c) override { sim_serial_write(_port, c); return 1; }
    using Print::write;
    operator bool() { return true; }

private:
    int _port;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;

#endif

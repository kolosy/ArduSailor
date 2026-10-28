#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

HardwareSerial Serial(0);
HardwareSerial Serial1(1);
HardwareSerial Serial2(2);
HardwareSerial Serial3(3);
EEPROMClass EEPROM;
TwoWire Wire;

// Every call costs a little CPU time, so busy-wait loops on millis() (the menu has one) still finish.
#define CALL_COST_US 2

unsigned long millis(void) {
    sim_advance_us(CALL_COST_US);
    return (unsigned long)(sim_now_us() / 1000);
}

unsigned long micros(void) {
    sim_advance_us(CALL_COST_US);
    return (unsigned long)sim_now_us();
}

void delay(unsigned long ms) { sim_advance_us((uint64_t)ms * 1000); }
void delayMicroseconds(unsigned int us) { sim_advance_us(us); }

void pinMode(uint8_t pin, uint8_t mode) { sim_pin_mode(pin, mode); }
void digitalWrite(uint8_t pin, uint8_t val) { sim_pin_write(pin, val); }
int digitalRead(uint8_t pin) { return sim_pin_read(pin); }

int analogRead(uint8_t pin) {
    // ADC conversion takes about 112us at the default prescaler
    sim_advance_us(112);
    return sim_analog_read(pin >= 54 ? pin - 54 : pin);
}

void analogReference(uint8_t) {}

long random(long howbig) { return howbig == 0 ? 0 : rand() % howbig; }
long random(long howsmall, long howbig) { return howsmall >= howbig ? howsmall : howsmall + random(howbig - howsmall); }
void randomSeed(unsigned long seed) { srand((unsigned)seed); }

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---- Print, following the Arduino core's formatting

size_t Print::write(const uint8_t *buffer, size_t size) {
    size_t n = 0;
    while (size--) n += write(*buffer++);
    return n;
}

static size_t printNumber(Print &p, unsigned long n, uint8_t base) {
    char buf[8 * sizeof(long) + 1];
    char *str = &buf[sizeof(buf) - 1];
    *str = '\0';
    if (base < 2) base = 10;
    do {
        char c = (char)(n % base);
        n /= base;
        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while (n);
    return p.write(str);
}

size_t Print::print(long n, int base) {
    if (base == 0) return write((uint8_t)n);
    if (base == 10 && n < 0) {
        size_t t = print('-');
        return t + printNumber(*this, (unsigned long)(-n), 10);
    }
    return printNumber(*this, (unsigned long)n, (uint8_t)base);
}

size_t Print::print(unsigned long n, int base) {
    if (base == 0) return write((uint8_t)n);
    return printNumber(*this, n, (uint8_t)base);
}

size_t Print::print(double number, int digits) {
    if (std::isnan(number)) return print("nan");
    if (std::isinf(number)) return print("inf");
    if (number > 4294967040.0 || number < -4294967040.0) return print("ovf");

    size_t n = 0;
    if (number < 0.0) {
        n += print('-');
        number = -number;
    }
    double rounding = 0.5;
    for (int i = 0; i < digits; ++i) rounding /= 10.0;
    number += rounding;

    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += print(int_part);
    if (digits > 0) n += print('.');
    while (digits-- > 0) {
        remainder *= 10.0;
        unsigned int to_print = (unsigned int)remainder;
        n += print(to_print);
        remainder -= to_print;
    }
    return n;
}

// ---- Stream parsing, as in the Arduino core (skip to the first digit, give up after the timeout)

int Stream::timedPeek() {
    unsigned long start = millis();
    do {
        int c = peek();
        if (c >= 0) return c;
        delay(1);
    } while (millis() - start < _timeout);
    return -1;
}

long Stream::parseInt() {
    int c;
    while (true) {
        c = timedPeek();
        if (c < 0) return 0;
        if (c == '-' || (c >= '0' && c <= '9')) break;
        read();
    }
    bool negative = false;
    long value = 0;
    do {
        if (c == '-')
            negative = true;
        else
            value = value * 10 + c - '0';
        read();
        c = timedPeek();
    } while (c >= '0' && c <= '9');
    return negative ? -value : value;
}

float Stream::parseFloat() {
    int c;
    while (true) {
        c = timedPeek();
        if (c < 0) return 0;
        if (c == '-' || c == '.' || (c >= '0' && c <= '9')) break;
        read();
    }
    bool negative = false, fraction = false;
    double value = 0, scale = 1;
    do {
        if (c == '-')
            negative = true;
        else if (c == '.')
            fraction = true;
        else {
            value = value * 10 + c - '0';
            if (fraction) scale *= 10;
        }
        read();
        c = timedPeek();
    } while ((c >= '0' && c <= '9') || (c == '.' && !fraction));
    value /= scale;
    return (float)(negative ? -value : value);
}

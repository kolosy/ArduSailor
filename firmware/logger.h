#ifndef __logger_h
#define __logger_h

#include "Arduino.h"

#define NO_SD
#define MAX_STRING 128

extern char gps_time[7];        // HHMMSS
extern uint32_t gps_seconds;    // seconds after midnight
extern boolean serial_logging;

void log(char *fmt, ... );
void logln(char *fmt, ... );
void logln(const __FlashStringHelper *ifsh, ...);
void logInit();

#endif

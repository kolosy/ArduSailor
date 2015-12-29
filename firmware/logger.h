#ifndef __logger_h
#define __logger_h

#include "Arduino.h"
extern char gps_time[7];        // HHMMSS
extern uint32_t gps_seconds;    // seconds after midnight
extern boolean serial_logging;

void log(char *fmt, ... );
void logln(char *fmt, ... );
void logInit();

#endif

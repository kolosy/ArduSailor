#include "logger.h"
#include <stdarg.h>

#ifndef NO_SD

#include <SD.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

SdCard card;
Fat16 file;

#endif

char gps_time[7];       // HHMMSS
uint32_t gps_seconds = 0;   // seconds after midnight

uint8_t fileReady = 0;

void do_log(char *fmt, va_list args, bool println);

void logInit() {
#ifndef NO_SD
	// initialize the SD card
	if (!card.init()) { 
		Serial.print("Error initializing card - ");
		Serial.println(card.errorCode, HEX);
		return;
	}
    
	// initialize a FAT16 volume
	if (!Fat16::init(&card)) {
		Serial.println("Can't initialize volume.");
		return;
	}
    
	// create a new file
	char name[] = "LOGGER00.TXT";
	for (uint8_t i = 0; i < 100; i++) {
		name[6] = i/10 + '0';
		name[7] = i%10 + '0';
		// O_CREAT - create the file if it does not exist
		// O_EXCL - fail if the file exists
		// O_WRITE - open for write only
		if (file.open(name, O_CREAT | O_EXCL | O_WRITE)) break;
	}

	if (!file.isOpen()) {
		Serial.println("Error creating log file.");
		return;
	}
    
	Serial.print("Logging to: ");
	Serial.println(name);

	// write data header
    
	// clear write error
	file.writeError = false;
	file.println("Started log.");
	file.sync();
    
	fileReady = 1;
#else
	Serial.println("Started log.");
#endif
}

void logln(const __FlashStringHelper *ifsh, ...) {
    const char PROGMEM *p = (const char PROGMEM *)ifsh;
	char * fmt = (char*)malloc(MAX_STRING);
	
    size_t n = 0;
	uint16_t count = 0;
    while (1) {
      unsigned char c = pgm_read_byte(p++);
      fmt[n++] = c;

	  // we want the 0 byte in there
      if (c == 0) break;
	  
	  if (count++ >= MAX_STRING) {
		  Serial.println("Malformed PROGMEM string - no null terminator found.");
		  free(fmt);
		  
		  return;
	  }
    }
	
	va_list args;
	va_start (args, *ifsh);
	do_log(fmt, args, true);
	va_end (args);
	
	free(fmt);
}

void logln(char *fmt, ... ) {
	va_list args;
	va_start (args, fmt );
	do_log(fmt, args, true);
	va_end (args);
}

void log(char *fmt, ... ) {
	va_list args;
	va_start (args, fmt );
	do_log(fmt, args, false);
	va_end (args);
}

void do_log(char *fmt, va_list args, bool println) {
	char buf[144]; // resulting string limited to 128 chars
	vsnprintf(buf, 144, fmt, args);

	if (serial_logging) {
		Serial.print(gps_time);
		Serial.print(':');
		Serial.print(millis());
		Serial.print(' ');
		Serial.print(buf);
		if (println)
			Serial.println();
	}
    
#ifndef NO_SD
	if (fileReady) {
		file.print(gps_time);
		file.print(':');
		file.print(millis());
		file.print(' ');
		file.print(buf);
		if (println)
			file.println();
		
		file.sync();
	}
#endif
}

#include "logger.h"
#include <stdarg.h>

#include <SD.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

SdCard card;
Fat16 file;

char gps_time[7];		// HHMMSS
uint32_t gps_seconds = 0;	// seconds after midnight

uint8_t fileReady = 0;

void logInit() {
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
}

void logln(char *fmt, ... ) {
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);

    Serial.print(gps_time);
    Serial.print(':');
    Serial.print(millis());
    Serial.print(' ');
    Serial.println(buf);
    if (fileReady) {
        file.print(gps_time);
        file.print(':');
        file.print(millis());
        file.print(' ');
        file.println(buf);
        file.sync();
    }
}

void log(char *fmt, ... ) {
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);

    Serial.print(gps_time);
    Serial.print(':');
    Serial.print(millis());
    Serial.print(' ');
    Serial.print(buf);
    if (fileReady) {
        file.print(gps_time);
        file.print(':');
        file.print(millis());
        file.print(' ');
        file.print(buf);
        file.sync();
    }
}
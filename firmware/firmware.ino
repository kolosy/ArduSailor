
#include <Servo.h> 
#include "LowPower.h"
//#include <SD.h>

#include "Wire.h"
#include "I2Cdev.h"

#include "SoftwareSerial.h"

//#include <Fat16.h>
//#include <Fat16util.h> // use functions to print strings from flash memory

#define debug

#define GPS_BAUDRATE 9600
#define NAV_LOCK 9

#define SCAN_RATE_NORMAL 30000
#define SCAN_RATE_FAST 2000

//SdCard card;
//Fat16 file;

// Public (extern) variables, readable from other modules
char gps_time[7];		// HHMMSS
uint32_t gps_seconds = 0;	// seconds after midnight
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[9];
char gps_aprs_lon[10];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = 0;
uint16_t ahrs_heading = 0;
uint16_t wind = 0;

uint8_t current_rudder = 0;
uint8_t current_winch = 0;
boolean adjustment_made = false;

SoftwareSerial gpsSerial(7,4);

void sdInit() {
	/*
	// initialize the SD card
	if (!card.init()) { 
	Serial.print("Error initializing card - ");
	Serial.println(card.errorCode, HEX);
	}
	
	// initialize a FAT16 volume
	if (!Fat16::init(&card))
	Serial.println("Can't initialize volume.");
	
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
	if (!file.isOpen()) Serial.println("Error creating log file.");
	Serial.print("Logging to: ");
	Serial.println(name);

	// write data header
	
	// clear write error
	file.writeError = false;
	file.print("Started log.");
	file.sync();
	*/
}

void setup() 
{ 
	Wire.begin();

	Serial.begin(38400);

	Serial.println("ArduSailor Starting...");
	
    pinMode(7, INPUT);
    pinMode(4, OUTPUT);
	gpsSerial.begin(GPS_BAUDRATE);
	
	pinMode(NAV_LOCK, INPUT);

	servoInit();
	mpuInit();
	// sdInit();
	gpsInit();

	Serial.print("Waiting for GPS lock...");
	while (digitalRead(NAV_LOCK) == LOW);
	Serial.println("locked.");
	
	pilotInit();
} 

void updateSensors() {
	// most of this will be used in human comparison stuff, no need to keep in radians.
	ahrs_heading = readSteadyHeading() * 180.0 / PI;
	wind = readSteadyWind() * 180.0 / PI;
	
	gpsSerial.begin(GPS_BAUDRATE);

	do {
		while (! gpsSerial.available());
	} while (! gps_decode(gpsSerial.read()));
    
    gpsSerial.end();
}

void loop() 
{ 
	updateSensors();
	
	doPilot();
		
	sleepMillis(adjustment_made ? SCAN_RATE_FAST : SCAN_RATE_NORMAL);
} 

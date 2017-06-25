#include "logger.h"
#include <Servo.h> 
#include "servo_ctl.h"

#include "LowPower.h"
#include "ahrs.h"

#ifndef NO_SD
#include <SD.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory
#endif

#include "Wire.h"
#include "I2Cdev.h"

#include "SoftwareSerial.h"
#include "MPU6050.h"

#define GPS_BAUDRATE 9600
#define STATUS_LED 32

#define SCAN_RATE_NORMAL 30000
#define SCAN_RATE_FAST 500

#define GPS_WARNING 15000
#define HIGH_RES_GPS_DEFAULT true

#define WAIT_FOR_COMMAND_EVERY 10
#define WAIT_FOR_COMMAND_FOR 1000

#define DATA_FREQ 1500

#define SERIAL_LOGGING_DEFAULT false

#define RC_DATA_FREQ 500

// seconds
#define MENU_TIMEOUT 10
// #define SHOW_MENU_ON_START

#define RAD(v) ((v) * PI / 180.0)
#define DEG(v) ((v) * 180.0 / PI)

// lets you do a logln of a float like so -> logln("blah %d.%d", FP(d))
#define FP(f) (int16_t)f, fracPart(f)
#define FP32(f) (int32_t)f, fracPart(f)

// glancing ahead at the util sketch. todo: make that a c file
int fracPart(float f, int precision = 2);

// Public (extern) variables, readable from other modules

// last time we got a gps update
uint32_t last_gps_time = 0;

// self-explanatory
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[9];
char gps_aprs_lon[10];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = 0;
float ahrs_heading = 0;

// winch adjustment to spill extra air if we're heeling too much
float heel_adjust = 0;

// rudder adjustment to compensate for excessive heel. todo: clarify naming; declared in servo_ctl.h
// float heel_offset = 0;

// wind direction
uint16_t wind = 0;
float trailing_wind = 0;

// current loop() count
uint32_t cycle = 0;

// whether the pilot made any changes
boolean adjustment_made = false;

// whether we need high-res gps (drives refresh frequency)
boolean high_res_gps = HIGH_RES_GPS_DEFAULT;

uint32_t last_data_update = 0;
float voltage = 0;

// here so we can log easier
double wp_heading = 0;
float wp_distance = 0;

// what the PID will steer to
double requested_heading = 0;

// mode variables
boolean manual_override = false;

// whather calls to logln should also log to Serial
boolean serial_logging = SERIAL_LOGGING_DEFAULT;
boolean remote_control = false;
boolean calibration = false;

#define AHRS_TRAIL 10
#define WIND_TRAIL 10

float ahrs_trail[AHRS_TRAIL];
float wind_trail[WIND_TRAIL];

void setup() 
{ 
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, HIGH);
    
	Wire.begin();

	Serial.begin(9600);
	logInit();

	logln(F("ArduSailor Starting..."));
	Serial2.begin(GPS_BAUDRATE);

	// config value
	mag_offset = RAD(-15.0);

	logln(F("Initializing servos..."));
	servoInit();
	logln(F("Initializing wind sensor..."));
	windInit();
	logln(F("Initializing MPU..."));
	mpuInit();
	logln(F("Enabling GPS..."));
	gpsInit();
	logln(F("Starting battery monitor..."));
	batteryInit();
	logln(F("Allocating log trail..."));
	initTrail();
    
	logln(F("Starting pilot..."));
	pilotInit();
    
	blink(STATUS_LED, 100, 10, HIGH);
	logln(F("Enabling GPS..."));
    
	warnGPS();

	logln(F("Done. System ready."));
    
	digitalWrite(STATUS_LED, LOW);
	
#ifdef SHOW_MENU_ON_START
	doMenu();
#endif
} 

void initTrail() {
	ahrs_heading = readSteadyHeading() * 180.0 / PI;
	wind = readSteadyWind() * 180.0 / PI;
	
	for (int i=0; i<AHRS_TRAIL; i++)
		ahrs_trail[i] = ahrs_heading;
	
	for (int i=0; i<WIND_TRAIL; i++)
		wind_trail[i] = wind;
}

void newTrailingValue(float new_val, int count, float *target_val, float *trail) {
	trail[cycle % count] = new_val;
	
	float v = 0;
	for (int i=0; i<count; i++)
		v += trail[i];

	(*target_val) = v / (float)count;
}

void updateSensors(boolean skip_gps) {
	// most of this will be used in human comparison stuff, no need to keep in radians.
	newTrailingValue(readSteadyHeading() * 180.0 / PI, AHRS_TRAIL, &ahrs_heading, ahrs_trail);
	newTrailingValue(readSteadyWind() * 180.0 / PI, WIND_TRAIL, &trailing_wind, wind_trail);
	
	wind = round(trailing_wind);
    
#ifdef SLEEP_GPS
	if (!skip_gps && (high_res_gps || (last_gps_time == 0) || ((millis() - last_gps_time) > GPS_REFRESH))) {
		updateGPS();

		last_gps_time = millis();
		gps_updated = true;
	} else
		gps_updated = false;
#endif
        
	if (!high_res_gps && (millis() - last_gps_time > GPS_WARNING))
		warnGPS();

	uint16_t gps_elapsed = millis() - last_gps_time;
	voltage = measureVoltage();
	logln(F("Position: %s, %s (%dms old), Speed: %d.%d, Direction: %d.%d, Wind: %d, Battery %d.%d"), 
	gps_aprs_lat, 
	gps_aprs_lon, 
	gps_elapsed,
	FP(gps_speed),
	FP(gps_course),
	wind, 
	FP(voltage));
}

void getMagOffset() {
	bool current_sl = serial_logging;
	serial_logging = true;
	
	logln(F("Current mag offset is %d.%d"), FP(DEG(mag_offset)));

	Serial.print(F("New offset: "));

	if (!waitForData(5000))
		return;
	
	float mag_offset_d = Serial.parseFloat();
	Serial.println(mag_offset_d, 4);
	mag_offset = RAD(mag_offset_d);

	Serial.println("Stored.");
	
	serial_logging = current_sl;
}

void printDataLine() {
	Serial.print(gps_aprs_lat); Serial.print(", ");
	Serial.print(gps_aprs_lon); Serial.print(", "); 
	Serial.print(gps_lat, 6); Serial.print(", ");
	Serial.print(gps_lon, 6); Serial.print(", "); 
	Serial.print(millis() - last_gps_time); Serial.print(", ");
	Serial.print(gps_speed); Serial.print(", ");
	Serial.print(gps_course); Serial.print(", ");

	Serial.print(ahrs_heading); Serial.print(", ");
	Serial.print(requested_heading); Serial.print(", ");
	Serial.print(current_roll); Serial.print(", ");
	Serial.print(heel_adjust); Serial.print(", ");
	Serial.print(wind); Serial.print(", ");
	Serial.print(wp_heading); Serial.print(", ");
	Serial.print(wp_distance); Serial.print(", ");
	Serial.print(current_rudder); Serial.print(", ");
	Serial.print(current_winch); Serial.print(", ");
	Serial.print(voltage); Serial.print(", ");
	Serial.print(cycle); Serial.println();
}

void doMenu() {
	Serial.print(F("Welcome to ArduSailor. Menu timeout is "));
	Serial.println(MENU_TIMEOUT); 
	Serial.println();

	Serial.println("Current configuration is:");
	Serial.print(F("calibration: ")); Serial.println(calibration);
	Serial.print(F("remote control: ")); Serial.println(remote_control);
	Serial.print(F("manual override: ")); Serial.println(manual_override);
	
	Serial.println(F("\nPlease select a menu option. \n"));
	
	Serial.println(F("(a) Automate."));
	Serial.println(F("(c) Calibrate."));
	Serial.println(F("(r) Remote Control."));
	Serial.println(F("(w) Change waypoints <NOT IMPLEMENTED>."));
	Serial.println(F("(t) Tune PID."));
	Serial.println(F("(m) Set mag offset."));
	Serial.print(F("\n>"));
	
	long t = millis();
	
	while ((millis() - t < (MENU_TIMEOUT * 1000)) && !Serial.available());
	
	if (Serial.available()) {
		char c = (char) Serial.read();
		switch (c) {
			case 'a':
			calibration = false;
			remote_control = false;
			manual_override = false;
			break;

			case 'c':
			calibration = true;
			remote_control = false;
			manual_override = false;
			break;

			case 'r':
			calibration = false;
			remote_control = true;
			manual_override = false;
			break;

			case 'w':
			// todo
			break;
			
			case 't':
			getPIDTunings();
			break;
			
			case 'm':
			getMagOffset();
			break;
		}
		
		Serial.print(' ');
		Serial.println(c);
	} else
		Serial.println(F("\nMenu timed out."));

	Serial.print(F("calibration: ")); Serial.println(calibration);
	Serial.print(F("remote control: ")); Serial.println(remote_control);
	Serial.print(F("manual override: ")); Serial.println(manual_override);
}

void checkInput() {
	// in case we're going too fast. only needed when serial_logging is on
	if (serial_logging && (cycle % WAIT_FOR_COMMAND_EVERY == 0))
		delay(WAIT_FOR_COMMAND_FOR);
  
	while (Serial.available()) {
		// a '[' signals the start of an RC command
		if (remote_control && (char)Serial.peek() == '[')
			return;
		
		switch ((char)Serial.read()) {
			case 'o':
			logln(F("Entering manual override"));
			manual_override = true;
			serial_logging = true;
			break;
            
			case 'l':
			serial_logging = !serial_logging;
			break;
			
			case 'm':
			doMenu();
			break;
		}
	}
}

void loop() 
{ 
	if (calibration) {
		writeCalibrationLine();
		checkInput();
		
		return;
	}

	if (!manual_override) {
		logln(F("[Cycle %d start]"), cycle);
		cycle++;
		updateSensors(false);

		checkInput();
		doPilot();
	} else
		// the actual steering commands are handled by doPilot for now
		doPilot();
        
	if (!serial_logging && (millis() - last_data_update > (remote_control ? RC_DATA_FREQ : DATA_FREQ))) {
		printDataLine();
      
		last_data_update = millis();
	}
} 

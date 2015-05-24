#include "logger.h"
#include <Servo.h> 
#include "LowPower.h"
#include "ahrs.h"

#include <SD.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

#include "Wire.h"
#include "I2Cdev.h"

#include "SoftwareSerial.h"
#include "MPU6050.h"
#include "HMC58X3.h"

#define GPS_BAUDRATE 9600
#define STATUS_LED 32

#define SCAN_RATE_NORMAL 30000
#define SCAN_RATE_FAST 1000

#define GPS_REFRESH 10000
#define GPS_WARNING 7000

// Public (extern) variables, readable from other modules
uint16_t last_gps_time = 0;
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[9];
char gps_aprs_lon[10];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = 0;
float ahrs_heading = 0;
uint16_t wind = 0;

uint8_t current_rudder = 0;
uint8_t current_winch = 0;
boolean adjustment_made = false;
boolean high_res_gps = true;
boolean gps_updated = false;
boolean manual_override = false;

void setup() 
{ 
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);
    
    Wire.begin();

    Serial.begin(9600);
    logInit();

    logln("ArduSailor Starting...");
    Serial2.begin(GPS_BAUDRATE);

#ifdef CALIBRATE_ONLY
    mpuInit();
#else
    servoInit();

    windInit();
    mpuInit();
    gpsInit();
    batteryInit();
    
    pilotInit();
    
    blink(STATUS_LED, 100, 10, HIGH);
    logln("Acquiring GPS...");
    
    updateGPS();
    last_gps_time = millis();
    gps_updated = true;
#endif

    logln("Done. System ready.");
    
    digitalWrite(STATUS_LED, LOW);
} 

void updateSensors() {
    // most of this will be used in human comparison stuff, no need to keep in radians.
    ahrs_heading = readSteadyHeading() * 180.0 / PI;
    wind = readSteadyWind() * 180.0 / PI;
    
    if (high_res_gps || (last_gps_time == 0) || ((millis() - last_gps_time) > GPS_REFRESH)) {
        updateGPS();
            
        last_gps_time = millis();
        gps_updated = true;
    } else
        gps_updated = false;
        
    if (!high_res_gps && (millis() - last_gps_time > GPS_WARNING))
        warnGPS();

    uint16_t gps_elapsed = millis() - last_gps_time;
    int16_t voltage = ((int16_t)(measureVoltage() * 100.0));
    logln("Position: %s, %s (%dms old), Speed (x10): %d, Direction: %d, Wind: %d, Battery %d", 
                gps_aprs_lat, 
                gps_aprs_lon, 
                gps_elapsed,
                (int16_t) (gps_speed * 10),
                (int16_t)gps_course,
                wind, 
                voltage);
}

void checkInput() {
    while (Serial.available())
        if (Serial.read() == 'o') {
            logln("Entering manual override");
            manual_override = true;
            break;
        }
}

void loop() 
{ 
#ifdef CALIBRATE_ONLY
    writeCalibrationLine();
    digitalWrite(STATUS_LED, HIGH);
    delay(10);
    digitalWrite(STATUS_LED, LOW);
    delay(10);
#else
    if (!manual_override) {
        updateSensors();
        checkInput();
        doPilot();

        sleepMillis(adjustment_made ? SCAN_RATE_FAST : SCAN_RATE_NORMAL);
    } else
        doPilot();
#endif
} 

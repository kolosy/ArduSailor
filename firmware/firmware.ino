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
#define SCAN_RATE_FAST 500

#define GPS_WARNING 15000
#define HIGH_RES_GPS_DEFAULT true

#define WAIT_FOR_COMMAND_EVERY 10
#define WAIT_FOR_COMMAND_FOR 1000

#define DATA_FREQ 1500
#define SERIAL_LOGGING_DEFAULT false;

#define WINCH_MAX 60
#define WINCH_MIN 130

// Public (extern) variables, readable from other modules
uint32_t last_gps_time = 0;
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[9];
char gps_aprs_lon[10];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = 0;
float ahrs_heading = 0;
float heel_adjust = 0;

uint16_t wind = 0;

int16_t heel_offset = 0;
uint32_t cycle = 0;
uint8_t current_rudder = 0;
uint8_t current_winch = 0;
boolean adjustment_made = false;
boolean high_res_gps = HIGH_RES_GPS_DEFAULT;
//boolean gps_updated = false;
boolean manual_override = false;
boolean serial_logging = true;
uint32_t last_data_update = 0;
float voltage = 0;

// here so we can log easier
float wp_heading = 0;
float wp_distance = 0;

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
    logln("Enabling GPS...");
    
//    updateGPS();
    warnGPS();
//    last_gps_time = millis();
//    gps_updated = true;
#endif

    logln("Done. System ready.");
    
    digitalWrite(STATUS_LED, LOW);
    serial_logging = SERIAL_LOGGING_DEFAULT;
} 

void updateSensors(boolean skip_gps) {
    // most of this will be used in human comparison stuff, no need to keep in radians.
    ahrs_heading = readSteadyHeading() * 180.0 / PI;
    wind = readSteadyWind() * 180.0 / PI;
    
//    if (!skip_gps && (high_res_gps || (last_gps_time == 0) || ((millis() - last_gps_time) > GPS_REFRESH))) {
//        updateGPS();
//            
//        last_gps_time = millis();
//        gps_updated = true;
//    } else
//        gps_updated = false;
        
    if (!high_res_gps && (millis() - last_gps_time > GPS_WARNING))
        warnGPS();

    uint16_t gps_elapsed = millis() - last_gps_time;
    voltage = measureVoltage();
    logln("Position: %s, %s (%dms old), Speed (x10): %d, Direction: %d, Wind: %d, Battery %d", 
                gps_aprs_lat, 
                gps_aprs_lon, 
                gps_elapsed,
                (int16_t) (gps_speed * 10.0),
                (int16_t)gps_course,
                wind, 
                voltage);
}

void checkInput() {
    // in case we're going too fast. only needed when serial_logging is on
    if (serial_logging && (cycle % WAIT_FOR_COMMAND_EVERY == 0))
      delay(WAIT_FOR_COMMAND_FOR);
  
    while (Serial.available()) {
        switch ((char)Serial.read()) {
          case 'o':
            logln("Entering manual override");
            manual_override = true;
            serial_logging = true;
            break;
            
          case 'l':
            serial_logging = !serial_logging;
            break;
        }
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
        logln("#################### Cycle %d start ####################", cycle);
        cycle++;
        updateSensors(false);
        checkInput();
        doPilot();

// no sleep for the wicked.
//        sleepMillis(SCAN_RATE_FAST);
    } else
        doPilot();
        
    if (!serial_logging && (millis() - last_data_update > DATA_FREQ)) {
      Serial.print(gps_aprs_lat); Serial.print(", ");
      Serial.print(gps_aprs_lon); Serial.print(", "); 
      Serial.print(gps_lat, 6); Serial.print(", ");
      Serial.print(gps_lon, 6); Serial.print(", "); 
      Serial.print(millis() - last_gps_time); Serial.print(", ");
      Serial.print(gps_speed); Serial.print(", ");
      Serial.print(gps_course); Serial.print(", ");

      Serial.print(ahrs_heading); Serial.print(", ");
      Serial.print(current_roll); Serial.print(", ");
      Serial.print(heel_adjust); Serial.print(", ");
      Serial.print(wind); Serial.print(", ");
      Serial.print(wp_heading); Serial.print(", ");
      Serial.print(wp_distance); Serial.print(", ");
      Serial.print(current_rudder); Serial.print(", ");
      Serial.print(current_winch); Serial.print(", ");
      Serial.print(voltage); Serial.print(", ");
      Serial.print(cycle); Serial.println();
      
      last_data_update = millis();
    }
#endif
} 

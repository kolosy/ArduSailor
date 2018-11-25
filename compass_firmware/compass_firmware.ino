#include <Wire.h>
#include <LSM303.h>
#include <EEPROM.h>

LSM303 compass;
LSM303::vector<int> orientation = (LSM303::vector<int>){0, 1, 0};
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

int16_t calibrationSteps = 0;
#define TOTAL_CALIBRATION_STEPS 300

char report[140];

boolean calibrating = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  compass.init();
  compass.enableDefault();
  
  EEPROM.get(0, running_min);
  EEPROM.get(sizeof(LSM303::vector<int16_t>), running_max);

  compass.m_min = running_min;
  compass.m_max = running_max;

  printStatus();
}

void loop() {
  compass.read();

  if (!Serial.available())
    return;

  char command = Serial.read();

  switch (command) {
    case 'i':
      printStatus();
      break;
    case 's':
        
        Serial.println("ACK");

        break;
    case 'c':
        calibrate();
        break;
    case 'r': {
        float heading = compass.heading(orientation);
        
        Serial.println(heading);
        break;
    }
  }
}

void printStatus() {
  Serial.println("Current settings:");
    
  snprintf(report, sizeof(report), "Starting values min: {%+6d, %+6d, %+6d}\tmax: {%+6d, %+6d, %+6d}.\tOrientation {%+1d, %+1d, %+1d}",
        compass.m_min.x, compass.m_min.y, compass.m_min.z,
        compass.m_max.x, compass.m_max.y, compass.m_max.z,
        orientation.x, orientation.y, orientation.z);
        
  Serial.println(report);
}

void calibrationLoop() {
  compass.read();
  
  running_min.x = min(running_min.x, compass.m.x);
  running_min.y = min(running_min.y, compass.m.y);
  running_min.z = min(running_min.z, compass.m.z);

  running_max.x = max(running_max.x, compass.m.x);
  running_max.y = max(running_max.y, compass.m.y);
  running_max.z = max(running_max.z, compass.m.z);

  delay(100);

  calibrationSteps++;
}

void calibrate() {
  Serial.println("ACK\r\nBeginning Calibration");
    
  snprintf(report, sizeof(report), "Starting values min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
        compass.m_min.x, compass.m_min.y, compass.m_min.z,
        compass.m_max.x, compass.m_max.y, compass.m_max.z);
        
  Serial.println(report);

  while (calibrationSteps < TOTAL_CALIBRATION_STEPS)
    calibrationLoop();

  Serial.println("Calibration complete");
    
  compass.m_min = running_min;
  compass.m_max = running_max;

  snprintf(report, sizeof(report), "New values min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
        compass.m_min.x, compass.m_min.y, compass.m_min.z,
        compass.m_max.x, compass.m_max.y, compass.m_max.z);

  Serial.print("Writing to eeprom...");

  EEPROM.put(0, running_min);
  EEPROM.put(sizeof(LSM303::vector<int16_t>), running_max);
        
  Serial.println(report);
}


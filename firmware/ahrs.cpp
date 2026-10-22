#include "config.h"

#ifndef PILOT_DEBUG
#include "ahrs.h"
#include "logger.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_9Axis_MotionApps41.h"
#include <EEPROM.h>

MPU6050 mpu;

#define MPU_LOOPS 2
#define MPU_PAUSES 1
#define DEVICE_ORIENTATION 1.0
#define TOTAL_CALIBRATION_STEPS 300
#define CALIBRATION_WAIT 5

// stored after the calibration in eeprom. calibrations from older firmware were 14-bit (and could have a
// bad centre), so they don't carry this marker and are ignored.
#define MAG_CAL_VERSION 'm'

// the MPU-9250's AK8963 magnetometer sits at 0x0C on the auxiliary bus. i2cdevlib's DMP 4.1 setup was
// written for an AK8975 at 0x0E, so point the MPU's I2C master at it ourselves.
#define AK8963_ADDRESS 0x0C

// CNTL1 value written each cycle to start the next reading: single measurement (MODE 0001) with 16-bit
// output (BIT 1), 0.15uT/LSB. i2cdevlib's 0x01 gives 14-bit at 0.6uT/LSB, only ~20 counts of horizontal
// field in Chicago.
#define AK8963_SINGLE_16BIT 0x11

// read the magnetometer every (1 + MAG_SAMPLE_DELAY) samples: 200Hz / 5 = 40Hz. i2cdevlib's 24 gives 8Hz,
// so on a rolling boat the reading could be 125ms older than the gravity vector it's tilt-compensated with.
#define MAG_SAMPLE_DELAY 4

int16_t calibrationSteps = 0;
// mag parameters; todo: should be moved to flash
//static const float b_field     = 61.2088;
//static const float offset[3]   = { 2.0848, -33.8208, 28.5815 };
////static const float b_inv[3][3] = {{ 1, 0, 0 },
////                                  { 0, 1, 0 },
////                                  { 0, 0, 1 }};
//static const float b_inv[3][3] = {{ 0.9745, 0.0240, 0.0054 },
//                                  { 0.0240, 1.0463, -0.0089 },
//                                  { 0.0054, -0.0089, 0.9814 }};

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float current_pitch = 0;
float current_roll = 0;
float mag_offset = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t mag[3];
int16_t acc[3];

struct vector
{
  float x, y, z;
};

struct vector_i16t {
  int16_t x, y, z;
};

float heading(vector a, vector m);
void vector_cross(const vector *a, const vector *b, vector *out);
float vector_dot(const vector *a, const vector *b);
void vector_normalize(vector *a);
int readFIFOPacket();

vector_i16t m_min, m_max, running_min, running_max;

// Sets the relative orientation of the MPU
vector from = {1, 0, 0};

int16_t _settingsAddress = 0;

int mpuInit(int16_t settingsAddress) {
    _settingsAddress = settingsAddress;
    
    logln(F("Reading calibration data at location %d"), _settingsAddress);

    if (EEPROM.read(_settingsAddress + 2 * sizeof(vector_i16t)) == MAG_CAL_VERSION) {
        EEPROM.get(_settingsAddress, m_min);
        EEPROM.get(_settingsAddress + sizeof(vector_i16t), m_max);
    } else
        logln(F("No compass calibration for this firmware. Calibrate from the menu (c or o)."));
//    m_min.x = -70;
//    m_min.y = -26;
//    m_min.z = -97;
//    m_max.x = +60;
//    m_max.y = +87;
//    m_max.z = +43;

    logln(F("Starting values min: {%+6d, %+6d, %+6d}\tmax: {%+6d, %+6d, %+6d}"),
        m_min.x, m_min.y, m_min.z,
        m_max.x, m_max.y, m_max.z);

	// initialize device
	logln(F("Initializing I2C devices..."));

	mpu.initialize();

	// verify connection
	logln(F("Testing device connections..."));
	mpu.testConnection() ? logln(F("MPU6050 connection successful")) : logln(F("MPU6050 connection failed"));

	// load and configure the DMP
	logln(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// slave 0 reads the AK8963's ST1..ST2, slave 2 writes CNTL1 to start the next measurement
		mpu.setSlaveAddress(0, 0x80 | AK8963_ADDRESS);
		mpu.setSlaveAddress(2, AK8963_ADDRESS);
		mpu.setSlaveOutputByte(2, AK8963_SINGLE_16BIT);
		mpu.setSlave4MasterDelay(MAG_SAMPLE_DELAY);

		// turn on the DMP, now that it's ready
		logln(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//        attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		logln(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();

		return 0;
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		logln(F("DMP Initialization failed (code %d)"), devStatus);

		return devStatus;
	}
}

void calibrationLoop() {
    if (!readFIFOPacket())
        return;

    mpu.dmpGetMag(mag, fifoBuffer);
    
    running_min.x = min(running_min.x, mag[0]);
    running_min.y = min(running_min.y, mag[1]);
    running_min.z = min(running_min.z, mag[2]);
    
    running_max.x = max(running_max.x, mag[0]);
    running_max.y = max(running_max.y, mag[1]);
    running_max.z = max(running_max.z, mag[2]);
    
    delay(100);
    
    calibrationSteps++;
}

void calibrateMag(bool waitForSetup) {
  if (waitForSetup) {
      logln(F("Beginning Calibration in %d seconds"), CALIBRATION_WAIT);
      delay(CALIBRATION_WAIT * 1000);
  }
    
  logln(F("Calibration start. Initial values min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}"),
        m_min.x, m_min.y, m_min.z,
        m_max.x, m_max.y, m_max.z);

  // start from the extremes, not 0: an axis whose offset is bigger than the field never reads 0
  calibrationSteps = 0;
  running_min = {INT16_MAX, INT16_MAX, INT16_MAX};
  running_max = {INT16_MIN, INT16_MIN, INT16_MIN};

  while (calibrationSteps < TOTAL_CALIBRATION_STEPS)
    calibrationLoop();

  logln(F("Calibration complete"));

  m_min = running_min;
  m_max = running_max;

  // a flat circle can't measure the vertical offset: Z only ever reads (vertical field + offset). Work it
  // out from the horizontal field (the X/Y radius) and the local inclination instead. The magnetometer's
  // Z points down through the board, so with the board upright the vertical field reads positive. A 3-D
  // calibration (turning through every orientation, including upside down) measures Z directly.
  float horizontal = ((m_max.x - m_min.x) + (m_max.y - m_min.y)) / 4.0;
  if (m_max.z - m_min.z < horizontal) {
      m_min.z = m_max.z = round((m_min.z + m_max.z) / 2.0 - horizontal * tan(MAG_INCLINATION * PI / 180.0));
      logln(F("Flat calibration; vertical offset estimated from MAG_INCLINATION"));
  }

  logln(F("New values min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}"),
        m_min.x, m_min.y, m_min.z,
        m_max.x, m_max.y, m_max.z);

  logln(F("Writing to eeprom at location %d"), _settingsAddress);

  EEPROM.put(_settingsAddress, m_min);
  EEPROM.put(_settingsAddress + sizeof(vector_i16t), m_max);
  EEPROM.write(_settingsAddress + 2 * sizeof(vector_i16t), MAG_CAL_VERSION);
}

void normalize_mpu(int16_t mag_val[3], float normalized[3]) {
    // normalize *and* re-align axes (because, you know, having all sensors be the same is too much to ask for).
    normalized[1] =   mag_val[0] - (m_min.x + m_max.x) / 2.0;
    normalized[0] =   mag_val[1] - (m_min.y + m_max.y) / 2.0;
    normalized[2] = -(mag_val[2] - (m_min.z + m_max.z) / 2.0);
}

int readFIFOPacket() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return 0;

    // the FIFO keeps filling while the rest of the loop runs, so whatever is queued is stale, and once it
    // has overflowed (512 bytes on the MPU-9250, not a whole number of packets, oldest bytes dropped)
    // it's misaligned too. Empty it and take the next packet the DMP writes.
    mpu.resetFIFO();
    mpu.getIntStatus();

    while (true) {
        // reading INT_STATUS clears it; the DMP sets bit 1 each time it finishes writing a packet
        mpuIntStatus = mpu.getIntStatus();
        if (!(mpuIntStatus & 0x02))
            continue;

        fifoCount = mpu.getFIFOCount();

        // a packet the DMP was partway through writing when we reset leaves its tail at the front
        if (fifoCount % packetSize) {
            mpu.resetFIFO();
            continue;
        }

        if (fifoCount >= packetSize)
            break;
    }

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    return 1;
}

int readHeading(float f_ypr[3], bool calibration) {
    if (!readFIFOPacket())
        return 0;

	mpu.dmpGetMag(mag, fifoBuffer);
    mpu.dmpGetAccel(acc, fifoBuffer);

	if (!calibration) {
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		float mpu_mag_out[3];
		normalize_mpu(mag, mpu_mag_out);

		float magnetom[3];
		for (int i=0; i<3; i++)
			magnetom[i] = mpu_mag_out[i];

        vector m = { magnetom[0], magnetom[1], magnetom[2] };
        vector a = { gravity.x, gravity.y, gravity.z };

		f_ypr[0] = heading(a, m);
		f_ypr[1] = -ypr[1];
		f_ypr[2] = ypr[2];
	} else
		logln(F("%d, %d, %d"), mag[0], mag[1], mag[2]);

	return 1;
}

void writeCalibrationLine() {
	float f_ypr[3];
	readHeading(f_ypr, true);
}

float readSteadyHeading() {
	static float last_heading = 0;
	float f_ypr[3];

	// average as unit vectors, so readings either side of north don't average out to south
	float s = 0;
	float c = 0;
	uint8_t valid = 0;

	for (uint8_t i=0; i<MPU_LOOPS;) {
		if (readHeading(f_ypr, false)) {
			// a zero field vector (compass not answering, or no calibration) gives NaN, which would
			// stay in the heading average and the steering PID for good
			if (!isnan(f_ypr[0])) {
				s += sin(f_ypr[0]);
				c += cos(f_ypr[0]);
				valid++;
			}
			i++;
		}
		delay(MPU_PAUSES);
	}

	float heading = last_heading;
	if (valid)
		heading = toCircle(atan2(s, c) + (DEVICE_ORIENTATION * PI) + mag_offset);
	else
		logln(F("No valid compass reading, keeping the last heading"));
	last_heading = heading;

	current_pitch = f_ypr[1] * 180.0 / PI;
	current_roll = f_ypr[2] * 180.0 / PI;
	// heading = (-heading) - (PI / 2.0);
	logln(F("AHRS (y,p,r): %d, %d, %d"),
			((int16_t) (heading * 180.0 / PI)),
			((int16_t) (current_pitch)),
			((int16_t) (current_roll))
		);

	return heading;
}

float heading(vector a, vector m)
{
    // compute E and N
    vector E;
    vector N;
    vector_cross(&m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from));
    if (heading < 0) heading += 2 * PI;
    return heading;
}

void vector_cross(const vector *a, const vector *b, vector *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

float vector_dot(const vector *a, const vector *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}
#endif

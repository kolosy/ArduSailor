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

    EEPROM.get(_settingsAddress, m_min);
    EEPROM.get(_settingsAddress + sizeof(vector_i16t), m_max);
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
        return 0;
    
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
        
  while (calibrationSteps < TOTAL_CALIBRATION_STEPS)
    calibrationLoop();

  logln(F("Calibration complete"));
    
  m_min = running_min;
  m_max = running_max;

  logln(F("New values min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}"),
        m_min.x, m_min.y, m_min.z,
        m_max.x, m_max.y, m_max.z);

  logln(F("Writing to eeprom at location %d"), _settingsAddress);

  EEPROM.put(_settingsAddress, m_min);
  EEPROM.put(_settingsAddress + sizeof(vector_i16t), m_max);
}

void normalize_mpu(int16_t mag_val[3], float normalized[3]) {
    // normalize *and* re-align axes (because, you know, having all sensors be the same is too much to ask for).
    normalized[1] =   mag_val[0] - (m_min.x + m_max.x) / 2.0;
    normalized[0] =   mag_val[1] - (m_min.y + m_max.y) / 2.0;
    normalized[2] = -(mag_val[2]);
}

int readFIFOPacket() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return 0;

    // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    while ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

        // do this again.
        // reset interrupt flag and get INT_STATUS byte
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    }

    while (!(mpuIntStatus & 0x02)) {
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    }

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    return 1;
}

int readHeading(float f_ypr[3], bool calibration) {
    if (!readFIFOPacket())
        return 0;
	
	// track FIFO count here in case there is > 1 packet available
	// (this lets us immediately read more without waiting for an interrupt)
	fifoCount -= packetSize;

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
	float f_ypr[3];
	float heading = 0;

	for (uint8_t i=0; i<MPU_LOOPS;) {
		i += readHeading(f_ypr, false);
		heading += f_ypr[0];
		delay(MPU_PAUSES);
	}

	heading /= ((float)MPU_LOOPS);
	heading = toCircle(heading + (DEVICE_ORIENTATION * PI) + mag_offset);

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


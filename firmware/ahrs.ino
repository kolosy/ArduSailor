#include "MPU6050_9Axis_MotionApps41.h"

#define MPU_ITERATIONS 50

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	 // holds actual interrupt status byte from MPU
uint8_t devStatus;			// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;		// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		 // count of all bytes currently in FIFO

int mpuInit() {

	// initialize device
	Serial.println(F("Initializing MPU9150 ..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU9150 connection successful") : F("MPU9150 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	
	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);

			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println(F("DMP ready! Waiting for first interrupt..."));
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
			
			return 0;
	} else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
			
			return devStatus;
	}
}

float readSteadyHeading() {
	if (!dmpReady) return 'NaN';

	// orientation/motion vars
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	Quaternion q;					 // [w, x, y, z]				 quaternion container
	VectorFloat gravity;		// [x, y, z]						gravity vector
	float euler[3];				 // [psi, theta, phi]		Euler angle container
	float ypr[3];					 // [yaw, pitch, roll]	 yaw/pitch/roll container and gravity vector
	int16_t mag[3];
	int skipIterations = 0; // number of iterations skipped because of overflow. want to not penalize ourselves for this
	
	float heading = 0;
	
	for (int i=0; i<(MPU_ITERATIONS + skipIterations); i++) {
		// reset interrupt flag and get INT_STATUS byte
		mpuIntStatus = mpu.getIntStatus();
	
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();
	
		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
				// reset so we can continue cleanly
				mpu.resetFIFO();
				skipIterations++;
	
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (mpuIntStatus & 0x02) {
				// wait for correct available data length, should be a VERY short wait
				while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	
				// read a packet from FIFO
				mpu.getFIFOBytes(fifoBuffer, packetSize);
				
				// track FIFO count here in case there is > 1 packet available
				// (this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize;

				// display Euler angles in degrees
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				mpu.dmpGetMag(mag, fifoBuffer);

				float phi = -ypr[2];
				float theta = ypr[1];
				
				float mx = (mag[1] - (-16.2371)) / 176.6863;
				float my = (mag[0] - (-33.3806)) / 173.1679;
				float mz = (mag[2] - (12.6471)) / 155.0401;
				
				heading += atan2(-(mz * sin(phi) - my * cos(phi)), mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi));
		} else
			skipIterations++;
	}

	// rotate by 180* to get to 0-360
	return toCircle(heading / ((float) MPU_ITERATIONS) + PI);
}

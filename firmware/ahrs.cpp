#include "ahrs.h"
#include "logger.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_9Axis_MotionApps41.h"
#include "HMC58X3.h"

MPU6050 mpu;
HMC58X3 magn;

#define MPU_LOOPS 5
#define MPU_PAUSES 1

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t mag[3];


int mpuInit() {
    // initialize device
    logln("Initializing I2C devices...");
    // no delay needed as we have already a delay(5) in HMC5843::init()
    magn.init(false); // Dont set mode yet, we'll do that later on.
    // Calibrate HMC using self test, not recommended to change the gain after calibration.
    magn.calibrate(1, 32); // Use gain 1=default, valid 0-7, 7 not recommended.
    // Single mode conversion was used in calibration, now set continuous mode
    magn.setMode(0);
  
    mpu.initialize();

    // verify connection
    logln("Testing device connections...");
    mpu.testConnection() ? logln("MPU6050 connection successful") : logln("MPU6050 connection failed");

    // load and configure the DMP
    logln("Initializing DMP...");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        logln("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
//        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        logln("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        logln("DMP Initialization failed (code %d)", devStatus);
    }
}

void normalize_hmc(int16_t mag_val[3], float normalized[3]) {
  float xn = mag_val[0] - 27.6279;
  float yn = mag_val[1] + 109.8952;
  float zn = mag_val[2] - 26.3622;
  
  normalized[0] = (0.9527 * xn + 0.0124 * yn + 0.0098 * zn) / 603.6086;
  normalized[1] = (0.0124 * xn + 1.0796 * yn + 0.0012 * zn) / 603.6086;
  normalized[2] = (0.0098 * xn + 0.0012 * yn + 0.9726 * zn) / 603.6086;
}

void normalize_mpu(int16_t mag_val[3], float normalized[3]) {
  float xn = mag_val[0] - 30.2675;
  float yn = mag_val[1] + 9.4344;
  float zn = mag_val[2] - 30.0818;
  
  // normalize *and* re-align axes (because, you know, having all sensors be the same is too much to ask for).
  normalized[1] =   ( 0.9984 * xn + 0.0037 * yn - 0.0008 * zn) / 144.4383;
  normalized[0] =   ( 0.0037 * xn + 0.9736 * yn - 0.0012 * zn) / 144.4383;
  normalized[2] = - (-0.0008 * xn - 0.0012 * yn + 1.0287 * zn) / 144.4383;
}

//void normalize_hmc(int16_t mag_val[3], float normalized[3]) {
//  float xn = mag_val[0] - 14.9727;
//  float yn = mag_val[1] + 156.7135;
//  float zn = mag_val[2] - 27.9874;
//  
//  normalized[0] = (0.9622 * xn + 0.0210 * yn + 0.0111 * zn) / 625.2703;
//  normalized[1] = (0.0210 * xn + 1.0454 * yn + 0.0030 * zn) / 625.2703;
//  normalized[2] = (0.0111 * xn + 0.0030 * yn + 0.9947 * zn) / 625.2703;
//}
//
//void normalize_mpu(int16_t mag_val[3], float normalized[3]) {
//  float xn = mag_val[0] + 44.9175;
//  float yn = mag_val[1] + 25.3482;
//  float zn = mag_val[2] - 20.9063;
//  
//  // normalize *and* re-align axes (because, you know, having all sensors be the same is too much to ask for).
//  normalized[1] =   ( 0.9878 * xn + 0.0102 * yn - 0.0040 * zn) / 146.4961;
//  normalized[0] =   ( 0.0102 * xn + 0.9619 * yn - 0.0024 * zn) / 146.4961;
//  normalized[2] = - (-0.0040 * xn - 0.0024 * yn + 1.0526 * zn) / 146.4961;
//}
//
void readHeading(float f_ypr[3]) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

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
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    int ix,iy,iz;
    // Get values, as ints and floats.
    magn.getValues(&ix,&iy,&iz);
    
    mpu.dmpGetMag(mag, fifoBuffer);
    
#ifdef CALIBRATE_ONLY
    logln("%d, %d, %d, %d, %d, %d", mag[0], mag[1], mag[2], ix, iy, iz);
#else
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    int hmc_mag_in[3] = { ix, iy, iz };
    float hmc_mag_out[3];
    normalize_hmc(hmc_mag_in, hmc_mag_out);
    
    float mpu_mag_out[3];
    normalize_mpu(mag, mpu_mag_out);

    float magnetom[3];
    for (int i=0; i<3; i++)
      magnetom[i] = (hmc_mag_out[i] + mpu_mag_out[i]) / 2.0;

    float magComp[3];
    float cos_pitch = cos(-ypr[1]);
    float sin_pitch = sin(-ypr[1]);
    float cos_roll = cos(ypr[2]);
    float sin_roll = sin(ypr[2]);

    // Tilt compensated magnetic field X
    float mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    float mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
    // Magnetic Heading
    float heading = atan2(-mag_y, mag_x);

    f_ypr[0] = heading;
    f_ypr[1] = -ypr[1];
    f_ypr[2] = ypr[2];
#endif
}

void writeCalibrationLine() {
    float f_ypr[3];
    readHeading(f_ypr);
}

float readSteadyHeading() {
    float f_ypr[3];
    float heading = 0;
    
    for (uint8_t i=0; i<MPU_LOOPS; i++) {
        readHeading(f_ypr);
        heading += f_ypr[0];
        delay(MPU_PAUSES);
    }
    
    heading /= ((float)MPU_LOOPS);
    heading = toCircle(-heading);
    // heading = (-heading) - (PI / 2.0);
    logln("AHRS (y,p,r): %d, %d, %d", 
        ((int16_t) (heading * 180.0 / PI)),
        ((int16_t) (f_ypr[1] * 180.0 / PI)),
        ((int16_t) (f_ypr[2] * 180.0 / PI)));
        
    return heading;
}

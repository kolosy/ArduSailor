// Stand-in for i2cdevlib's MPU6050 class with the MotionApps 4.1 DMP calls the firmware uses. Register
// traffic goes to the simulated MPU-9250 (FIFO, interrupt status, the I2C master's magnetometer slaves);
// packets are decoded exactly as i2cdevlib decodes them, so a misaligned read gives the same garbage.
#ifndef SIM_MPU6050_H
#define SIM_MPU6050_H

#include "Arduino.h"

struct Quaternion {
    float w = 1, x = 0, y = 0, z = 0;
};

struct VectorFloat {
    float x = 0, y = 0, z = 0;
};

struct VectorInt16 {
    int16_t x = 0, y = 0, z = 0;
};

class MPU6050 {
public:
    MPU6050() {}
    explicit MPU6050(uint8_t) {}

    void initialize() { sim_imu_initialize(); }
    bool testConnection() { return true; }
    uint8_t dmpInitialize() { return (uint8_t)sim_imu_dmp_initialize(); }
    void setDMPEnabled(bool enabled) { sim_imu_set_dmp_enabled(enabled); }
    uint16_t dmpGetFIFOPacketSize() { return 48; }

    void setSlaveAddress(uint8_t num, uint8_t address) { sim_imu_set_slave_address(num, address); }
    void setSlaveOutputByte(uint8_t num, uint8_t data) { sim_imu_set_slave_output(num, data); }
    void setSlave4MasterDelay(uint8_t delay) { sim_imu_set_mst_delay(delay); }

    uint8_t getIntStatus() { return sim_imu_int_status(); }
    uint16_t getFIFOCount() { return sim_imu_fifo_count(); }
    void resetFIFO() { sim_imu_fifo_reset(); }
    void getFIFOBytes(uint8_t *data, uint8_t length) { sim_imu_fifo_read(data, length); }

    uint8_t dmpGetQuaternion(int16_t *data, const uint8_t *packet) {
        data[0] = (int16_t)((packet[0] << 8) | packet[1]);
        data[1] = (int16_t)((packet[4] << 8) | packet[5]);
        data[2] = (int16_t)((packet[8] << 8) | packet[9]);
        data[3] = (int16_t)((packet[12] << 8) | packet[13]);
        return 0;
    }
    uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t *packet) {
        int16_t qI[4];
        dmpGetQuaternion(qI, packet);
        q->w = (float)qI[0] / 16384.0f;
        q->x = (float)qI[1] / 16384.0f;
        q->y = (float)qI[2] / 16384.0f;
        q->z = (float)qI[3] / 16384.0f;
        return 0;
    }
    uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q) {
        v->x = 2 * (q->x * q->z - q->w * q->y);
        v->y = 2 * (q->w * q->x + q->y * q->z);
        v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
        return 0;
    }
    uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
        data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
        data[1] = atan2(gravity->x, sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
        data[2] = atan2(gravity->y, gravity->z);
        if (gravity->z < 0) {
            if (data[1] > 0)
                data[1] = PI - data[1];
            else
                data[1] = -PI - data[1];
        }
        return 0;
    }
    uint8_t dmpGetMag(int16_t *data, const uint8_t *packet) {
        data[0] = (int16_t)((packet[28] << 8) | packet[29]);
        data[1] = (int16_t)((packet[30] << 8) | packet[31]);
        data[2] = (int16_t)((packet[32] << 8) | packet[33]);
        return 0;
    }
    uint8_t dmpGetAccel(int16_t *data, const uint8_t *packet) {
        data[0] = (int16_t)((packet[34] << 8) | packet[35]);
        data[1] = (int16_t)((packet[38] << 8) | packet[39]);
        data[2] = (int16_t)((packet[42] << 8) | packet[43]);
        return 0;
    }
};

#endif

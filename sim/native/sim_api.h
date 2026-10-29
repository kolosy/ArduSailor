// The boundary between the firmware side (the sketch plus the fake Arduino layer, compiled with the Arduino
// macros) and the simulation engine (plain C++). Only C types cross it, so neither side sees the other's
// headers.
#ifndef SIM_API_H
#define SIM_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- time: every wait in the firmware ends up here, and runs the world forward
uint64_t sim_now_us(void);
void sim_advance_us(uint64_t us);

// ---- pins
void sim_pin_mode(int pin, int mode);
void sim_pin_write(int pin, int value);
int sim_pin_read(int pin);
int sim_analog_read(int channel);

// ---- servos (by signal pin)
void sim_servo_attach(int pin);
void sim_servo_write(int pin, int angle);
int sim_servo_read(int pin);

// ---- UARTs: 0 = Serial (USB / radio), 2 = Serial2 (GPS)
void sim_serial_begin(int port, long baud);
int sim_serial_available(int port);
int sim_serial_read(int port);
int sim_serial_peek(int port);
void sim_serial_write(int port, uint8_t c);

// ---- EEPROM
uint8_t sim_eeprom_read(int addr);
void sim_eeprom_write(int addr, uint8_t value);

// ---- MPU-9250, at the level of i2cdevlib's MPU6050 + MotionApps 4.1 calls the firmware makes
void sim_imu_initialize(void);
int sim_imu_dmp_initialize(void);
void sim_imu_set_dmp_enabled(int enabled);
void sim_imu_set_slave_address(int num, int address);
void sim_imu_set_slave_output(int num, int data);
void sim_imu_set_mst_delay(int delay);
uint8_t sim_imu_int_status(void);
uint16_t sim_imu_fifo_count(void);
void sim_imu_fifo_reset(void);
void sim_imu_fifo_read(uint8_t *data, int length);

// ---- firmware state, filled in by sketch_bridge.inc (compiled inside the sketch)
typedef struct {
    double ahrs_heading, requested_heading, wp_heading, wp_distance;
    double gps_lat, gps_lon, gps_speed, gps_course;
    uint32_t gps_age_ms;
    double current_roll, current_pitch, heel_adjust, voltage;
    int wind, target_wp, wp_direction, cycle;
    int was_beating, tack, current_tack, recovering, making_way, tack_wanted, gybing_round;
    double cross_track, leg_lat, leg_lon, corridor;
    int rebuilding, failed_tacks;
    int tuning_pid, manual_override, remote_control, serial_logging;
    double kp, ki, kd, pid_output;
    int rudder_servo_cmd, winch_servo_cmd;
    int calibration_steps, calibration_total;  // compass calibration progress (ahrs.cpp)
} simfw_state;

void simfw_snapshot(simfw_state *s);
int simfw_waypoint_count(void);
void simfw_get_waypoint(int i, double *lat, double *lon);
void simfw_set_waypoint(int i, double lat, double lon);
void simfw_set_pid(double kp, double ki, double kd);
void simfw_set_autotune(int on);

// ---- firmware #defines turned into variables by build.py (generated tunables.cpp)
typedef struct {
    const char *name, *file, *comment;
    int is_int, needs_restart;
    void *ptr;
    double default_value, min_value, max_value;
} sim_tunable;

extern const sim_tunable sim_tunables[];
extern const int sim_tunable_count;

#ifdef __cplusplus
}
#endif

#endif

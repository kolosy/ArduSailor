// The hardware the firmware talks to: pins, servos, UARTs, EEPROM, the GPS, the wind vane, the battery divider
// and the MPU-9250. Pin numbers and wiring follow firmware/*.ino and layout/boat7.sch.
#include "engine.h"
#include "sim_api.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

namespace sim {

namespace {

const double PI = 3.14159265358979323846;
const double EARTH_RADIUS = 6371000;  // same as pilot.ino

// pins (firmware.ino, servo_ctl.cpp, gps.ino, wind.ino, battery.ino)
const int SP_EN = 25, RUDDER_EN = 26, WINCH_EN = 27, GPS_EN = 30, WIND_EN = 33, BATT_V_EN = 34;
const int RUDDER_PIN = 10, WINCH_PIN = 11;
const int WS1 = 8, WS2 = 9, BATT_VAL = 10;  // analog channels

const double VANE_HEIGHT = 1.4;    // masthead, m
const double VANE_RADIUS = 300;    // hall sensor swing, ADC counts
const double R1 = 324000, R2 = 100000;
const size_t RX_BUFFER = 512;      // HardwareSerial patched to 512 bytes (see gps.ino)
const double TX_BUFFER = 64;

int pins[128];
int servo_cmd[64];

struct Uart {
    long baud = 0;
    std::deque<uint8_t> rx, incoming;
    uint64_t next_rx_us = 0;
    double tx_fill = 0;
    uint64_t tx_time = 0;
    std::string line;
};
Uart uarts[4];

uint8_t eeprom[4096];
bool eeprom_dirty = false;
std::string eeprom_file;

double vane = 0;  // where the vane points, radians clockwise from the bow

double origin_lat = 0, origin_lon = 0;
uint64_t next_fix_us = 1000000;
double gps_err_x = 0, gps_err_y = 0;
double gps_course_held = 0;

// MPU-9250
struct Imu {
    bool dmp_enabled = false;
    int slave_addr[4] = {0, 0, 0, 0};
    int slave_do[4] = {0, 0, 0, 0};
    int mst_delay = 0;
    std::deque<uint8_t> fifo;
    uint8_t int_status = 0;
    uint64_t next_packet_us = 0, next_mag_us = 0;
    int16_t mag[3] = {0, 0, 0};  // the AK8963's data registers as last read by the I2C master
};
Imu imu;

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

double wrap_pi(double a) {
    a = fmod(a + PI, 2 * PI);
    return a < 0 ? a + PI : a - PI;
}

uint64_t byte_time_us(long baud) { return baud > 0 ? (uint64_t)(10e6 / baud) : 0; }

// ---- GPS: NMEA at 1Hz, roughly what a Venus638FLPx sends by default

std::string nmea(const char *body) {
    unsigned char sum = 0;
    for (const char *c = body; *c; c++) sum ^= (unsigned char)*c;
    char out[160];
    snprintf(out, sizeof out, "$%s*%02X\r\n", body, sum);
    return out;
}

void nmea_angle(double deg, int degree_digits, char *out, char *hemi, char pos, char neg) {
    *hemi = deg < 0 ? neg : pos;
    deg = fabs(deg);
    int whole = (int)deg;
    double minutes = (deg - whole) * 60;
    snprintf(out, 16, "%0*d%07.4f", degree_digits, whole, minutes);
}

void gps_fix(uint64_t now) {
    // slowly wandering error, like a real receiver's, plus a little jitter
    const double tau = 30;
    double sigma = params.gps_noise;
    gps_err_x += -gps_err_x / tau + sqrt(2 / tau) * sigma * gaussian();
    gps_err_y += -gps_err_y / tau + sqrt(2 / tau) * sigma * gaussian();
    double lat, lon;
    local_to_latlon(boat.x + gps_err_x + 0.2 * sigma * gaussian(), boat.y + gps_err_y + 0.2 * sigma * gaussian(), &lat, &lon);

    double ve = boat.u * sin(boat.psi) + boat.v * cos(boat.psi);
    double vn = boat.u * cos(boat.psi) - boat.v * sin(boat.psi);
    double knots = std::max(0.0, hypot(ve, vn) / KNOT + 0.05 * gaussian());
    if (knots > 0.2) gps_course_held = fmod(atan2(ve, vn) / DEG + 3 * gaussian() + 360, 360);

    unsigned secs = (unsigned)(now / 1000000) + 12 * 3600;
    char hms[16], latbuf[16], lonbuf[16], ns, ew, body[128];
    snprintf(hms, sizeof hms, "%02u%02u%02u.000", (secs / 3600) % 24, (secs / 60) % 60, secs % 60);
    nmea_angle(lat, 2, latbuf, &ns, 'N', 'S');
    nmea_angle(lon, 3, lonbuf, &ew, 'E', 'W');

    std::string out;
    snprintf(body, sizeof body, "GPGGA,%s,%s,%c,%s,%c,1,08,0.9,176.0,M,-33.8,M,,0000", hms, latbuf, ns, lonbuf, ew);
    out += nmea(body);
    out += nmea("GPGSA,A,3,05,07,09,13,16,20,23,30,,,,,1.6,0.9,1.3");
    out += nmea("GPGSV,3,1,12,05,62,045,44,07,48,301,42,09,33,122,40,13,21,175,38");
    out += nmea("GPGSV,3,2,12,16,15,265,35,20,72,195,45,23,40,075,41,30,12,318,33");
    out += nmea("GPGSV,3,3,12,02,05,030,,04,08,140,,26,03,220,,29,02,350,");
    snprintf(body, sizeof body, "GPRMC,%s,A,%s,%c,%s,%c,%05.1f,%05.1f,281026,,,A", hms, latbuf, ns, lonbuf, ew, knots,
             gps_course_held);
    out += nmea(body);
    snprintf(body, sizeof body, "GPVTG,%05.1f,T,,M,%05.1f,N,%05.1f,K,A", gps_course_held, knots, knots * 1.852);
    out += nmea(body);

    Uart &u = uarts[2];
    if (u.incoming.empty()) u.next_rx_us = now;
    u.incoming.insert(u.incoming.end(), out.begin(), out.end());
}

// ---- the MPU-9250

struct Mat {
    double m[3][3];
};

Mat mul(const Mat &a, const Mat &b) {
    Mat r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

Mat rot_x(double a) { return {{{1, 0, 0}, {0, cos(a), -sin(a)}, {0, sin(a), cos(a)}}}; }
Mat rot_y(double a) { return {{{cos(a), 0, sin(a)}, {0, 1, 0}, {-sin(a), 0, cos(a)}}}; }
Mat rot_z(double a) { return {{{cos(a), -sin(a), 0}, {sin(a), cos(a), 0}, {0, 0, 1}}}; }

// Sensor axes to world (north, west, up). The boat's axes are x forward, y port, z up; the board is mounted
// with its x axis pointing aft (ahrs.cpp's DEVICE_ORIENTATION), so the sensor's x and y are the boat's reversed.
Mat sensor_to_world() {
    double heading, pitch, roll;
    boat_attitude(&heading, &pitch, &roll);
    Mat boat_to_world = mul(mul(rot_z(-heading), rot_y(-pitch)), rot_x(roll));
    Mat mount = {{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}};
    return mul(boat_to_world, mount);
}

// world vector in sensor axes
void to_sensor(const Mat &r, const double w[3], double s[3]) {
    for (int i = 0; i < 3; i++) s[i] = r.m[0][i] * w[0] + r.m[1][i] * w[1] + r.m[2][i] * w[2];
}

void quaternion(const Mat &r, double q[4]) {
    const double(*m)[3] = r.m;
    double tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0) {
        double s = sqrt(tr + 1.0) * 2;
        q[0] = 0.25 * s;
        q[1] = (m[2][1] - m[1][2]) / s;
        q[2] = (m[0][2] - m[2][0]) / s;
        q[3] = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        double s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
        q[0] = (m[2][1] - m[1][2]) / s;
        q[1] = 0.25 * s;
        q[2] = (m[0][1] + m[1][0]) / s;
        q[3] = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        double s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
        q[0] = (m[0][2] - m[2][0]) / s;
        q[1] = (m[0][1] + m[1][0]) / s;
        q[2] = 0.25 * s;
        q[3] = (m[1][2] + m[2][1]) / s;
    } else {
        double s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
        q[0] = (m[1][0] - m[0][1]) / s;
        q[1] = (m[0][2] + m[2][0]) / s;
        q[2] = (m[1][2] + m[2][1]) / s;
        q[3] = 0.25 * s;
    }
}

void put32(std::deque<uint8_t> &out, int32_t v) {
    out.push_back((uint8_t)(v >> 24));
    out.push_back((uint8_t)(v >> 16));
    out.push_back((uint8_t)(v >> 8));
    out.push_back((uint8_t)v);
}

void put16(std::deque<uint8_t> &out, int16_t v) {
    out.push_back((uint8_t)(v >> 8));
    out.push_back((uint8_t)v);
}

// the I2C master reads the AK8963 (slave 0) and starts its next measurement (slave 2) every (1 + delay) samples
void sample_magnetometer() {
    bool addressed = imu.slave_addr[0] == (0x80 | 0x0C) && imu.slave_addr[2] == 0x0C;
    bool triggered = (imu.slave_do[2] & 0x0F) == 0x01;
    if (!addressed || !triggered) return;  // nothing answers: the data registers keep their old contents

    double inc = params.mag_inclination * DEG, dec = params.mag_declination * DEG;
    double world[3] = {params.mag_field * cos(inc) * cos(dec), -params.mag_field * cos(inc) * sin(dec),
                       -params.mag_field * sin(inc)};
    double s[3];
    to_sensor(sensor_to_world(), world, s);
    // the AK8963's axes: x along the accelerometer's y, y along its x, z opposite (MPU-9250 datasheet)
    double ak[3] = {s[1] + params.mag_offset_x, s[0] + params.mag_offset_y, -s[2] + params.mag_offset_z};
    bool sixteen = (imu.slave_do[2] & 0x10) != 0;
    double scale = sixteen ? 0.15 : 0.6;
    double limit = sixteen ? 32760 : 8190;
    for (int i = 0; i < 3; i++)
        imu.mag[i] = (int16_t)lround(clamp((ak[i] + params.mag_noise * gaussian()) / scale, -limit, limit));
}

// a 48-byte MotionApps 4.1 packet: quaternion, gyro, magnetometer, accelerometer
void dmp_packet() {
    Mat r = sensor_to_world();
    double q[4];
    quaternion(r, q);
    double up[3] = {0, 0, 1}, g[3];
    to_sensor(r, up, g);

    std::deque<uint8_t> p;
    for (int i = 0; i < 4; i++) put32(p, (int32_t)lround(clamp(q[i], -1.99, 1.99) * 1073741824.0));
    for (int i = 0; i < 3; i++) put32(p, 0);
    for (int i = 0; i < 3; i++) put16(p, imu.mag[i]);
    for (int i = 0; i < 3; i++) put32(p, (int32_t)lround(g[i] * 8192) * 65536);
    put16(p, 0);

    imu.fifo.insert(imu.fifo.end(), p.begin(), p.end());
    while (imu.fifo.size() > 512) {  // full: the oldest bytes go, packets and all
        imu.fifo.pop_front();
        imu.int_status |= 0x10;
    }
    imu.int_status |= 0x02;
}

void i2c(uint64_t us) { sim_advance_us(us); }

}  // namespace

// ---- engine side

void devices_reset() {
    memset(pins, 0, sizeof pins);
    for (int &c : servo_cmd) c = -1;
    memset(eeprom, 0xFF, sizeof eeprom);
}

void devices_tick(uint64_t now) {
    // GPS sentences arrive a byte at a time; past the receive buffer they're lost, as on the real board
    Uart &gps = uarts[2];
    uint64_t bt = byte_time_us(gps.baud);
    while (!gps.incoming.empty() && gps.next_rx_us <= now) {
        if (gps.rx.size() < RX_BUFFER) gps.rx.push_back(gps.incoming.front());
        gps.incoming.pop_front();
        gps.next_rx_us += bt;
    }
    if (now >= next_fix_us) {
        next_fix_us += 1000000;
        if (pins[GPS_EN] == 0 && gps.baud > 0) gps_fix(now);
    }

    // masthead vane: follows the apparent wind with a little lag
    double awa, aws;
    apparent_wind_at(VANE_HEIGHT, &awa, &aws);
    if (aws > 0.3) {
        double target = awa + params.vane_offset * DEG;
        vane += wrap_pi(target - vane) * std::min(1.0, 0.001 / std::max(0.001, params.vane_lag));
    }

    if (imu.dmp_enabled) {
        while (now >= imu.next_mag_us) {
            sample_magnetometer();
            imu.next_mag_us += (uint64_t)(1 + imu.mst_delay) * 5000;
        }
        while (now >= imu.next_packet_us) {
            dmp_packet();
            imu.next_packet_us += 20000;  // MPU6050_DMP_FIFO_RATE_DIVISOR 3: 200Hz / 4
        }
    }
}

bool servo_powered(int pin) {
    int en = pin == RUDDER_PIN ? RUDDER_EN : pin == WINCH_PIN ? WINCH_EN : -1;
    return en >= 0 && pins[SP_EN] && pins[en] && servo_cmd[pin] >= 0;
}

int servo_command(int pin) { return servo_cmd[pin]; }

double gps_origin_lat() { return origin_lat; }
double gps_origin_lon() { return origin_lon; }

void set_gps_origin(double lat, double lon) {
    origin_lat = lat;
    origin_lon = lon;
}

void local_to_latlon(double x, double y, double *lat, double *lon) {
    *lat = origin_lat + y / EARTH_RADIUS / DEG;
    *lon = origin_lon + x / (EARTH_RADIUS * cos(origin_lat * DEG)) / DEG;
}

void latlon_to_local(double lat, double lon, double *x, double *y) {
    *y = (lat - origin_lat) * DEG * EARTH_RADIUS;
    *x = (lon - origin_lon) * DEG * EARTH_RADIUS * cos(origin_lat * DEG);
}

void serial_input(const std::string &text) {
    for (char c : text)
        if (uarts[0].rx.size() < RX_BUFFER) uarts[0].rx.push_back((uint8_t)c);
}

void eeprom_load(const std::string &path) {
    eeprom_file = path;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return;
    size_t n = fread(eeprom, 1, sizeof eeprom, f);
    (void)n;
    fclose(f);
}

void eeprom_save_if_dirty() {
    if (!eeprom_dirty || eeprom_file.empty()) return;
    FILE *f = fopen(eeprom_file.c_str(), "wb");
    if (!f) return;
    fwrite(eeprom, 1, sizeof eeprom, f);
    fclose(f);
    eeprom_dirty = false;
}

int vane_reading_deg() { return (int)lround(fmod(vane / DEG + 360, 360)); }

}  // namespace sim

using namespace sim;

// ---- firmware side (sim_api.h)

extern "C" {

void sim_pin_mode(int, int) {}

void sim_pin_write(int pin, int value) {
    if (pin >= 0 && pin < 128) pins[pin] = value ? 1 : 0;
}

int sim_pin_read(int pin) { return pin >= 0 && pin < 128 ? pins[pin] : 0; }

int sim_analog_read(int channel) {
    if (channel == WS1 || channel == WS2) {
        if (pins[WIND_EN]) return 0;  // p-channel switch: the vane is only powered with WIND_EN low
        double a = vane + params.vane_noise * DEG * gaussian();
        double v = channel == WS1 ? 500 - VANE_RADIUS * sin(a) : 500 + VANE_RADIUS * cos(a);
        return (int)clamp(lround(v), 0, 1023);
    }
    if (channel == BATT_VAL) {
        if (!pins[BATT_V_EN]) return 0;
        double v = params.battery * R2 / (R1 + R2) / 3.3 * 1023;
        return (int)clamp(lround(v + 0.5 * gaussian()), 0, 1023);
    }
    return 0;
}

void sim_servo_attach(int pin) {
    if (pin >= 0 && pin < 64 && servo_cmd[pin] < 0) servo_cmd[pin] = 90;
}

void sim_servo_write(int pin, int angle) {
    if (pin >= 0 && pin < 64) servo_cmd[pin] = (int)clamp(angle, 0, 180);
}

int sim_servo_read(int pin) { return pin >= 0 && pin < 64 ? std::max(0, servo_cmd[pin]) : 0; }

void sim_serial_begin(int port, long baud) {
    if (port >= 0 && port < 4) uarts[port].baud = baud;
}

int sim_serial_available(int port) { return port >= 0 && port < 4 ? (int)uarts[port].rx.size() : 0; }

int sim_serial_read(int port) {
    if (port < 0 || port >= 4 || uarts[port].rx.empty()) return -1;
    int c = uarts[port].rx.front();
    uarts[port].rx.pop_front();
    return c;
}

int sim_serial_peek(int port) {
    if (port < 0 || port >= 4 || uarts[port].rx.empty()) return -1;
    return uarts[port].rx.front();
}

// a full transmit buffer blocks the firmware until bytes go out at the baud rate, as on the board
void sim_serial_write(int port, uint8_t c) {
    if (port < 0 || port >= 4) return;
    Uart &u = uarts[port];
    if (u.baud > 0) {
        double bt = 10e6 / u.baud;
        uint64_t now = sim_now_us();
        u.tx_fill = std::max(0.0, u.tx_fill - (now - u.tx_time) / bt);
        u.tx_time = now;
        if (u.tx_fill >= TX_BUFFER) {
            sim_advance_us((uint64_t)((u.tx_fill - TX_BUFFER + 1) * bt));
            now = sim_now_us();
            u.tx_fill = std::max(0.0, u.tx_fill - (now - u.tx_time) / bt);
            u.tx_time = now;
        }
        u.tx_fill += 1;
    }
    if (port != 0) return;
    if (c == '\n') {
        emit_serial(u.line);
        u.line.clear();
    } else if (c != '\r' && u.line.size() < 4000) {
        u.line += (char)c;
    }
}

uint8_t sim_eeprom_read(int addr) { return addr >= 0 && addr < 4096 ? eeprom[addr] : 0xFF; }

void sim_eeprom_write(int addr, uint8_t value) {
    if (addr < 0 || addr >= 4096) return;
    sim_advance_us(3300);  // an EEPROM write takes 3.3ms on the ATmega2560
    eeprom[addr] = value;
    eeprom_dirty = true;
}

void sim_imu_initialize(void) { i2c(3000); }

// what i2cdevlib's MotionApps 4.1 dmpInitialize() leaves behind, including its AK8975-era magnetometer setup
int sim_imu_dmp_initialize(void) {
    i2c(1100000);  // uploading and verifying the DMP image over 100kHz I2C
    imu = Imu();
    imu.slave_addr[0] = 0x8E;
    imu.slave_addr[2] = 0x0E;
    imu.slave_do[2] = 0x01;
    imu.mst_delay = 0x18;
    return 0;
}

void sim_imu_set_dmp_enabled(int enabled) {
    i2c(300);
    imu.dmp_enabled = enabled != 0;
    imu.next_packet_us = sim_now_us() + 20000;
    imu.next_mag_us = sim_now_us();
}

void sim_imu_set_slave_address(int num, int address) {
    i2c(300);
    if (num >= 0 && num < 4) imu.slave_addr[num] = address;
}

void sim_imu_set_slave_output(int num, int data) {
    i2c(300);
    if (num >= 0 && num < 4) imu.slave_do[num] = data;
}

void sim_imu_set_mst_delay(int delay) {
    i2c(300);
    imu.mst_delay = delay & 0x1F;
}

uint8_t sim_imu_int_status(void) {
    i2c(300);
    uint8_t s = imu.int_status;
    imu.int_status = 0;
    return s;
}

uint16_t sim_imu_fifo_count(void) {
    i2c(400);
    return (uint16_t)imu.fifo.size();
}

void sim_imu_fifo_reset(void) {
    i2c(300);
    imu.fifo.clear();
}

void sim_imu_fifo_read(uint8_t *data, int length) {
    i2c(200 + 90 * (uint64_t)length);
    for (int i = 0; i < length; i++) {
        data[i] = imu.fifo.empty() ? 0 : imu.fifo.front();
        if (!imu.fifo.empty()) imu.fifo.pop_front();
    }
}

}  // extern "C"

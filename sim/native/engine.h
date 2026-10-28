// Shared state of the simulation engine: parameters, the boat, and the hooks between the world model, the
// simulated devices and the main loop.
#ifndef SIM_ENGINE_H
#define SIM_ENGINE_H

#include <stdint.h>
#include <random>
#include <string>

namespace sim {

const double KNOT = 0.514444;  // m/s
const double DEG = 3.14159265358979323846 / 180.0;
const double G = 9.81;

// Everything the UI can change about the world, the boat and its sensors. Registered with ranges and labels in
// main.cpp.
struct Params {
    // wind
    double wind_dir = 290;     // degrees, direction it blows from
    double wind_speed = 8;     // knots
    double gust = 15;          // % of the wind speed, standard deviation
    double shift = 8;          // degrees, standard deviation
    double wind_period = 20;   // seconds, how quickly gusts and shifts come and go

    // waves
    double wave_height = 0.08;  // metres, crest to trough
    double wave_period = 1.8;   // seconds
    double wave_angle = 0;      // degrees from the wind direction

    // boat
    double mass = 4.0;           // kg
    double sail_area = 0.40;     // m^2
    double heel_arm = 0.50;      // m, sail centre of effort above the keel's centre of lateral resistance
    double gm = 0.25;            // m, metacentric height (stability)
    double helm_balance = 0.01;  // m, centre of effort aft of the centre of lateral resistance (weather helm)
    double rudder_ratio = 1.0;   // rudder degrees per servo degree
    double rudder_servo_speed = 300;  // servo degrees per second
    double winch_servo_speed = 25;    // servo degrees per second
    double sheet_in_servo = 56;       // winch servo position with the sheet all the way in
    double sheet_out_servo = 120;     // ... and all the way out
    double boom_in = 5;               // boom angle (degrees off the centreline) allowed with the sheet all in
    double boom_out = 85;             // ... and all out

    // sensors
    double mag_inclination = 68.4;  // degrees: the real field the compass sees
    double mag_declination = -4.2;  // degrees, east positive
    double mag_field = 52.7;        // microtesla
    double mag_offset_x = 0;        // hard-iron offsets on the AK8963's own axes, microtesla
    double mag_offset_y = 0;
    double mag_offset_z = 0;
    double mag_noise = 0.3;   // microtesla, per reading
    double vane_noise = 2;    // degrees, per reading
    double vane_lag = 0.15;   // seconds
    double vane_offset = 0;   // degrees the vane is mounted off true
    double gps_noise = 1.5;   // metres
    double battery = 11.8;    // volts
};

struct Boat {
    // state, in a local frame: x east, y north (metres); headings clockwise from true north (radians)
    double x = 0, y = 0, psi = 0;
    double u = 0, v = 0, r = 0;   // surge, sway (+ starboard), yaw rate (+ clockwise)
    double phi = 0, p = 0;        // heel (+ starboard side down) and its rate
    double theta = 0, heave = 0;  // pitch (+ bow up) and heave, driven by the waves
    double boom = 0;              // radians off the centreline, + to starboard
    double rudder_servo = 90, winch_servo = 56;  // where the servos actually are

    // derived each step, for sensors and telemetry
    double tws = 0, twd = 0;      // true wind, m/s and radians (from)
    double awa = 0, aws = 0;      // apparent wind at the sail: radians (+ from starboard), m/s
    double rudder = 0;            // rudder angle, radians (+ trailing edge to starboard: turns to starboard)
    double sheet_limit = 0;       // how far out the sheet lets the boom go, radians
    double alpha = 0, lift = 0, drag = 0, drive = 0, side = 0;  // sail angle of attack and forces (N)
    double wave_phase = 0;
};

extern Params params;
extern Boat boat;
extern std::mt19937 rng;
double gaussian();

// world.cpp
void world_reset(double x, double y, double heading_deg);
void world_step(double dt);
double wind_at(double x, double y, double *dir);  // true wind speed (m/s) and direction (radians, from)
// apparent wind seen by the masthead vane: radians clockwise from the bow (0 = from ahead), m/s
void apparent_wind_at(double height, double *awa, double *aws);
void boat_attitude(double *heading, double *pitch, double *roll);

// devices.cpp
void devices_reset();
void devices_tick(uint64_t now_us);
bool servo_powered(int pin);
int servo_command(int pin);
double gps_origin_lat();
double gps_origin_lon();
void set_gps_origin(double lat, double lon);
void local_to_latlon(double x, double y, double *lat, double *lon);
void latlon_to_local(double lat, double lon, double *x, double *y);
void eeprom_load(const std::string &path);
void eeprom_save_if_dirty();
void serial_input(const std::string &text);  // characters typed into the firmware's Serial port
int vane_reading_deg();

// world.cpp: lift the boat out and turn it through every orientation, for a 3-D compass calibration
void tumble_for(double seconds);
bool tumbling();
void tumble_attitude(double t, double *heading, double *pitch, double *roll);

// main.cpp
void emit_serial(const std::string &line);  // a line the firmware printed on Serial

}  // namespace sim

#endif

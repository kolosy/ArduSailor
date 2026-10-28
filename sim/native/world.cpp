// The world: wind with gusts and shifts, waves, and a ~1m single-sail RC yacht. Three degrees of freedom in the
// water plane (surge, sway, yaw) plus heel, with pitch and heave driven directly by the waves. The constants
// are picked to behave plausibly for a boat that size, not fitted to measurements.
#include "engine.h"

#include <algorithm>
#include <cmath>

namespace sim {

Params params;
Boat boat;
std::mt19937 rng(12345);

double gaussian() {
    static std::normal_distribution<double> normal(0.0, 1.0);
    return normal(rng);
}

namespace {

const double PI = 3.14159265358979323846;
const double RHO_AIR = 1.2;
const double RHO_WATER = 1000;
const double CE_HEIGHT = 0.5;    // sail centre of effort above the waterline, m (for roll-induced wind)
const double RUDDER_AREA = 0.004;
const double RUDDER_ARM = 0.40;  // rudder behind the centre of lateral resistance, m
const double BOOM_RATE = 4.0;    // fastest the boom swings, rad/s
const double HULL_WINDAGE = 0.05; // hull and deck area times drag coefficient, m^2
const double WINDAGE_ARM = 0.15;  // hull windage centre ahead of the centre of lateral resistance, m
const int RUDDER_PIN = 10, WINCH_PIN = 11;

double gust_n = 0, shift_n = 0;  // unit-variance random processes behind the gusts and shifts
double t = 0;
double tumble_left = 0, tumble_t = 0;

double wrap_pi(double a) {
    a = fmod(a + PI, 2 * PI);
    return a < 0 ? a + PI : a - PI;
}

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

double toward(double from, double to, double max_step) { return from + clamp(to - from, -max_step, max_step); }

// sail: lift peaks at ~25 degrees angle of attack, then stalls; drag grows to a flat plate's at 90
void sail_coefficients(double alpha, double *cl, double *cd) {
    const double stall = 25 * DEG, cl_max = 1.4;
    double a = alpha <= PI / 2 ? alpha : PI - alpha;
    double c = a <= stall ? cl_max * sin(a / stall * PI / 2) : cl_max * cos((a - stall) / (PI / 2 - stall) * PI / 2);
    *cl = alpha <= PI / 2 ? c : -c;
    *cd = 0.05 + 1.2 * sin(alpha) * sin(alpha) + 0.05 * c * c;
}

// low aspect ratio rudder: lift slope ~3/rad, stalling past ~20 degrees
double rudder_cl(double angle) {
    double a = fabs(angle);
    double c = a <= 0.35 ? 3.0 * a : std::max(0.4, 1.05 - 1.5 * (a - 0.35));
    return angle < 0 ? -c : c;
}

// friction plus a wall of wave-making drag past hull speed (~1.2 m/s for a 0.9m waterline)
double hull_resistance(double u) {
    double a = fabs(u);
    double r = 0.6 * a + 1.6 * a * a;
    if (a > 1.2) r += 8 * (a - 1.2) * (a - 1.2);
    return u < 0 ? -r : r;
}

void apparent(double height, double *aw_f, double *aw_s) {
    double dir;
    double tws = wind_at(boat.x, boat.y, &dir);
    double we = -tws * sin(dir), wn = -tws * cos(dir);  // wind velocity (east, north)
    double fe = sin(boat.psi), fn = cos(boat.psi);       // forward
    double se = cos(boat.psi), sn = -sin(boat.psi);      // starboard
    double ae = we - (boat.u * fe + boat.v * se);
    double an = wn - (boat.u * fn + boat.v * sn);
    *aw_f = ae * fe + an * fn;
    // rolling to starboard swings the rig to starboard, so the air comes from that side
    *aw_s = ae * se + an * sn - boat.p * height;
}

void update_rigging(double dt) {
    if (servo_powered(RUDDER_PIN))
        boat.rudder_servo = toward(boat.rudder_servo, servo_command(RUDDER_PIN), params.rudder_servo_speed * dt);
    if (servo_powered(WINCH_PIN))
        boat.winch_servo = toward(boat.winch_servo, servo_command(WINCH_PIN), params.winch_servo_speed * dt);

    boat.rudder = clamp((boat.rudder_servo - 90) * params.rudder_ratio, -60, 60) * DEG;

    double span = params.sheet_out_servo - params.sheet_in_servo;
    double frac = span != 0 ? (boat.winch_servo - params.sheet_in_servo) / span : 0;
    boat.sheet_limit = clamp(params.boom_in + frac * (params.boom_out - params.boom_in), 0, 90) * DEG;
}

void update_wind(double dt) {
    double tau = std::max(1.0, params.wind_period);
    double k = sqrt(2 * dt / tau);
    gust_n += -gust_n / tau * dt + k * gaussian();
    shift_n += -shift_n / tau * dt + k * gaussian();
    boat.tws = wind_at(boat.x, boat.y, &boat.twd);
}

void step_tumble(double dt) {
    tumble_t += dt;
    tumble_left -= dt;
    boat.u = boat.v = boat.r = boat.p = 0;
    tumble_attitude(tumble_t, &boat.psi, &boat.theta, &boat.phi);
    if (tumble_left <= 0) {
        boat.phi = boat.theta = 0;
        tumble_left = 0;
    }
}

}  // namespace

double wind_at(double, double, double *dir) {
    *dir = (params.wind_dir + params.shift * shift_n) * DEG;
    return std::max(0.0, params.wind_speed * KNOT * (1 + params.gust / 100 * gust_n));
}

void apparent_wind_at(double height, double *awa, double *aws) {
    double f, s;
    apparent(height, &f, &s);
    *aws = hypot(f, s);
    *awa = atan2(-s, -f);
}

void boat_attitude(double *heading, double *pitch, double *roll) {
    *heading = boat.psi;
    *pitch = boat.theta;
    *roll = boat.phi;
}

void world_reset(double x, double y, double heading_deg) {
    double rudder = boat.rudder_servo, winch = boat.winch_servo;
    boat = Boat();
    boat.rudder_servo = rudder;
    boat.winch_servo = winch;
    boat.x = x;
    boat.y = y;
    boat.psi = heading_deg * DEG;
    tumble_left = 0;
}

void tumble_for(double seconds) {
    tumble_left = seconds;
    tumble_t = 0;
}

bool tumbling() { return tumble_left > 0; }

// slow turns in heading, full rolls and steep pitching, so every orientation comes round
void tumble_attitude(double s, double *heading, double *pitch, double *roll) {
    *heading = fmod(2 * PI * s / 9.0, 2 * PI);
    *roll = wrap_pi(2 * PI * s / 6.0);
    *pitch = 75 * DEG * sin(2 * PI * s / 7.3);
}

void world_step(double dt) {
    t += dt;
    update_wind(dt);
    update_rigging(dt);
    if (tumble_left > 0) {
        step_tumble(dt);
        return;
    }

    const double m = params.mass;
    const double mass_u = 1.1 * m, mass_v = 1.5 * m;  // with added mass of water
    const double inertia_z = m * 0.25 * 0.25, inertia_x = m * 0.2 * 0.2;

    // waves: deep-water sinusoid; the boat pitches and rolls with the slope and gets pushed around
    double period = std::max(0.3, params.wave_period);
    double length = G * period * period / (2 * PI);
    double k = 2 * PI / length, omega = 2 * PI / period;
    double slope = k * params.wave_height / 2;
    double to = (params.wind_dir + params.wave_angle) * DEG + PI;  // the direction the waves travel
    boat.wave_phase = omega * t - k * (boat.x * sin(to) + boat.y * cos(to));
    double rel = wrap_pi(to - boat.psi);
    boat.theta = 0.8 * slope * cos(rel) * cos(boat.wave_phase);
    boat.heave = params.wave_height / 2 * sin(boat.wave_phase);
    double wave_roll = m * G * params.gm * slope * sin(rel) * cos(boat.wave_phase);
    double wave_yaw = inertia_z * 1.5 * slope * sin(2 * rel) * omega * omega * sin(boat.wave_phase);
    double wave_surge = 0.15 * m * G * slope * cos(rel) * sin(boat.wave_phase);

    // sail: the boom swings to leeward until the sheet stops it
    double aw_f, aw_s;
    apparent(CE_HEIGHT, &aw_f, &aw_s);
    boat.aws = hypot(aw_f, aw_s);
    boat.awa = atan2(-aw_s, -aw_f);
    double target = boat.aws > 0.2 ? (boat.awa > 0 ? -1 : 1) * std::min(fabs(boat.awa), boat.sheet_limit) : boat.boom;
    boat.boom = toward(boat.boom, target, BOOM_RATE * dt);

    double sail_f = 0, sail_s = 0;
    if (boat.aws > 0.01) {
        double df = aw_f / boat.aws, ds = aw_s / boat.aws;          // direction the air flows
        double cf = -cos(boat.boom), cs = sin(boat.boom);            // boom, from the mast aft
        boat.alpha = acos(clamp(cf * df + cs * ds, -1, 1));
        bool left = cf * ds - cs * df > 0;
        double lf = left ? -ds : ds, ls = left ? df : -df;           // lift, square to the flow
        double cl, cd;
        sail_coefficients(boat.alpha, &cl, &cd);
        double q = 0.5 * RHO_AIR * boat.aws * boat.aws * params.sail_area * cos(boat.phi) * cos(boat.phi);
        boat.lift = q * cl;
        boat.drag = q * cd;
        sail_f = boat.lift * lf + boat.drag * df;
        sail_s = boat.lift * ls + boat.drag * ds;
    }
    boat.drive = sail_f;
    boat.side = sail_s;

    // rudder: works on the water flowing past the stern, which swings out when the boat turns. Going backwards
    // the flow comes from behind, so both the sideslip's effect and the lift reverse. Stalled, it's a brake.
    double stern_v = boat.v - boat.r * RUDDER_ARM;
    double flow2 = boat.u * boat.u + stern_v * stern_v;
    double attack = boat.rudder + atan(stern_v / (boat.u >= 0 ? std::max(boat.u, 0.05) : std::min(boat.u, -0.05)));
    double rudder_lift = 0.5 * RHO_WATER * RUDDER_AREA * flow2 * rudder_cl(attack) * (boat.u < 0 ? -1 : 1);
    double rudder_drag = 0.5 * RHO_WATER * RUDDER_AREA * flow2 *
                         (0.02 + 0.1 * rudder_cl(attack) * rudder_cl(attack) + 1.1 * sin(attack) * sin(attack));

    // windage on the hull, centred ahead of the keel: blows the bow off the wind when the boat stops
    double hull_q = 0.5 * RHO_AIR * boat.aws * boat.aws * HULL_WINDAGE;
    double windage_f = -hull_q * cos(boat.awa), windage_s = -hull_q * sin(boat.awa);

    double X = sail_f + windage_f - hull_resistance(boat.u) - (boat.u < 0 ? -rudder_drag : rudder_drag) + wave_surge;
    double Y = sail_s + windage_s - 50 * fabs(boat.u) * boat.v - 15 * boat.v * fabs(boat.v) - rudder_lift;
    double N = RUDDER_ARM * rudder_lift - (0.1 + 0.3 * fabs(boat.u)) * boat.r
               - 0.25 * sail_f * params.heel_arm * sin(boat.phi)  // heeled, the drive pulls the bow to windward
               - params.helm_balance * sail_s + WINDAGE_ARM * windage_s + wave_yaw;
    double K = sail_s * params.heel_arm * cos(boat.phi) - m * G * params.gm * sin(boat.phi) - 0.8 * boat.p + wave_roll;

    boat.u += (X / mass_u + boat.v * boat.r) * dt;
    boat.v += (Y / mass_v - boat.u * boat.r) * dt;
    boat.r += N / inertia_z * dt;
    boat.p += K / inertia_x * dt;
    boat.phi = clamp(boat.phi + boat.p * dt, -85 * DEG, 85 * DEG);
    boat.psi = fmod(boat.psi + boat.r * dt + 2 * PI, 2 * PI);
    boat.x += (boat.u * sin(boat.psi) + boat.v * cos(boat.psi)) * dt;
    boat.y += (boat.u * cos(boat.psi) - boat.v * sin(boat.psi)) * dt;
}

}  // namespace sim

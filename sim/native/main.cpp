// Runs the firmware's setup() and loop() against the simulated world. All time passes through
// sim_advance_us(), called whenever the firmware waits (delay, millis, serial output, I2C): it steps the world,
// keeps pace with the wall clock, takes commands from stdin and writes JSON telemetry to stdout.
//
// stdin, one command per line:
//   set <param> <value>         world, boat and sensor parameters (listed in the info message)
//   tune <NAME> <value>         firmware #define made adjustable by build.py
//   pid <kp> <ki> <kd>          through the firmware's own updateCurrentPIDTunings()
//   autotune <0|1>              start or stop the firmware's PID autotune
//   wp <index> <lat> <lon>      move a waypoint
//   serial <text>               type into the firmware's Serial port (\n, \r and \\ escapes)
//   speed <multiple>            0 runs as fast as possible
//   pause <0|1>
//   reset                       put the boat back at the start, stopped
//   place <x> <y> <heading>     put the boat somewhere (metres east and north of the origin, degrees)
//   tumble <seconds>            lift the boat out and turn it every which way (for a 3-D compass calibration)
//   quit
// stdout, one JSON object per line, by "type": info, frame, serial, event, param, tunable, waypoints, summary.
#include "engine.h"
#include "sim_api.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// the sketch
void setup();
void loop();
void serialEvent2() __attribute__((weak));

namespace sim {

namespace {

typedef std::chrono::steady_clock Clock;

const uint64_t PHYSICS_STEP_US = 5000;

uint64_t now = 0, next_physics = PHYSICS_STEP_US, next_tick = 1000;

struct Options {
    std::string eeprom;
    double speed = 1;
    bool headless = false;
    double duration = 0;
    double frame_interval = 0.1;
    bool paused = false;
    unsigned seed = 1;
    double fps = 30;
};
Options opt;

double speed = 1;
bool paused = false;
Clock::time_point wall_anchor, last_frame_wall, last_save_wall, rate_wall;
uint64_t sim_anchor = 0, last_frame_sim = 0, last_check_sim = 0, rate_sim = 0;
double realtime = 1;
bool busy = false;

std::mutex queue_mutex;
std::deque<std::string> commands;
std::atomic<bool> commands_waiting(false);
std::atomic<bool> quitting(false);
std::deque<std::string> firmware_commands;  // run between loop() calls

uint64_t loop_started = 0;
double loop_ms = 0;
int last_target_wp = -1;
double start_x = 0, start_y = 0, start_heading = 0;

// ---- JSON output

std::string num(double v, int precision = 6) {
    if (!std::isfinite(v)) return "null";
    char buf[32];
    snprintf(buf, sizeof buf, "%.*g", precision, v);
    return buf;
}

std::string str(const std::string &s) {
    std::string out = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += (char)c;
        } else if (c < 0x20 || c == 0x7f) {  // bytes past 0x7f are UTF-8 and pass through
            char buf[8];
            snprintf(buf, sizeof buf, "\\u%04x", c);
            out += buf;
        } else {
            out += (char)c;
        }
    }
    return out + "\"";
}

struct Obj {
    std::string s = "{";
    Obj &add(const char *key, const std::string &raw) {
        if (s.size() > 1) s += ',';
        s += '"';
        s += key;
        s += "\":";
        s += raw;
        return *this;
    }
    Obj &n(const char *key, double v, int precision = 6) { return add(key, num(v, precision)); }
    Obj &b(const char *key, bool v) { return add(key, v ? "true" : "false"); }
    Obj &t(const char *key, const std::string &v) { return add(key, str(v)); }
    std::string done() const { return s + "}"; }
};

void send(const std::string &json) {
    fwrite(json.data(), 1, json.size(), stdout);
    fputc('\n', stdout);
    fflush(stdout);
}

void event(const std::string &text) { send(Obj().t("type", "event").t("text", text).done()); }

// ---- the world, boat and sensor parameters the UI can change

struct ParamInfo {
    const char *key, *group, *label, *unit;
    double min, max, step;
    double *value;
    double def;
};

std::vector<ParamInfo> &param_table() {
    static std::vector<ParamInfo> t;
    if (!t.empty()) return t;
    Params &p = params;
    t = {
        {"wind_dir", "Wind", "Direction (from)", "deg", 0, 359, 1, &p.wind_dir, 0},
        {"wind_speed", "Wind", "Speed", "kn", 0, 30, 0.5, &p.wind_speed, 0},
        {"gust", "Wind", "Gusts", "%", 0, 60, 1, &p.gust, 0},
        {"shift", "Wind", "Shifts", "deg", 0, 45, 1, &p.shift, 0},
        {"wind_period", "Wind", "Gust and shift period", "s", 2, 120, 1, &p.wind_period, 0},
        {"wave_height", "Waves", "Height", "m", 0, 1, 0.01, &p.wave_height, 0},
        {"wave_period", "Waves", "Period", "s", 0.5, 6, 0.1, &p.wave_period, 0},
        {"wave_angle", "Waves", "Direction off the wind", "deg", -180, 180, 5, &p.wave_angle, 0},
        {"mass", "Boat", "Displacement", "kg", 1, 10, 0.1, &p.mass, 0},
        {"sail_area", "Boat", "Sail area", "m²", 0.1, 1, 0.01, &p.sail_area, 0},
        {"heel_arm", "Boat", "Heeling arm", "m", 0.2, 1, 0.01, &p.heel_arm, 0},
        {"gm", "Boat", "Stability (GM)", "m", 0.05, 0.6, 0.01, &p.gm, 0},
        {"helm_balance", "Boat", "Weather helm (CE aft of CLR)", "m", -0.1, 0.1, 0.005, &p.helm_balance, 0},
        {"rudder_ratio", "Servos & rigging", "Rudder deg per servo deg", "", 0.2, 2, 0.05, &p.rudder_ratio, 0},
        {"rudder_servo_speed", "Servos & rigging", "Rudder servo speed", "deg/s", 10, 1000, 10, &p.rudder_servo_speed, 0},
        {"winch_servo_speed", "Servos & rigging", "Winch servo speed", "deg/s", 1, 200, 1, &p.winch_servo_speed, 0},
        {"sheet_in_servo", "Servos & rigging", "Winch servo at sheet all in", "deg", 0, 180, 1, &p.sheet_in_servo, 0},
        {"sheet_out_servo", "Servos & rigging", "Winch servo at sheet all out", "deg", 0, 180, 1, &p.sheet_out_servo, 0},
        {"boom_in", "Servos & rigging", "Boom angle, sheet all in", "deg", 0, 45, 1, &p.boom_in, 0},
        {"boom_out", "Servos & rigging", "Boom angle, sheet all out", "deg", 30, 90, 1, &p.boom_out, 0},
        {"mag_inclination", "Sensors", "Earth field inclination", "deg", -90, 90, 0.1, &p.mag_inclination, 0},
        {"mag_declination", "Sensors", "Earth field declination", "deg", -30, 30, 0.1, &p.mag_declination, 0},
        {"mag_field", "Sensors", "Earth field strength", "µT", 20, 70, 0.1, &p.mag_field, 0},
        {"mag_offset_x", "Sensors", "Compass hard iron, AK8963 x", "µT", -100, 100, 0.5, &p.mag_offset_x, 0},
        {"mag_offset_y", "Sensors", "Compass hard iron, AK8963 y", "µT", -100, 100, 0.5, &p.mag_offset_y, 0},
        {"mag_offset_z", "Sensors", "Compass hard iron, AK8963 z", "µT", -100, 100, 0.5, &p.mag_offset_z, 0},
        {"mag_noise", "Sensors", "Compass noise", "µT", 0, 5, 0.05, &p.mag_noise, 0},
        {"vane_noise", "Sensors", "Wind vane noise", "deg", 0, 20, 0.5, &p.vane_noise, 0},
        {"vane_lag", "Sensors", "Wind vane lag", "s", 0.01, 2, 0.01, &p.vane_lag, 0},
        {"vane_offset", "Sensors", "Wind vane mounting error", "deg", -45, 45, 1, &p.vane_offset, 0},
        {"gps_noise", "Sensors", "GPS error", "m", 0, 10, 0.1, &p.gps_noise, 0},
        {"battery", "Sensors", "Battery", "V", 9, 13, 0.1, &p.battery, 0},
    };
    for (ParamInfo &i : t) i.def = *i.value;
    return t;
}

ParamInfo *find_param(const std::string &key) {
    for (ParamInfo &i : param_table())
        if (key == i.key) return &i;
    return nullptr;
}

const sim_tunable *find_tunable(const std::string &name) {
    for (int i = 0; i < sim_tunable_count; i++)
        if (name == sim_tunables[i].name) return &sim_tunables[i];
    return nullptr;
}

double tunable_value(const sim_tunable &t) { return t.is_int ? *(int *)t.ptr : *(double *)t.ptr; }

bool set_param(const std::string &key, double v) {
    ParamInfo *p = find_param(key);
    if (!p) return false;
    *p->value = std::max(p->min, std::min(p->max, v));
    send(Obj().t("type", "param").t("key", key).n("value", *p->value).done());
    return true;
}

bool set_tunable(const std::string &name, double v) {
    const sim_tunable *t = find_tunable(name);
    if (!t) return false;
    v = std::max(t->min_value, std::min(t->max_value, v));
    if (t->is_int)
        *(int *)t->ptr = (int)lround(v);
    else
        *(double *)t->ptr = v;
    send(Obj().t("type", "tunable").t("name", name).n("value", tunable_value(*t)).done());
    return true;
}

// ---- messages

std::string waypoints_json() {
    std::string out = "[";
    for (int i = 0; i < simfw_waypoint_count(); i++) {
        double lat, lon, x, y;
        simfw_get_waypoint(i, &lat, &lon);
        latlon_to_local(lat, lon, &x, &y);
        if (i) out += ',';
        out += "[" + num(lat, 10) + "," + num(lon, 10) + "," + num(x) + "," + num(y) + "]";
    }
    return out + "]";
}

void send_info() {
    std::string ps = "[";
    for (const ParamInfo &i : param_table()) {
        if (ps.size() > 1) ps += ',';
        ps += Obj().t("key", i.key).t("group", i.group).t("label", i.label).t("unit", i.unit).n("min", i.min)
                  .n("max", i.max).n("step", i.step).n("value", *i.value).n("default", i.def).done();
    }
    ps += "]";
    std::string ts = "[";
    for (int k = 0; k < sim_tunable_count; k++) {
        const sim_tunable &t = sim_tunables[k];
        if (ts.size() > 1) ts += ',';
        ts += Obj().t("name", t.name).t("file", t.file).t("comment", t.comment).b("int", t.is_int)
                  .b("restart", t.needs_restart).n("value", tunable_value(t)).n("default", t.default_value)
                  .n("min", t.min_value).n("max", t.max_value).done();
    }
    ts += "]";
    send(Obj().t("type", "info")
             .add("origin", "[" + num(gps_origin_lat(), 10) + "," + num(gps_origin_lon(), 10) + "]")
             .add("start", "[" + num(start_x) + "," + num(start_y) + "," + num(start_heading) + "]")
             .add("params", ps)
             .add("tunables", ts)
             .add("waypoints", waypoints_json())
             .done());
}

void send_frame() {
    simfw_state fw;
    simfw_snapshot(&fw);
    double gx, gy, lx, ly;
    latlon_to_local(fw.gps_lat, fw.gps_lon, &gx, &gy);
    latlon_to_local(fw.leg_lat, fw.leg_lon, &lx, &ly);
    double ve = boat.u * sin(boat.psi) + boat.v * cos(boat.psi);
    double vn = boat.u * cos(boat.psi) - boat.v * sin(boat.psi);
    double length = G * params.wave_period * params.wave_period / (2 * 3.14159265358979);

    std::string b = Obj()
                        .n("x", boat.x).n("y", boat.y).n("heading", boat.psi / DEG, 5)
                        .n("speed", boat.u / KNOT, 4).n("sog", hypot(ve, vn) / KNOT, 4)
                        .n("cog", fmod(atan2(ve, vn) / DEG + 360, 360), 4)
                        .n("leeway", atan2(boat.v, std::max(0.05, fabs(boat.u))) / DEG, 3)
                        .n("heel", boat.phi / DEG, 4).n("pitch", boat.theta / DEG, 3).n("heave", boat.heave, 3)
                        .n("yaw_rate", boat.r / DEG, 4).n("rudder", boat.rudder / DEG, 4)
                        .n("rudder_servo", boat.rudder_servo, 4).n("winch_servo", boat.winch_servo, 4)
                        .b("rudder_powered", servo_powered(10)).b("winch_powered", servo_powered(11))
                        .n("sheet_limit", boat.sheet_limit / DEG, 4).n("boom", boat.boom / DEG, 4)
                        .n("awa", boat.awa / DEG, 4).n("aws", boat.aws / KNOT, 4).n("alpha", boat.alpha / DEG, 3)
                        .n("lift", boat.lift, 3).n("drag", boat.drag, 3).n("drive", boat.drive, 3)
                        .n("side", boat.side, 3).n("vane", vane_reading_deg()).b("tumbling", tumbling())
                        .done();
    std::string env = Obj()
                          .n("tws", boat.tws / KNOT, 4).n("twd", fmod(boat.twd / DEG + 360, 360), 4)
                          .n("wave_height", params.wave_height, 3).n("wave_length", length, 4)
                          .n("wave_dir", fmod(params.wind_dir + params.wave_angle + 360, 360), 4)
                          .n("wave_phase", boat.wave_phase, 6)
                          .done();
    std::string f = Obj()
                        .n("heading", fw.ahrs_heading, 5).n("requested", fw.requested_heading, 5)
                        .n("wp_heading", fw.wp_heading, 5).n("wp_distance", fw.wp_distance, 4)
                        .n("target_wp", fw.target_wp).n("direction", fw.wp_direction).n("wind", fw.wind)
                        .n("roll", fw.current_roll, 4).n("pitch", fw.current_pitch, 4)
                        .n("heel_adjust", fw.heel_adjust, 4).n("gps_lat", fw.gps_lat, 10).n("gps_lon", fw.gps_lon, 10)
                        .n("gps_x", fw.gps_lat != 0 ? gx : NAN).n("gps_y", fw.gps_lat != 0 ? gy : NAN)
                        .n("gps_speed", fw.gps_speed, 4).n("gps_course", fw.gps_course, 4)
                        .n("gps_age_ms", fw.gps_age_ms).b("beating", fw.was_beating).n("tack", fw.tack)
                        .n("on_tack", fw.current_tack).b("recovering", fw.recovering).b("making_way", fw.making_way)
                        .b("tack_wanted", fw.tack_wanted).b("gybing_round", fw.gybing_round)
                        .n("cross_track", fw.cross_track, 4).n("corridor", fw.corridor, 4)
                        .b("rebuilding", fw.rebuilding).n("failed_tacks", fw.failed_tacks)
                        .n("leg_x", fw.leg_lat != 0 ? lx : NAN).n("leg_y", fw.leg_lat != 0 ? ly : NAN)
                        .n("cycle", fw.cycle, 10).n("kp", fw.kp).n("ki", fw.ki).n("kd", fw.kd)
                        .n("pid_output", fw.pid_output, 4).n("rudder_cmd", fw.rudder_servo_cmd)
                        .n("winch_cmd", fw.winch_servo_cmd).b("autotune", fw.tuning_pid)
                        .b("manual", fw.manual_override).b("remote", fw.remote_control)
                        .b("logging", fw.serial_logging).n("voltage", fw.voltage, 4)
                        .n("cal_steps", fw.calibration_steps).n("cal_total", fw.calibration_total)
                        .done();
    send(Obj().t("type", "frame").n("t", now / 1e6, 9).n("speed", speed).b("paused", paused)
             .n("realtime", realtime, 3).n("loop_ms", loop_ms, 4)
             .add("boat", b).add("env", env).add("fw", f).done());
}

// ---- commands

std::string unescape(const std::string &s) {
    std::string out;
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '\\' && i + 1 < s.size()) {
            char c = s[++i];
            out += c == 'n' ? '\n' : c == 'r' ? '\r' : c == 't' ? '\t' : c;
        } else {
            out += s[i];
        }
    }
    return out;
}

void reanchor() {
    wall_anchor = Clock::now();
    sim_anchor = now;
}

void handle(const std::string &line) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd.empty()) return;

    if (cmd == "set") {
        std::string key;
        double v;
        if (!(in >> key >> v) || !set_param(key, v)) event("unknown parameter: " + line);
    } else if (cmd == "tune") {
        std::string name;
        double v;
        if (!(in >> name >> v) || !set_tunable(name, v)) event("unknown firmware setting: " + line);
    } else if (cmd == "pid" || cmd == "autotune" || cmd == "wp") {
        firmware_commands.push_back(line);
    } else if (cmd == "serial") {
        std::string rest;
        std::getline(in, rest);
        if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
        serial_input(unescape(rest));
    } else if (cmd == "speed") {
        double v;
        if (in >> v) {
            speed = std::max(0.0, v);
            reanchor();
        }
    } else if (cmd == "pause") {
        int v;
        if (in >> v) {
            paused = v != 0;
            reanchor();
        }
    } else if (cmd == "reset") {
        world_reset(start_x, start_y, start_heading);
        event("boat back at the start");
    } else if (cmd == "place") {
        double x, y, h;
        if (in >> x >> y >> h) world_reset(x, y, h);
    } else if (cmd == "tumble") {
        double s;
        if (in >> s) {
            tumble_for(s);
            event("boat lifted out and tumbled for a 3-D compass calibration");
        }
    } else if (cmd == "quit") {
        quitting = true;
    } else {
        event("unknown command: " + line);
    }
}

void run_firmware_commands() {
    while (!firmware_commands.empty()) {
        std::istringstream in(firmware_commands.front());
        firmware_commands.pop_front();
        std::string cmd;
        in >> cmd;
        if (cmd == "pid") {
            double kp, ki, kd;
            if (in >> kp >> ki >> kd) simfw_set_pid(kp, ki, kd);
        } else if (cmd == "autotune") {
            int on;
            if (in >> on) simfw_set_autotune(on);
        } else if (cmd == "wp") {
            int i;
            double lat, lon;
            if (in >> i >> lat >> lon) {
                simfw_set_waypoint(i, lat, lon);
                send(Obj().t("type", "waypoints").add("list", waypoints_json()).done());
            }
        }
    }
}

void take_commands() {
    std::deque<std::string> batch;
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        batch.swap(commands);
        commands_waiting = false;
    }
    for (const std::string &line : batch) handle(line);
}

void read_stdin() {
    std::string line;
    while (std::getline(std::cin, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::lock_guard<std::mutex> lock(queue_mutex);
        commands.push_back(line);
        commands_waiting = true;
    }
    if (!opt.headless) quitting = true;  // the runner went away
}

void finish() {
    eeprom_save_if_dirty();
    if (opt.headless) {
        simfw_state fw;
        simfw_snapshot(&fw);
        send(Obj().t("type", "summary").n("t", now / 1e6).n("x", boat.x).n("y", boat.y)
                 .n("target_wp", fw.target_wp).n("cycle", fw.cycle).done());
    }
    fflush(stdout);
    std::_Exit(0);
}

void watch_firmware() {
    simfw_state fw;
    simfw_snapshot(&fw);
    if (last_target_wp >= 0 && fw.target_wp != last_target_wp)
        event("waypoint " + std::to_string(last_target_wp) + " reached, heading for " + std::to_string(fw.target_wp));
    last_target_wp = fw.target_wp;
}

// once per simulated millisecond
void housekeeping() {
    if (busy) return;
    busy = true;

    if (commands_waiting) take_commands();
    while (paused && !quitting) {
        if (Clock::now() - last_frame_wall > std::chrono::milliseconds(100)) {
            send_frame();
            last_frame_wall = Clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (commands_waiting) take_commands();
        if (!paused) reanchor();
    }

    Clock::time_point wall = Clock::now();
    if (!opt.headless && speed > 0) {
        Clock::time_point target = wall_anchor + std::chrono::microseconds((int64_t)((now - sim_anchor) / speed));
        if (target > wall + std::chrono::milliseconds(2)) {
            std::this_thread::sleep_until(target);
            wall = Clock::now();
        } else if (wall - target > std::chrono::milliseconds(250)) {
            reanchor();  // can't keep up: drop the backlog rather than race to catch up
        }
    }

    double rate_span = std::chrono::duration<double>(wall - rate_wall).count();
    if (rate_span >= 1) {
        realtime = (now - rate_sim) / 1e6 / rate_span;
        rate_wall = wall;
        rate_sim = now;
    }

    if (opt.headless) {
        if (opt.frame_interval > 0 && now - last_frame_sim >= (uint64_t)(opt.frame_interval * 1e6)) {
            send_frame();
            last_frame_sim = now;
        }
    } else if (wall - last_frame_wall >= std::chrono::microseconds((int64_t)(1e6 / opt.fps))) {
        send_frame();
        last_frame_wall = wall;
    }

    if (now - last_check_sim >= 100000) {
        watch_firmware();
        last_check_sim = now;
    }
    if (wall - last_save_wall > std::chrono::seconds(1)) {
        eeprom_save_if_dirty();
        last_save_wall = wall;
    }
    if (quitting || (opt.duration > 0 && now >= (uint64_t)(opt.duration * 1e6))) finish();
    busy = false;
}

void usage() {
    fprintf(stderr,
            "usage: ardusailor-sim [--eeprom FILE] [--speed X] [--paused] [--headless] [--duration S]\n"
            "                      [--frame-interval S] [--seed N] [--set PARAM=VALUE]... [--tune NAME=VALUE]...\n");
    exit(2);
}

}  // namespace

uint64_t now_us() { return now; }

void emit_serial(const std::string &line) { send(Obj().t("type", "serial").t("text", line).done()); }

}  // namespace sim

using namespace sim;

extern "C" uint64_t sim_now_us(void) { return now; }

extern "C" void sim_advance_us(uint64_t us) {
    uint64_t target = now + us;
    while (now < target) {
        now = std::min(target, std::min(next_physics, next_tick));
        if (now >= next_physics) {
            world_step(PHYSICS_STEP_US / 1e6);
            next_physics += PHYSICS_STEP_US;
        }
        if (now >= next_tick) {
            devices_tick(now);
            next_tick += 1000;
            housekeeping();
        }
    }
}

int main(int argc, char **argv) {
    std::vector<std::pair<std::string, double>> sets, tunes;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) usage();
            return argv[++i];
        };
        auto pair = [&](std::vector<std::pair<std::string, double>> &into) {
            std::string kv = value();
            size_t eq = kv.find('=');
            if (eq == std::string::npos) usage();
            into.push_back({kv.substr(0, eq), atof(kv.c_str() + eq + 1)});
        };
        if (a == "--eeprom") opt.eeprom = value();
        else if (a == "--speed") opt.speed = atof(value().c_str());
        else if (a == "--paused") opt.paused = true;
        else if (a == "--headless") opt.headless = true;
        else if (a == "--duration") opt.duration = atof(value().c_str());
        else if (a == "--frame-interval") opt.frame_interval = atof(value().c_str());
        else if (a == "--seed") opt.seed = (unsigned)atoi(value().c_str());
        else if (a == "--set") pair(sets);
        else if (a == "--tune") pair(tunes);
        else usage();
    }

    rng.seed(opt.seed);
    srand(opt.seed);
    devices_reset();
    if (!opt.eeprom.empty()) eeprom_load(opt.eeprom);

    // the course is centred on the firmware's waypoints; the boat starts 25m south of the first one
    int n = simfw_waypoint_count();
    double lat = 0, lon = 0;
    for (int i = 0; i < n; i++) {
        double a, b;
        simfw_get_waypoint(i, &a, &b);
        lat += a / n;
        lon += b / n;
    }
    set_gps_origin(lat, lon);
    double wx, wy, wlat, wlon;
    simfw_get_waypoint(0, &wlat, &wlon);
    latlon_to_local(wlat, wlon, &wx, &wy);
    start_x = wx;
    start_y = wy - 25;
    start_heading = 0;
    world_reset(start_x, start_y, start_heading);

    for (auto &s : sets)
        if (!find_param(s.first)) fprintf(stderr, "unknown parameter %s\n", s.first.c_str());
        else {
            ParamInfo *p = find_param(s.first);
            *p->value = std::max(p->min, std::min(p->max, s.second));
        }
    for (auto &s : tunes)
        if (!find_tunable(s.first)) fprintf(stderr, "unknown firmware setting %s\n", s.first.c_str());
        else set_tunable(s.first, s.second);

    speed = opt.headless ? 0 : opt.speed;
    paused = opt.paused;
    rate_wall = last_frame_wall = last_save_wall = Clock::now();
    reanchor();
    send_info();

    std::thread(read_stdin).detach();

    setup();
    for (;;) {
        loop_ms = (now - loop_started) / 1000.0;
        loop_started = now;
        loop();
        if (serialEvent2 && sim_serial_available(2) > 0) serialEvent2();
        run_firmware_commands();
    }
}

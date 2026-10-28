// Appended to the generated sketch.cpp by build.py, so it sees the sketch's globals with their real types.

extern int16_t calibrationSteps;  // ahrs.cpp
extern int sim_TOTAL_CALIBRATION_STEPS;

extern "C" void simfw_snapshot(simfw_state *s) {
    s->ahrs_heading = ahrs_heading;
    s->requested_heading = requested_heading;
    s->wp_heading = wp_heading;
    s->wp_distance = wp_distance;
    s->gps_lat = gps_lat;
    s->gps_lon = gps_lon;
    s->gps_speed = gps_speed;
    s->gps_course = gps_course;
    s->gps_age_ms = (uint32_t)(sim_now_us() / 1000 - last_gps_time);
    s->current_roll = current_roll;
    s->current_pitch = current_pitch;
    s->heel_adjust = heel_adjust;
    s->voltage = voltage;
    s->wind = wind;
    s->target_wp = target_wp;
    s->wp_direction = direction;
    s->cycle = (int)cycle;
    s->was_beating = was_beating;
    s->tack = tack;
    s->current_tack = current_tack;
    s->recovering = recovering;
    s->making_way = making_way;
    s->tack_wanted = tack_wanted_since != 0;
    s->gybing_round = gybing_round;
    s->cross_track = cross_track;
    s->leg_lat = leg_lat;
    s->leg_lon = leg_lon;
    s->corridor = CORRIDOR;
    s->pointing = pointing;
    s->tuning_pid = tuningPID;
    s->manual_override = manual_override;
    s->remote_control = remote_control;
    s->serial_logging = serial_logging;
    s->kp = steeringPID.GetKp();
    s->ki = steeringPID.GetKi();
    s->kd = steeringPID.GetKd();
    s->pid_output = new_rudder;
    s->rudder_servo_cmd = current_rudder;
    s->winch_servo_cmd = current_winch;
    s->calibration_steps = calibrationSteps;
    s->calibration_total = sim_TOTAL_CALIBRATION_STEPS;
}

extern "C" int simfw_waypoint_count(void) { return WP_COUNT; }

extern "C" void simfw_get_waypoint(int i, double *lat, double *lon) {
    *lat = wp_list[i * 2];
    *lon = wp_list[i * 2 + 1];
}

extern "C" void simfw_set_waypoint(int i, double lat, double lon) {
    if (i < 0 || i >= WP_COUNT) return;
    wp_list[i * 2] = lat;
    wp_list[i * 2 + 1] = lon;
    // the pilot keeps its own copy of the waypoint it's heading for
    if (i == target_wp) {
        wp_lat = lat;
        wp_lon = lon;
    }
}

extern "C" void simfw_set_pid(double kp, double ki, double kd) {
    double tunings[3] = {kp, ki, kd};
    updateCurrentPIDTunings(tunings);
}

extern "C" void simfw_set_autotune(int on) { tuningPID = on != 0; }

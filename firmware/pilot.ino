#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>

// radius of Earth in m
#define R 6371000

// minimum speed needed to establish course (knots)
#define MIN_SPEED 1.0

// only tack with at least this much speed on (knots): a tack started slowly ends in irons. In the simulator, tacks
// from under 1.5 knots mostly failed and from over 2 almost all worked.
#define MIN_TACK_SPEED 2.0

// how close you have to get to the waypoint to consider it hit
#define GET_WITHIN 5

// adjust sails when we're more than this much off-plan
#define SAIL_ADJUST_ON 10

// either side of 0 for "in irons"
#define IRONS 40
#define IN_IRONS(v) (((v) < IRONS || (v) > (360 - IRONS)))

// either side of 180 for "running"
#define ON_RUN 20

// either side of 180 for must gybe / might gybe accidentally
#define GYBE_AT 10
#define MIGHT_GYBE(v) (((v) > 180 - GYBE_AT) && ((v) < 180 + GYBE_AT))

// straighten rudder when we make this much of a turn
#define TACK_START_STRAIGHT 50
#define GYBE_START_STRAIGHT 50

// slower than this (knots), or drifting backwards, for STALL_TIME (ms) means we're in irons. Once we've made way
// again for RECOVER_TIME (ms), we're out
#define STALL_SPEED 0.5
#define STALL_TIME 3000
#define RECOVER_TIME 5000

// have sails at this position for a gybe
#define GYBE_SAIL_POS 80

#define SERVO_ORIENTATION -1

// beating up-wind, zigzag up a corridor this many metres either side of the line from where the leg started to
// the waypoint, tacking at its edges
#define CORRIDOR 15

// don't tack again within this long (ms) of the last tack, or of getting out of irons
#define MIN_TACK_INTERVAL 10000

// tack for a waypoint that's off the wind on the other tack only once it's this many degrees outside the no-go
// zone: the apparent wind swings aft after a tack, so the new tack can't point quite as high as it looks from here
#define LAYLINE_MARGIN 10

// the apparent wind has to be this many degrees off the bow before we trust which side it's on
#define WIND_SIDE_MIN 15

// tack from, and onto, TACK_MARGIN degrees outside the no-go zone: come up to that edge, then go through the
// wind, the smallest turn there is. Not up to MIN_TACK_SPEED at the edge within TACK_WAIT (ms), or the last
// MAX_TACK_TRIES tacks all ended in irons: go round the other way, bearing away until the wind is GYBE_FROM
// degrees off the bow and then gybing
#define TACK_MARGIN 10
#define TACK_WAIT 15000
#define MAX_TACK_TRIES 2
#define GYBE_FROM 150

// in irons, fall off this many degrees from the wind to get going again
#define RECOVER_ANGLE 70

// if we're at more than this heel, start easing the mainsheet
#define START_HEEL_COMP 30
#define MAX_HEEL_COMP 30

// if heel compensation drops below this amount, zero it out
#define MIN_HEEL_COMP 5

// we're going decrease the amount of heel adjust by this factor to bring things back slowly
#define HEEL_COMP_DECAY 0.1

// the trim map in adjustSails() puts the fully eased sheet at WINCH_MIN, so easing moves toward it
#define EASE_SHEET(v) ((WINCH_MIN > WINCH_MAX) ? (v) : -(v))

// how close we can get to our waypoint before we switch to High Res GPS
#define HRG_THRESHOLD 50

// how long to wait for a complete RC command before we ditch it
#define RC_TIMEOUT 1000

#define EEPROM_START_ADDR 0

// # of waypoints below
#define WP_COUNT 4

// lat,lon pairs
float wp_list[] =
  {
	41.920708, -87.630361,
	41.921237, -87.630292,
	41.921202, -87.630787,
	41.920902, -87.630388
	//
	// 41.923584, -87.631463,
	// 41.923004, -87.631139,
	//     41.9207923576838 ,-87.63011004661024,
	//     41.92040355439754,-87.63045132003815,
	//     41.92024341159895,-87.63004258543924,
	//     41.91948366250413,-87.62996748502381,
	//     41.91848868180579,-87.62960748575088,
	//     41.91771524159618,-87.62928507512221,
	//     41.91665396400703,-87.62889035219517 // end of circuit
  };
int8_t direction = 1;

// current lat,lon
float wp_lat, wp_lon;
int target_wp = 0;

uint32_t last_turn = 0;

float ahrs_offset = 0;
uint8_t offset_set = 0;

// not real fusion for now
double fused_heading = 0;

int16_t turning_by = 0;
boolean turning = false;
boolean tacking = false;

// the tack we're on, from the wind vane: 1 is starboard (wind over the starboard side), -1 port. tack is the one
// we're sailing for; the two differ while we're tacking or gybing.
int8_t current_tack = 1;
int8_t tack = 1;
uint32_t last_tack = 0;
boolean was_beating = false;

// when we started wanting the other tack (0: we don't), and whether we're going round by gybing
uint32_t tack_wanted_since = 0;
boolean gybing_round = false;

// a tack through the wind started at tack_started and isn't settled yet; tacks in a row that ended in irons
boolean tacking_through = false;
uint32_t tack_started = 0;
uint8_t failed_tacks = 0;

// after a tack or getting out of irons, sail TACK_MARGIN free of close-hauled until we're back up to speed
boolean rebuilding = false;
uint32_t rebuild_since = 0;

// in irons: not making way (too slow, or drifting backwards) for STALL_TIME
boolean making_way = false;
boolean recovering = false;
uint32_t last_made_way = 0;
uint32_t last_lost_way = 0;

// where the leg to the current waypoint started, the line's bearing, and how far we are to the right (+) or
// left (-) of the line, in m
float leg_lat = 0, leg_lon = 0;
float leg_bearing = 0;
float cross_track = 0;

double new_rudder = 0;

// the PID steers on a continuous ("unwrapped") heading so it never sees the jump at 0/360 and
// always turns the short way. steer_input accumulates the shortest turn between readings, and
// steer_setpoint sits the shortest signed distance to requested_heading away from it.
double steer_input = 0;
double steer_setpoint = 0;
float last_fused_heading = 0;

int16_t _pilotSettingsAddress = 0;
double aTuneStep=5, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

// Specify the links and initial tuning parameters
PID steeringPID(&steer_input, &new_rudder, &steer_setpoint, 0.1, 0.001, 2.8, P_ON_E, DIRECT);
PID_ATune pidTune(&steer_input, &new_rudder);

inline void toPort(int amt) {
    rudderTo(current_rudder + (SERVO_ORIENTATION * amt));
}

inline void toSbord(int amt) {
    rudderTo(current_rudder - (SERVO_ORIENTATION * amt));
}

inline void fuseHeading() {
    // no fusion for now. todo: add gps-based mag calibration compensation
    fused_heading = ahrs_heading;
}

inline void unwrapHeadings() {
    // a NaN would stay in the accumulator, and so in the PID, for good
    if (isnan(fused_heading) || isnan(requested_heading))
        return;

    steer_input += angleDiff(last_fused_heading, fused_heading, true);
    last_fused_heading = fused_heading;
    steer_setpoint = steer_input + angleDiff(fused_heading, requested_heading, true);
}

// lat/lon and result in radians
float computeBearing(float i_lat, float i_lon, float f_lat, float f_lon) {
    float y = sin(f_lon-i_lon) * cos(f_lat);
    float x = cos(i_lat)*sin(f_lat) - sin(i_lat)*cos(f_lat)*cos(f_lon-i_lon);
    return atan2(y, x);
}

// lat/lon in radians. returns distance in meters
float computeDistance(float i_lat, float i_lon, float f_lat, float f_lon) {
    float a = sin((f_lat - i_lat)/2) * sin((f_lat - i_lat)/2) +
                        cos(i_lat) * cos(f_lat) *
                        sin((f_lon - i_lon)/2) * sin((f_lon - i_lon)/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    return R * c;
}

// metres east and north of lat0, lon0 (degrees). Flat is close enough over a course.
void offsetFrom(float lat0, float lon0, float lat, float lon, float *east, float *north) {
    *north = RAD(lat - lat0) * R;
    *east = RAD(lon - lon0) * R * cos(RAD(lat0));
}

inline boolean haveFix() {
    return gps_lat != 0.0 || gps_lon != 0.0;
}

// a new leg starts where we are now
void startLeg() {
    leg_lat = gps_lat;
    leg_lon = gps_lon;
}

void updateCrossTrack() {
    float line_e, line_n, boat_e, boat_n;
    offsetFrom(leg_lat, leg_lon, wp_lat, wp_lon, &line_e, &line_n);
    offsetFrom(leg_lat, leg_lon, gps_lat, gps_lon, &boat_e, &boat_n);

    float length = sqrt(line_e * line_e + line_n * line_n);
    leg_bearing = toCircleDeg(DEG(atan2(line_e, line_n)));
    cross_track = length > 1 ? (boat_e * line_n - boat_n * line_e) / length : 0;
}

void startRebuilding() {
    rebuilding = true;
    rebuild_since = millis();
}

void updateStall() {
    // GPS speed has no sign: drifting backwards shows up as a course opposite the heading
    making_way = gps_speed >= STALL_SPEED && angleDiff(gps_course, ahrs_heading, false) < 90;
    if (making_way)
        last_made_way = millis();
    else
        last_lost_way = millis();

    if (!recovering && millis() - last_made_way > STALL_TIME) {
        recovering = true;
        if (tacking_through && millis() - tack_started < MIN_TACK_INTERVAL)
            failed_tacks++;
        tacking_through = false;
        tack_wanted_since = 0;
        gybing_round = false;
        logln(F("Not making way. Falling off to get out of irons."));
    } else if (recovering && millis() - last_lost_way > RECOVER_TIME) {
        // stay on the tack we fell off onto for a while: tacking straight back is what put us in irons
        recovering = false;
        tack = current_tack;
        last_tack = millis();
        startRebuilding();
        logln(F("Making way again."));
    }
}

void adjustSails() {
#ifdef NO_SAIL
    return;
#endif

    logln(F("Checking sail trim"));
    if (abs(current_roll) > START_HEEL_COMP)
        heel_adjust = min(MAX_HEEL_COMP, 2 * (abs(current_roll) - START_HEEL_COMP));
    else if (heel_adjust > MIN_HEEL_COMP)
        heel_adjust -= heel_adjust * HEEL_COMP_DECAY;
    else
        heel_adjust = 0;

    float new_winch = map(constrain(abs(wind - 180), IRONS, 180), IRONS, 180, WINCH_MIN, WINCH_MAX) + EASE_SHEET(heel_adjust);

    // stuck in irons, let the sheet go: hauled in, the main only turns the bow back into the wind
    if (recovering && !making_way)
        new_winch = WINCH_MIN;

    if (abs(new_winch - current_winch) > SAIL_ADJUST_ON) {
        logln(F("New winch position of %d is more than %d off from %d. Adjusting trim."), (int16_t) new_winch, SAIL_ADJUST_ON, current_winch);
        adjustment_made = true;
        winchTo(new_winch);
    } else
        logln(F("No trim adjustment needed"));
}

void autotune() {
    if (pidTune.Runtime()) {
        logln(F("PID Autotuning complete"));
        double tunings[] = {pidTune.GetKp(), pidTune.GetKi(), pidTune.GetKd()};
        updateCurrentPIDTunings(tunings);
        tuningPID = false;
    }
}

// the heading angle degrees off the wind on tack t (world_wind is where the wind comes from)
inline float offWind(float world_wind, int8_t t, float angle) {
    return toCircleDeg(world_wind - t * angle);
}

// close-hauled on tack t: IRONS off the wind, or a little freer while we build speed after a tack or irons
inline float closeHauled(float world_wind, int8_t t) {
    return offWind(world_wind, t, rebuilding ? IRONS + TACK_MARGIN : IRONS);
}

// whether sailing close-hauled on tack t takes us further from the leg's line
boolean awayFromLine(float world_wind, int8_t t) {
    float across = sin(RAD(angleDiff(leg_bearing, closeHauled(world_wind, t), true)));
    return cross_track > 0 ? across > 0 : across < 0;
}

void changeTack(boolean through_wind) {
    tack = -tack;
    last_tack = millis();
    tacking_through = through_wind;
    if (through_wind) {
        tack_started = millis();
        startRebuilding();
    } else
        failed_tacks = 0;
    logln(F("%s onto %s tack"), through_wind ? "Tacking" : "Gybing", tack > 0 ? "starboard" : "port");
}

// the course on tack t: straight for the waypoint whenever we can get there on this tack, otherwise close-hauled
float courseOn(float world_wind, int8_t t, float wp_from_wind) {
    int8_t wp_tack = wp_from_wind < 0 ? 1 : -1;
    return abs(wp_from_wind) < IRONS || wp_tack != t ? closeHauled(world_wind, t) : wp_heading;
}

// The heading to steer for the waypoint. It's never inside the no-go zone, and it only crosses the wind (a tack)
// with way on, from the edge of the no-go zone, and not too soon after the last tack. Upwind, it zigzags
// close-hauled up the corridor around the leg's line, until the waypoint can be reached on one tack.
float chooseCourse(float world_wind) {
    // where the waypoint and the bow are from the wind: negative to the left of it, as on starboard tack
    float wp_from_wind = angleDiff(world_wind, wp_heading, true);
    float heading_from_wind = angleDiff(world_wind, fused_heading, true);
    int8_t wp_tack = wp_from_wind < 0 ? 1 : -1;

    was_beating = abs(wp_from_wind) < IRONS;

    boolean want_other;
    if (was_beating)
        want_other = abs(cross_track) > CORRIDOR && awayFromLine(world_wind, tack);
    else if (wp_tack == tack)
        want_other = false;
    else if (abs(heading_from_wind) + abs(wp_from_wind) > 180) {
        // the waypoint is round the other side of the wind behind us: gybe, which we can do at any speed
        changeTack(false);
        want_other = false;
    } else
        want_other = abs(wp_from_wind) > IRONS + LAYLINE_MARGIN;

    if (gybing_round) {
        if (!want_other)
            gybing_round = false;
        else if (abs(heading_from_wind) > GYBE_FROM) {
            // far enough off the wind that the short way onto the other tack is through a gybe
            gybing_round = false;
            changeTack(false);
            return courseOn(world_wind, tack, wp_from_wind);
        } else
            return offWind(world_wind, tack, GYBE_FROM + 20);
    }

    if (!want_other) {
        tack_wanted_since = 0;
        return courseOn(world_wind, tack, wp_from_wind);
    }

    if (tack_wanted_since == 0)
        tack_wanted_since = millis();

    boolean settled = tack == current_tack && millis() - last_tack > MIN_TACK_INTERVAL;
    if (settled && (failed_tacks >= MAX_TACK_TRIES || millis() - tack_wanted_since > TACK_WAIT)) {
        logln(F("Can't tack. Going round by gybing."));
        tack_wanted_since = 0;
        gybing_round = true;
        return offWind(world_wind, tack, GYBE_FROM + 20);
    }

    // up at the edge of the no-go zone with way on: go through the wind, and come out at the same edge
    if (settled && making_way && gps_speed >= MIN_TACK_SPEED && abs(heading_from_wind) < IRONS + 2 * TACK_MARGIN) {
        tack_wanted_since = 0;
        changeTack(true);
        return offWind(world_wind, tack, IRONS + TACK_MARGIN);
    }

    // come up to the edge of the no-go zone, ready to tack
    return offWind(world_wind, tack, IRONS + TACK_MARGIN);
}

void adjustHeading() {
#ifndef NO_SAIL
    float world_wind = toCircleDeg(fused_heading + wind);

    // which side the wind is on. Head to wind, keep the last side we were sure of
    int16_t wind_from_bow = wind > 180 ? wind - 360 : wind;
    if (wind_from_bow > WIND_SIDE_MIN)
        current_tack = 1;
    else if (wind_from_bow < -WIND_SIDE_MIN)
        current_tack = -1;

    // a tack through the wind that got round without ending in irons resets the count of failed ones
    if (tacking_through && millis() - tack_started > MIN_TACK_INTERVAL) {
        tacking_through = false;
        if (current_tack == tack)
            failed_tacks = 0;
    }

    // back up to speed after a tack or irons: back to pointing as high as we can
    if (rebuilding && making_way && gps_speed >= MIN_TACK_SPEED && millis() - rebuild_since > RECOVER_TIME)
        rebuilding = false;

    // a tack or gybe gets MIN_TACK_INTERVAL to happen. After that, whichever tack we're on is the one we sail.
    if (tack != current_tack && millis() - last_tack > MIN_TACK_INTERVAL)
        tack = current_tack;

    if (recovering) {
        // in irons: fall off on the tack we're on and get going there
        tack = current_tack;
        was_beating = false;
        requested_heading = offWind(world_wind, tack, RECOVER_ANGLE);
    } else if (!haveFix()) {
        // no position yet: a beam reach on the tack we're on
        tack = current_tack;
        was_beating = false;
        requested_heading = offWind(world_wind, tack, 90);
    } else
        requested_heading = chooseCourse(world_wind);

    logln(F("World wind: %d.%d. Tack %d (on %d). Beating: %d. Off line %dm. Recovering: %d. Requested heading %d.%d"),
        FP(world_wind),
        tack,
        current_tack,
        was_beating,
        (int16_t) cross_track,
        recovering,
        FP(requested_heading));

    // drifting backwards, the rudder works the other way round
    boolean backwards = recovering && gps_speed > 0 && angleDiff(gps_course, fused_heading, false) > 90;
    steeringPID.SetControllerDirection(backwards ? REVERSE : DIRECT);

#else
    requested_heading = wp_heading;

    logln(F("Requested heading %d.%d, Actual heading %d.%d"),
    	  FP(requested_heading),
    	  FP(fused_heading));
#endif

    unwrapHeadings();

    if (tuningPID)
        autotune();
	else
	    steeringPID.Compute();

	rudderFromCenter(round(new_rudder));
}

void pilotInit(int16_t pilotSettingsAddress) {
    _pilotSettingsAddress = pilotSettingsAddress;

    centerRudder();
    centerWinch();

    wp_lat = wp_list[0];
    wp_lon = wp_list[1];

    // start the unwrapped heading where we are, so the first PID step doesn't see a jump from 0
    last_fused_heading = ahrs_heading;

	steeringPID.SetMode(AUTOMATIC);
	steeringPID.SetOutputLimits(-45, 45);
    
    pidTune.SetNoiseBand(aTuneNoise);
    pidTune.SetOutputStep(aTuneStep);
    pidTune.SetLookbackSec((int)aTuneLookBack);

	 // check if PID values are stored
    if ((char)EEPROM.read(_pilotSettingsAddress) == 'w') {
    	int addr = _pilotSettingsAddress + 1;
    	double kp, ki, kd;

    	EEPROM.get(addr, kp);
    	addr += sizeof(double);

    	EEPROM.get(addr, ki);
    	addr += sizeof(double);

    	EEPROM.get(addr, kd);

    	logln(F("Read stored PID tuning values of %d.%d, %d.%d, %d.%d"), FP(kp), FP(ki), FP(kd));
    	steeringPID.SetTunings(kp, ki, kd);
    }
}

void getCurrentPIDTunings(double* tuningsOut) {
    logln(F("Current PID tuning values are %d.%d, %d.%d, %d.%d"), FP(steeringPID.GetKp()), FP(steeringPID.GetKi()), FP(steeringPID.GetKd()));

    tuningsOut[0] = steeringPID.GetKp();
    tuningsOut[1] = steeringPID.GetKi();
    tuningsOut[2] = steeringPID.GetKd();
}

void updateCurrentPIDTunings(double* tunings) {
    steeringPID.SetTunings(tunings[0], tunings[1], tunings[2]);

    logln(F("New PID tuning values are %d.%d, %d.%d, %d.%d"), FP(steeringPID.GetKp()), FP(steeringPID.GetKi()), FP(steeringPID.GetKd()));
    int addr = _pilotSettingsAddress + 1;

    EEPROM.put(addr, tunings[0]);
    addr += sizeof(double);

    EEPROM.put(addr, tunings[1]);
    addr += sizeof(double);

    EEPROM.put(addr, tunings[2]);
    EEPROM.write(_pilotSettingsAddress, 'w');

    logln(F("New PID tunings stored."));
}

void setNextWaypoint() {
    logln(F("Waypoint %d reached."), target_wp);

    if ((target_wp == 0 && direction == -1) || (target_wp + 1 == WP_COUNT && direction == 1))
        direction *= -1;

    target_wp += direction;
    wp_lat = wp_list[target_wp * 2];
    wp_lon = wp_list[target_wp * 2 + 1];

    // wp changed, need to recompute
    wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
    wp_heading = DEG(toCircle(computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon))));
    startLeg();

    // we should allow tacks now, as this mechanism is just to space out zig zags
    // also can just zero it out as there's no inherent value in the time
    last_turn = 0;

    logln(F("Waypoint %d selected, HTW: %d, DTW: %dm"),
            target_wp,
            ((int16_t) wp_heading),
            ((int16_t) wp_distance));
}

void updateSituation() {
#ifndef NO_SAIL
    if (haveFix())
        updateStall();
#endif

    // still want to check this
    if (gps_speed > MIN_SPEED) {
        ahrs_offset = angleDiff(ahrs_heading, gps_course, true);
        logln(F("GPS vs AHRS difference is %d"), (int16_t) ahrs_offset * 10);
    }

    fuseHeading();

    // check if we've hit the waypoint
    wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
    wp_heading = DEG(toCircle(computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon))));
    adjustment_made = false;

    logln(F("GPS heading: %d, GPS speed (x10): %dkts, HTW: %d, DTW: %dm"),
            ((int16_t) gps_course),
            ((int16_t) (gps_speed * 10.0)),
            ((int16_t) wp_heading),
            ((int16_t) wp_distance));

    if (wp_distance < GET_WITHIN)
        setNextWaypoint();

    // the first leg starts at the first fix
    if (leg_lat == 0.0 && leg_lon == 0.0 && haveFix())
        startLeg();
    updateCrossTrack();

    if (wp_distance < HRG_THRESHOLD) {
        high_res_gps = true;
        warnGPS();
        logln(F("Within high-res gps threshold. Switching to HRG"));
    } else {
        if (!high_res_gps && HIGH_RES_GPS_DEFAULT)
            warnGPS();

        high_res_gps = HIGH_RES_GPS_DEFAULT;
    }
}

void doPilot() {
    if (manual_override) {
        processManualCommands();

        return;
    }

#ifdef NO_SAIL
  // run the motor
    if (gps_lat != 0.0 && gps_lon != 0.0)
        runMotor();
    else
        stopMotor();
#endif

    updateSituation();

	if (remote_control)
		processRCCommands();
	else {
		adjustHeading();
		adjustSails();
	}
}

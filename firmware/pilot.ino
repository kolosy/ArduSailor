#define NO_SAIL

#include <PID_v1.h>
#include <EEPROM.h>

// radius of Earth in m
#define R 6371000

// minimum speed needed to establish course (knots)
#define MIN_SPEED 1.0
#define MIN_TACK_SPEED 1.0

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

#define STALL_SPEED 0.5

// have sails at this position for a gybe
#define GYBE_SAIL_POS 80

#define SERVO_ORIENTATION -1

// how often to change tacks when beating up-wind
#define TACK_EVERY 30000

// if we're at more than this heel, start easing the mainsheet
#define START_HEEL_COMP 30
#define MAX_HEEL_COMP 30

// if heel compensation drops below this amount, zero it out
#define MIN_HEEL_COMP 5

// we're going decrease the amount of heel adjust by this factor to bring things back slowly
#define HEEL_COMP_DECAY 0.1

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
boolean stalled = true;

double new_rudder = 0;

int16_t _pilotSettingsAddress = 0;

// Specify the links and initial tuning parameters
PID steeringPID(&fused_heading, &new_rudder, &requested_heading, 0.1, 0.001, 2.8, P_ON_E, DIRECT);

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

void adjustSails() {
#ifdef NO_SAIL
    return;
#endif

    logln(F("Checking sail trim"));
    if (abs(current_roll) > START_HEEL_COMP)
        heel_adjust = min(MAX_HEEL_COMP, 2 * (abs(current_roll) - START_HEEL_COMP));
    else if (heel_adjust > MIN_HEEL_COMP)
        heel_adjust = heel_adjust * HEEL_COMP_DECAY;
    else
        heel_adjust = 0;

    float new_winch = map(constrain(abs(wind - 180), IRONS, 180), IRONS, 180, WINCH_MIN, WINCH_MAX) - heel_adjust;

    if (abs(new_winch - current_winch) > SAIL_ADJUST_ON) {
        logln(F("New winch position of %d is more than %d off from %d. Adjusting trim."), (int16_t) new_winch, SAIL_ADJUST_ON, current_winch);
        adjustment_made = true;
        winchTo(new_winch);
    } else
        logln(F("No trim adjustment needed"));
}

long time_since_tack_change = 0;
bool beat_to_port = false;
bool was_beating = false;

void adjustHeading() {
#ifndef NO_SAIL
	float world_wind = toCircleDeg(fused_heading + wind);

	// starbord tack: world wind + irons
	// port tack: world wind - irons

	// cyclomatic complexity is a tad high, but more readable this way
	if (angleDiff(world_wind, wp_heading, false) < IRONS) {
		if (was_beating) {
			if (time_since_tack_change + TACK_EVERY < millis()) {
				beat_to_port = !beat_to_port;
				time_since_tack_change = millis();
			}
		} else {
			was_beating = true;

			// pick the closer direction when we start beating
			// if sbord tack is farther, go to port
			beat_to_port = angleDiff(world_wind + IRONS, wp_heading, false) > angleDiff(world_wind - IRONS, wp_heading, false);
			time_since_tack_change = millis();
		}

		requested_heading = toCircleDeg(world_wind + (beat_to_port ? -IRONS : IRONS));
	} else {
		was_beating = false;

		requested_heading = wp_heading;
	}

	logln(F("World wind: %d.%d. Was beating: %d. Beat to port: %d, Time since change: %d. Requested heading %d.%d"),
		FP(world_wind),
		was_beating,
		beat_to_port,
		millis() - time_since_tack_change,
		FP(requested_heading));

#else
    requested_heading = wp_heading;
    
    logln(F("Requested heading %d.%d, Actual heading %d.%d"),
    	  FP(requested_heading),
    	  FP(fused_heading));
#endif

	// PID!!
	if (steeringPID.Compute())
		rudderFromCenter(round(new_rudder));
}

void pilotInit(int16_t pilotSettingsAddress) {
    _pilotSettingsAddress = pilotSettingsAddress;
    
    centerRudder();
    centerWinch();

    wp_lat = wp_list[0];
    wp_lon = wp_list[1];

	steeringPID.SetMode(AUTOMATIC);
	steeringPID.SetOutputLimits(-45, 45);

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

    // we should allow tacks now, as this mechanism is just to space out zig zags
    // also can just zero it out as there's no inherent value in the time
    last_turn = 0;

    logln(F("Waypoint %d selected, HTW: %d, DTW: %dm"),
            target_wp,
            ((int16_t) wp_heading),
            ((int16_t) wp_distance));
}

void updateSituation() {
    stalled = gps_speed < STALL_SPEED;

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

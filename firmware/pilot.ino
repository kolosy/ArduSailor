// radius of Earth in m
#define R 6371000

// minimum speed needed to establish course (knots)
#define MIN_SPEED 1.0

// our course!
#define WP1_LAT 41.923374
#define WP1_LON -87.631205

#define WP2_LAT 41.916709
#define WP2_LON -87.628885

// how close you have to get to the waypoint to consider it hit
#define GET_WITHIN 5

// how much to move tiller by when making slight adjustments
#define FEATHER 3

// adjust course when we're more than this much off-course
#define COURSE_ADJUST_ON 7

// adjust sails when we're more than this much off-plan
#define SAIL_ADJUST_ON 10

// either side of 0 for "in irons"
#define IRONS 35
#define IN_IRONS(v) (((v) < IRONS || (v) > (360 - IRONS)))

// either side of 180 for "running"
#define ON_RUN 20

// either side of 180 for must gybe / might gybe accidentally
#define GYBE_AT 10
#define MIGHT_GYBE(v) (((v) > 180 - GYBE_AT) && ((v) < 180 + GYBE_AT))

// straighten rudder when we make this much of a turn
#define TACK_START_STRAIGHT 50
#define GYBE_START_STRAIGHT 50

// have sails at this position for a gybe
#define GYBE_SAIL_POS 80

// if the course requires zig-zags, turn every x millis
#define TURN_EVERY 90000
#define CAN_TURN() ((last_turn + TURN_EVERY) < millis())
#define COURSE_CORRECTION_TIME 750

#define TO_PORT(amt) rudderTo(current_rudder + amt)
#define TO_SBRD(amt) rudderTo(current_rudder - amt)
#define RAD(v) ((v) * PI / 180.0)
#define DEG(v) ((v) * 180.0 / PI)

#define WINCH_MIN 60
#define WINCH_MAX 115

float wp_lat, wp_lon;
int target_wp = 1;
int last_turn = 0;

float wp_heading = 0;
float wp_distance = 0;

float ahrs_offset = 0;
uint8_t offset_set = 0;

// not real fusion for now
float fused_heading = 0;

inline void fuseHeading() {
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

// when this is called, we're assuming that we're close to irons, otherwise why tack?
void tack() {
    logln("Tacking...");
	float start_heading = fused_heading;

	// yank the tiller, sails are fine
	if (wind > 180) {
		// tacking to port
		// todo: what's the servo direction here?
		TO_PORT(25);
		while (!isPast(start_heading, TACK_START_STRAIGHT, fused_heading, false)) {
			updateSensors();
			fuseHeading();
		}
	} else {
		TO_SBRD(25);
		while (!isPast(start_heading, TACK_START_STRAIGHT, fused_heading, true)) {
			updateSensors();
			fuseHeading();
		}
	}
	
	centerRudder();
    logln("Finished tack");
	// sails are set as is
}

void gybe() {
    logln("Gybing...");

	float start_heading = fused_heading;
	int start_winch = current_winch;
	
	// move sail to safer position
	winchTo(GYBE_SAIL_POS);

	// yank the tiller, sails are fine
	if (wind > 180) {
		// gybe to starboard
		// todo: what's the servo direction here?
		TO_SBRD(25);
		while (!isPast(start_heading, GYBE_START_STRAIGHT, fused_heading, true)) {
			updateSensors();
			fuseHeading();
		}
	} else {
		TO_PORT(25);
		while (!isPast(start_heading, GYBE_START_STRAIGHT, fused_heading, false)) {
			updateSensors();
			fuseHeading();
		}
	}
	
	centerRudder();
	winchTo(start_winch);

    logln("Finished gybe");
}

void safetyCheck() {
	if (IN_IRONS(wind)) {
        logln("wind direction of %d has put us in irons. falling off...", wind);
		if (wind < 180) TO_PORT(15); else TO_SBRD(15);
		sleepMillis(1000);
		adjustment_made = true;
		updateSensors();
		fuseHeading();
	}
	
	// i don't think we really care about a gybe for our specific boat.
	
    //  else if (MIGHT_GYBE(wind)) {
    //  if (wind > 180) TO_PORT(15); else TO_SBRD(15);
    //  sleepMillis(1000);
    //  adjustment_made = true;
    //  updateSensors();
    //  fuseHeading();
    // }
}

// steer with counter steer. port is +, starboard is -
void steer_with_cs(int amount) {
    rudderTo(current_rudder + amount);
	delay(COURSE_CORRECTION_TIME);
	
	rudderTo(current_rudder - amount/2);
	delay(COURSE_CORRECTION_TIME/2);
	
    centerRudder();
}

void adjustSails() {
    logln("Checking sail trim");
	float new_winch = map(abs(wind - 180), 30, 170, WINCH_MIN, WINCH_MAX);
	
	if (abs(new_winch - current_winch) > SAIL_ADJUST_ON) {
        logln("New winch position of %d is more than %d off from %d. Adjusting trim.", (int16_t) new_winch, SAIL_ADJUST_ON, current_winch);
		adjustment_made = true;
		winchTo(new_winch);
	} else
        logln("No trim adjustment needed");
}

void adjustHeading() {
	// we wouldn't be here without a magnitude of error check. don't need one here.
	float off_course = angleDiff(fused_heading, wp_heading, true);
	
    logln("Off course by %d", (int16_t) off_course);
	
	int new_wind = DEG(toCircle(RAD(wind - off_course)));
    logln("When corrected, new wind direction will be %d", new_wind);
	
	if (IN_IRONS(new_wind) || MIGHT_GYBE(new_wind)) {
        logln("Requested course unsafe, requires tack/gybe");
		if (CAN_TURN()) {
            logln("Tack change allowed. Turning.");
			IN_IRONS(new_wind) ? tack() : gybe();
			last_turn = millis();
			adjustment_made = true;
		}
		else {
			if (off_course < 0)
			    new_wind = wind > 270 ? 360 - IRONS : 180 - ON_RUN;
			else
			    new_wind = wind < 90 ? IRONS : 180 + ON_RUN;

			float max_adjust = angleDiff(new_wind, wind, true);
            logln("Tack change not yet allowed. Adjusting by %d", (int16_t) -max_adjust);
            
			if (abs(max_adjust) > COURSE_ADJUST_ON) {
                steer_with_cs(-max_adjust);
                // if (max_adjust < 0) {
                //  TO_PORT(-max_adjust);
                //  delay(COURSE_CORRECTION_TIME);
                //  TO_SBRD(-max_adjust/2);
                //  delay(COURSE_CORRECTION_TIME/2);
                // }
                // else {
                //  TO_SBRD(max_adjust);
                //  delay(COURSE_CORRECTION_TIME);
                //  TO_PORT(max_adjust/2);
                //  delay(COURSE_CORRECTION_TIME/2);
                // }
				updateSensors();
				fuseHeading();
				adjustSails();

				adjustment_made = true;
			}
		}
	} else {
        logln("Adjusting by %d", (int16_t) -off_course);
        steer_with_cs(-off_course);
        // off_course < 0 ? TO_PORT(-off_course) : TO_SBRD(off_course);
        // delay(COURSE_CORRECTION_TIME);
        // // counter steer
        // off_course < 0 ? TO_PORT(off_course) : TO_SBRD(-off_course);
        // delay(COURSE_CORRECTION_TIME / 2);
		updateSensors();
		fuseHeading();
		adjustSails();
		adjustment_made = true;
	}
}

void pilotInit() {
	centerRudder();
	centerWinch();
	
	wp_lat = WP1_LAT;
	wp_lon = WP1_LON;
}

void doPilot() {
	// still want to check this
	if ((gps_speed > MIN_SPEED) && gps_updated) {
		ahrs_offset = angleDiff(ahrs_heading, gps_course, true);
        logln("GPS vs AHRS difference is %d", (int16_t) ahrs_offset * 10);
	}

	fuseHeading();		  
	
	// check if we've hit the waypoint
	wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
	wp_heading = toCircle(computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon))) * 180 / PI;
	adjustment_made = false;

    logln("GPS heading: %d, GPS speed (x10): %dkts, HTW: %d, DTW: %dm", 
            ((int16_t) gps_course), 
            ((int16_t) gps_speed * 10),
            ((int16_t) wp_heading), 
            ((int16_t) wp_distance));

	if (wp_distance < GET_WITHIN) {
		log("Waypoint reached.");
		if (target_wp == 1) {
			target_wp = 2;
			wp_lat = WP2_LAT;
			wp_lon = WP2_LON;
    		logln(" Setting waypoint 2.");
		} else {
			target_wp = 1;
			wp_lat = WP1_LAT;
			wp_lon = WP1_LON;
    		logln(" Setting waypoint 1.");
		}
	
		// wp changed, need to recompute
		wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
		wp_heading = computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon)) * 180 / PI;
	}

    safetyCheck();
    
	adjustSails();

	if (angleDiff(fused_heading, wp_heading, false) > COURSE_ADJUST_ON)
		adjustHeading();
}

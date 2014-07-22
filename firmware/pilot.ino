// radius of Earth in m
#define R 6371000

// our course!
#define WP1_LAT 41.923398
#define WP1_LON -87.631633

#define WP2_LAT 41.916709
#define WP2_LON -87.628885

// how close you have to get to the waypoint to consider it hit
#define GET_WITHIN 5

// how much to move tiller by when making slight adjustments
#define FEATHER 3

// adjust course when we're more than this much off-course
#define COURSE_ADJUST_ON 5

// adjust sails when we're more than this much off-plan
#define SAIL_ADJUST_ON 5

// either side of 0 for "in irons"
#define IRONS 30
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
#define CAN_TURN() (last_turn + TURN_EVERY < millis())
#define COURSE_CORRECTION_TIME 8000

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

// lat/lon and result in radians
float computeBearing(float i_lat, float i_lon, float f_lat, float f_lon) {
	float y = sin(f_lon-i_lon) * cos(f_lat);
	float x = cos(i_lat)*sin(f_lat) -
						sin(i_lat)*cos(f_lat)*cos(f_lon-i_lon);
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
	float start_heading = ahrs_heading;

	// yank the tiller, sails are fine
	if (wind > 180) {
		// tacking to port
		// todo: what's the servo direction here?
		TO_PORT(25);
		while (!isPast(start_heading, TACK_START_STRAIGHT, readSteadyHeading() * 180.0 / PI, false));
	} else {
		TO_SBRD(25);
		while (!isPast(start_heading, TACK_START_STRAIGHT, readSteadyHeading() * 180.0 / PI, true));
	}
	
	centerRudder();
	// sails are set as is
}

void gybe() {
	float start_heading = ahrs_heading;
	int start_winch = current_winch;
	
	// move sail to safer position
	winchTo(GYBE_SAIL_POS);

	// yank the tiller, sails are fine
	if (wind > 180) {
		// gybe to starboard
		// todo: what's the servo direction here?
		TO_SBRD(25);
		while (!isPast(start_heading, GYBE_START_STRAIGHT, readSteadyHeading() * 180.0 / PI, true));
	} else {
		TO_PORT(25);
		while (!isPast(start_heading, GYBE_START_STRAIGHT, readSteadyHeading() * 180.0 / PI, false));
	}
	
	centerRudder();
	winchTo(start_winch);
}

void safetyCheck() {
	return;
	
	if (IN_IRONS(wind)) {
		if (wind < 180) TO_PORT(15); else TO_SBRD(15);
		sleepMillis(1000);
		adjustment_made = true;
		updateSensors();
	} else if (MIGHT_GYBE(wind)) {
		if (wind > 180) TO_PORT(15); else TO_SBRD(15);
		sleepMillis(1000);
		adjustment_made = true;
		updateSensors();
	}
}

void adjustSails() {
	float new_winch = map(abs(wind - 180), 30, 170, WINCH_MIN, WINCH_MAX);
	
	if (abs(new_winch - current_winch) > SAIL_ADJUST_ON) {
		adjustment_made = true;
		winchTo(new_winch);
	}
}

void adjustHeading() {
	float off_course = angleDiff(ahrs_heading, wp_heading, true);
	
	int new_wind = DEG(toCircle(RAD(wind - off_course)));
	
	if (IN_IRONS(new_wind) || MIGHT_GYBE(new_wind)) {
		if (CAN_TURN()) {
			IN_IRONS(new_wind) ? tack() : gybe();
			last_turn = millis();
			adjustment_made = true;
		}
		else {
			if (off_course < 0) {
				if (wind > 270)
					new_wind = 360 - IRONS;
				else
					new_wind = 180 - ON_RUN;
			} else {
				if (wind < 90)
					new_wind = IRONS;
				else 
					new_wind = 180 + ON_RUN;
			}

			float max_adjust = angleDiff(new_wind, wind, true);
			if (abs(max_adjust) > COURSE_ADJUST_ON) {
				if (max_adjust < 0)
					TO_PORT(-max_adjust);
				else 
					TO_SBRD(max_adjust);
				adjustment_made = true;
			}
		}
	} else {
		off_course < 0 ? TO_PORT(2 * -off_course) : TO_SBRD(2 * off_course);
		sleepMillis(COURSE_CORRECTION_TIME);
		updateSensors();
		adjustSails();
		centerRudder();
		adjustment_made = true;
	}
}

void pilotInit() {
	centerRudder();
	centerWinch();
	
	wp_lat = WP1_LAT;
	wp_lon = WP1_LON;
}

// returns true when everything is good, false when another cycle is needed
void doPilot() {
	// check if we've hit the waypoint
	wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
	wp_heading = toCircle(computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon))) * 180 / PI;
	adjustment_made = false;
	
	Serial.print("Distance to waypoint: ");
	Serial.println(wp_distance, 2);
	
	Serial.print("Heading to waypoint: ");
	Serial.println(wp_heading, 2);
	
	if (wp_distance < GET_WITHIN) {
		Serial.println("Waypoint reached");
		if (target_wp == 1) {
			target_wp = 2;
			wp_lat = WP2_LAT;
			wp_lon = WP2_LON;
		} else {
			target_wp = 1;
			wp_lat = WP1_LAT;
			wp_lon = WP1_LON;
		}
		
		// wp changed, need to recompute
		wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
		wp_heading = computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon)) * 180 / PI;
	}
	
    // safetyCheck();
    // 
	adjustSails();
	
	if (angleDiff(ahrs_heading, wp_heading, false) > COURSE_ADJUST_ON)
		adjustHeading();
}

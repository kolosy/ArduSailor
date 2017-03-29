// radius of Earth in m
#define R 6371000

// minimum speed needed to establish course (knots)
#define MIN_SPEED 1.0
#define MIN_TACK_SPEED 1.0

// how close you have to get to the waypoint to consider it hit
#define GET_WITHIN 5

// how much to move tiller by when making slight adjustments
#define FEATHER 5.0

// how to equate two requested rudder positions (i.e. if the new one is less than this from the old one, don't do it)
#define RUDDER_TOLERANCE 1

// adjust course when we're more than this much off-course
#define COURSE_ADJUST_ON 7.0

#define COURSE_ADJUST_SPEED 7.0

// adjust the rudder by this amount to end up at turn of v
#define ADJUST_BY(v) ((v/(FEATHER * COURSE_ADJUST_SPEED)+1.) * FEATHER)

// adjust sails when we're more than this much off-plan
#define SAIL_ADJUST_ON 10

// when we're turning, how far ahead are we looking to check if we're going to fall in irons
#define WIND_LOOKAHEAD 10

// either side of 0 for "in irons"
#define IRONS 45
#define IN_IRONS(v) (((v) < IRONS || (v) > (360 - IRONS)))
#define TACK_TIMEOUT 10000

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

// if the course requires zig-zags, turn every x millis
#define TURN_EVERY 90000
#define CAN_TURN() ((last_turn + TURN_EVERY) < millis())
#define COURSE_CORRECTION_TIME 3000

#define SERVO_ORIENTATION -1
#define TO_PORT(amt) rudderTo(current_rudder + (SERVO_ORIENTATION * amt))
#define TO_SBRD(amt) rudderTo(current_rudder - (SERVO_ORIENTATION * amt))

#define ADJUST_TO_PORT(amt) ( SERVO_ORIENTATION * amt)
#define ADJUST_TO_SBRD(amt) (-SERVO_ORIENTATION * amt)

#define RAD(v) ((v) * PI / 180.0)
#define DEG(v) ((v) * 180.0 / PI)

// if we're at more than this heel, start easing the mainsheet
#define START_HEEL_COMP 30
#define MAX_HEEL_COMP 30

// if heel compensation drops below this amount, zero it out
#define MIN_HEEL_COMP 5

// we're going decrease the amount of heel adjust by this factor to bring things back slowly
#define HEEL_COMP_DECAY 0.1

// # of waypoints below
#define WP_COUNT 7

// how close we can get to our waypoint before we switch to High Res GPS
#define HRG_THRESHOLD 50

// how long to wait for a complete RC command before we ditch it
#define RC_TIMEOUT 10000

// lat,lon pairs
float wp_list[] = 
  {
    41.923015, -87.631082,
    41.9207923576838 ,-87.63011004661024,
    41.92040355439754,-87.63045132003815,
    41.92024341159895,-87.63004258543924,
    41.91948366250413,-87.62996748502381,
    41.91848868180579,-87.62960748575088,
    41.91771524159618,-87.62928507512221,
    41.91665396400703,-87.62889035219517 // end of circuit
  };
int8_t direction = 1;

// current lat,lon
float wp_lat, wp_lon;
int target_wp = 0;

uint32_t last_turn = 0;

float ahrs_offset = 0;
uint8_t offset_set = 0;

// not real fusion for now
float fused_heading = 0;

int16_t turning_by = 0;
boolean turning = false;
boolean tacking = false;
boolean stalled = true;

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

// when this is called, we're assuming that we're close to irons, otherwise why tack?
void tack() {
    logln(F("Tacking..."));
    float start_heading = fused_heading;
    uint32_t tack_start_time = millis();

    // yank the tiller, sails are fine
    if (wind > 180) {
        // tacking to port
        // todo: what's the servo direction here?
        TO_PORT(25);
        while (!isPast(start_heading, TACK_START_STRAIGHT, fused_heading, false) && ((millis() - tack_start_time) < TACK_TIMEOUT)) {
            updateSensors(true);
            fuseHeading();
        }
    } else {
        TO_SBRD(25);
        while (!isPast(start_heading, TACK_START_STRAIGHT, fused_heading, true) && ((millis() - tack_start_time) < TACK_TIMEOUT)) {
            updateSensors(true);
            fuseHeading();
        }
    }
    
    centerRudder();
    logln(F("Finished tack"));
    // sails are set as is
}

void adjustTo(int amount) {
    int corrected = amount * SERVO_ORIENTATION;
    turning_by = amount;
    turning = true;
    
    rudderFromCenter(corrected);
}

void adjustSails() {
    logln(F("Checking sail trim"));
    if (abs(current_roll) > START_HEEL_COMP)
      heel_adjust = min(MAX_HEEL_COMP, 2 * (abs(current_roll) - START_HEEL_COMP));
    else if (heel_adjust > MIN_HEEL_COMP)
      heel_adjust = heel_adjust * HEEL_COMP_DECAY;
    else
      heel_adjust = 0;

    float new_winch = map(abs(wind - 180), 30, 170, WINCH_MIN, WINCH_MAX) - heel_adjust;
    
    if (abs(new_winch - current_winch) > SAIL_ADJUST_ON) {
        logln(F("New winch position of %d is more than %d off from %d. Adjusting trim."), (int16_t) new_winch, SAIL_ADJUST_ON, current_winch);
        adjustment_made = true;
        winchTo(new_winch);
    } else
        logln(F("No trim adjustment needed"));
}

void adjustHeading() {
    float off_course = angleDiff(fused_heading, wp_heading, true);
    boolean skip_irons_check = false;

    // + off_course == will turn to starboard
    // rudder values: 
    //       90: center. 
    //      100: turning to sbrd, pointing to port
    //       80: turning to port, pointing to sbrd
    
//    if (stalled) {
//      off_course = -(wind > 180 ? 270.0 - wind : 90.0 - wind);
//      skip_irons_check = true;
//      logln(F("Stalled. Setting course to beam reach."));
//    } else if (IN_IRONS(wind)) {
//      // wind > 180 == port tack, want to turn to starboard to fix
//      // wind < 180 == starboard tack, want to turn to port to fix
//      off_course = -(wind > 180 ? 360 - IRONS - wind : IRONS - wind);
//      skip_irons_check = true;
//      logln(F("In irons. Setting course for close reach."));
//    }
    
    logln(F("Off course by %d"), (int16_t) off_course);
    if (turning) 
      logln(F("Currently turning by %d, rudder is at %d"), turning_by, current_rudder);
    else
      logln(F("Not currently turning, ruder is at %d"), current_rudder);
    
    if (fabs(off_course) > COURSE_ADJUST_ON) {
        logln(F("Current course is more than %d off target. Trying to adjust"), COURSE_ADJUST_ON);

        // new wind if we turn the direction that we want to. The +/- is to account for the fact that 
        // rotated rudder will keep us turning
        int new_wind = toCircleDeg(wind - (off_course < 0 ? -WIND_LOOKAHEAD:WIND_LOOKAHEAD));
        
        // if adjusting any more puts us in irons
        if (!skip_irons_check && IN_IRONS(new_wind)) {
            logln(F("Requested (new wind %d) course unsafe, requires tack"), new_wind);
            // and if we're allowed to tack, we tack
            if (CAN_TURN() && gps_speed > MIN_TACK_SPEED) {
                logln(F("Tack change allowed. Turning."));
                tack();
                last_turn = millis();
                
                // tack() centers the rudder, and we should re-evaluate where we are at that point.
                turning_by = 0;
                turning = false;
                adjustment_made = true;
            // if we aren't allowed to tack, but are currently turning, we stop turning
            } else if (turning) {
                logln(F("Tack change not allowed. Currently in a turn, centering rudder."));
                turning = false;
                turning_by = 0;
                adjustment_made = true;
                centerRudder();
            }
        } else {
            // todo: there's something in ADJUST_BY that's misbehaving with negative off-course values. being lazy here.
            logln(F("Projected new wind is %d"), new_wind);
            int new_rudder = ADJUST_BY(abs(off_course)) * (off_course >= 0 ? -1 : 1);
            if (abs(new_rudder - turning_by) > RUDDER_TOLERANCE) {
                logln(F("Adjusting rudder by %d"), new_rudder);
                adjustTo(new_rudder);
                adjustment_made = true;
            } else
              logln(F("New rudder position of %d is close to current rudder position of %d. Not adjusting"), new_rudder, turning_by);
        }
    } else if (turning) {
        logln(F("Current course is not more than %d off target. Centering."), COURSE_ADJUST_ON);
        turning = false;
        turning_by = 0;
        adjustment_made = true;
        centerRudder();
    }
}

void pilotInit() {
    centerRudder();
    centerWinch();
    
    wp_lat = wp_list[0];
    wp_lon = wp_list[1];
}

void resetRudder() {
    turning = false;
    turning_by = 0;
    adjustment_made = false;
    centerRudder();
}

void processManualCommands() {
    while (Serial.available()) {
        switch ((char)Serial.read()) {
            case 'i':
                updateSensors(false);
                break;
            case 'a':
                TO_PORT(10);
                logln(F("10 degrees to port"));
                break;
            case 'd':
                TO_SBRD(10);
                logln(F("10 degrees to starboard"));
                break;
            case 's':
                centerRudder();
                logln(F("Center rudder"));
                break;
            case 'q':
                winchTo(current_winch + 5);
                logln(F("Sheet out"));
                break;
            case 'e':
                winchTo(current_winch - 5);
                logln(F("Sheet in"));
                break;
            case 'w':
                centerWinch();
                logln(F("Center winch"));
                break;
            case 'x':
                manual_override = false;
				serial_logging = SERIAL_LOGGING_DEFAULT;
                resetRudder();
                logln(F("End manual override"));
                break;
        }
    }
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

void getOutOfIrons() {
  // - roll == rolling to sbord
  // + roll == rolling to port
  
  centerWinch();
  
  rudderFromCenter(current_roll < 0 ? ADJUST_TO_PORT(10) : ADJUST_TO_SBRD(10));
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
    }
    else {
      if (!high_res_gps && HIGH_RES_GPS_DEFAULT)
        warnGPS();
        
      high_res_gps = HIGH_RES_GPS_DEFAULT;
    }
}

void processRCCommands() {
  if (!Serial.available())
    return;
  
  char c = 0;
  
  // read out the buffer until we get a command start
  while (Serial.available() && c != '[')
    c = (char)Serial.read();
    
  // command format : "[RRR;WW]" where RR is a signed two digit rudder position and WW is a two-digit winch position

  if (c != '[')
    return;

  int rudder = Serial.parseInt();
  int winch = Serial.parseInt();

  rudderFromCenter(rudder);
  normalizedWinchTo(winch);  
}

void doPilot() {
    if (manual_override) {
        processManualCommands();        
        
        return;
    }

    updateSituation();    

	if (remote_control)
		processRCCommands();
	else {
	    if (IN_IRONS(wind) && stalled) {
	      getOutOfIrons();
	    } else {
	      adjustHeading();
	      adjustSails();
	    }
	}
}

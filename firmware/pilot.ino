// radius of Earth in m
#define R 6371000

// minimum speed needed to establish course (knots)
#define MIN_SPEED 1.0

// how close you have to get to the waypoint to consider it hit
#define GET_WITHIN 25

// how much to move tiller by when making slight adjustments
#define FEATHER 5

// adjust course when we're more than this much off-course
#define COURSE_ADJUST_ON 7

#define COURSE_ADJUST_SPEED 7

// adjust the rudder by this amount to end up at turn of v
#define ADJUST_BY(v) ((v/(FEATHER * COURSE_ADJUST_SPEED)+1) * FEATHER)

// adjust sails when we're more than this much off-plan
#define SAIL_ADJUST_ON 10

// either side of 0 for "in irons"
#define IRONS 35
#define IN_IRONS(v) (((v) < IRONS || (v) > (360 - IRONS)))
#define TACK_TIMEOUT 30000

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
#define COURSE_CORRECTION_TIME 3000

#define SERVO_ORIENTATION -1
#define TO_PORT(amt) rudderTo(current_rudder + (SERVO_ORIENTATION * amt))
#define TO_SBRD(amt) rudderTo(current_rudder - (SERVO_ORIENTATION * amt))
#define RAD(v) ((v) * PI / 180.0)
#define DEG(v) ((v) * 180.0 / PI)

#define WINCH_MIN 60
#define WINCH_MAX 115

// # of waypoints below
#define WP_COUNT 2

// lat,lon pairs
float wp_list[] = {41.923374, -87.631205, 41.916709, -87.628885};
int8_t direction = 1;

// current lat,lon
float wp_lat, wp_lon;
int target_wp = 0;

uint32_t last_turn = 0;

float wp_heading = 0;
float wp_distance = 0;

float ahrs_offset = 0;
uint8_t offset_set = 0;

// not real fusion for now
float fused_heading = 0;

int16_t turning_by = 0;
boolean turning = false;
boolean tacking = false;

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
    logln("Tacking...");
    float start_heading = fused_heading;
    uint32_t tack_start_time = millis();

    // yank the tiller, sails are fine
    if (wind > 180) {
        // tacking to port
        // todo: what's the servo direction here?
        TO_PORT(25);
        while (!isPast(start_heading, TACK_START_STRAIGHT, fused_heading, false) && ((millis() - tack_start_time) < TACK_TIMEOUT)) {
            updateSensors();
            fuseHeading();
        }
    } else {
        TO_SBRD(25);
        while (!isPast(start_heading, TACK_START_STRAIGHT, fused_heading, true) && ((millis() - tack_start_time) < TACK_TIMEOUT)) {
            updateSensors();
            fuseHeading();
        }
    }
    
    centerRudder();
    logln("Finished tack");
    // sails are set as is
}

// void gybe() {
//     logln("Gybing...");
// 
//     float start_heading = fused_heading;
//     int start_winch = current_winch;
//     
//     // move sail to safer position
//     winchTo(GYBE_SAIL_POS);
// 
//     // yank the tiller, sails are fine
//     if (wind > 180) {
//         // gybe to starboard
//         TO_SBRD(25);
//         while (!isPast(start_heading, GYBE_START_STRAIGHT, fused_heading, true)) {
//             updateSensors();
//             fuseHeading();
//         }
//     } else {
//         TO_PORT(25);
//         while (!isPast(start_heading, GYBE_START_STRAIGHT, fused_heading, false)) {
//             updateSensors();
//             fuseHeading();
//         }
//     }
//     
//     centerRudder();
//     winchTo(start_winch);
// 
//     logln("Finished gybe");
// }
// 
// void safetyCheck() {
//     if (IN_IRONS(wind)) {
//         logln("wind direction of %d has put us in irons. falling off...", wind);
//         // todo: this should be a steer_with_cs call
//         if (wind < 180) TO_PORT(15); else TO_SBRD(15);
//         sleepMillis(1000);
//         adjustment_made = true;
//         updateSensors();
//         fuseHeading();
//     }
//     
//     // i don't think we really care about a gybe for our specific boat.
//     
//     //  else if (MIGHT_GYBE(wind)) {
//     //  if (wind > 180) TO_PORT(15); else TO_SBRD(15);
//     //  sleepMillis(1000);
//     //  adjustment_made = true;
//     //  updateSensors();
//     //  fuseHeading();
//     // }
//     
//     centerRudder();
// }

// // steer with counter steer. port is +, starboard is -
// void steer_with_cs(int amount) {
//     int corrected = amount * SERVO_ORIENTATION;
//     
//     int c_rudder = current_rudder;
//     
//     rudderTo(c_rudder + corrected);
//     delay(COURSE_CORRECTION_TIME);
//     
//     rudderTo(c_rudder - corrected/2);
//     delay(COURSE_CORRECTION_TIME/4);
//     
//     rudderTo(c_rudder);
// }

void adjustTo(int amount) {
    int corrected = amount * SERVO_ORIENTATION;
    turning_by = amount;
    turning = true;
    
    rudderFromCenter(corrected);
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
    float off_course = angleDiff(fused_heading, wp_heading, true);
    
    logln("Off course by %d", (int16_t) off_course);
    if (turning) 
      logln("Currently turning by %d, rudder is at %d", turning_by, current_rudder);
    else
      logln("Not currently turning, ruder is at %d", current_rudder);
    
    if (fabs(off_course) > COURSE_ADJUST_ON) {
        logln("Current course is more than %d off target. Trying to adjust", COURSE_ADJUST_ON);

        // new wind if we turn the direction that we want to
        int new_wind = DEG(toCircle(RAD(wind - (off_course < 0 ? -1:1))));
        
        // if adjusting any more puts us in irons
        if (IN_IRONS(new_wind)) {
            logln("Requested course unsafe, requires tack/gybe");
            // and if we're allowed to tack, we tack
            if (CAN_TURN()) {
                logln("Tack change allowed. Turning.");
                tack();
                last_turn = millis();
                adjustment_made = true;
            // if we aren't allowed to tack, but are currently turning, we stop turning
            } else if (turning) {
                logln("Tack change not allowed. Currently in a turn, centering rudder.");
                turning = false;
                turning_by = 0;
                adjustment_made = true;
                centerRudder();
            }
        } else {
            int new_rudder = ADJUST_BY(-off_course);
            if (new_rudder != turning_by) {
                logln("Adjusting rudder by %d", new_rudder);
                adjustTo(new_rudder);
                adjustment_made = true;
            }
        }
    } else if (turning) {
        logln("Current course is not more than %d off target. Centering.", COURSE_ADJUST_ON);
        turning = false;
        turning_by = 0;
        adjustment_made = true;
        centerRudder();
    }
}

// void adjustHeading() {
//     // we wouldn't be here without a magnitude of error check. don't need one here.
//     float off_course = angleDiff(fused_heading, wp_heading, true);
//     
//     logln("Off course by %d", (int16_t) off_course);
//     
//     int new_wind = DEG(toCircle(RAD(wind - off_course)));
//     
//     logln("When corrected, new wind direction will be %d", new_wind);
//     
//     if (IN_IRONS(new_wind) || MIGHT_GYBE(new_wind)) {
//         logln("Requested course unsafe, requires tack/gybe");
//         if (CAN_TURN()) {
//             logln("Tack change allowed. Turning.");
//             IN_IRONS(new_wind) ? tack() : gybe();
//             last_turn = millis();
//             adjustment_made = true;
//         }
//         else {
//             if (off_course < 0)
//                 new_wind = wind > 270 ? 360 - IRONS : 180 - ON_RUN;
//             else
//                 new_wind = wind < 90 ? IRONS : 180 + ON_RUN;
// 
//             float max_adjust = angleDiff(new_wind, wind, true);
//             logln("Tack change not yet allowed. Adjusting by %d", (int16_t) -max_adjust);
//             
//             if (abs(max_adjust) > COURSE_ADJUST_ON) {
//                 int new_rudder = ADJUST_BY(max_adjust);
//                 if (new_rudder != turning_by) {
//                     turning_by = new_rudder;
//                     adjust_by(turning_by);
// 
//                     adjustment_made = true;
//                     turning = true;
//                 }
//             }
//         }
//     } else {
//         logln("Adjusting by %d", (int16_t) -off_course);
//         steer_with_cs(-off_course);
//         updateSensors();
//         fuseHeading();
//         adjustSails();
//         adjustment_made = true;
//     }
// }

void pilotInit() {
    centerRudder();
    centerWinch();
    
    wp_lat = wp_list[0];
    wp_lon = wp_list[1];
}

void processManualCommands() {
    while (Serial.available()) {
        switch ((char)Serial.read()) {
            case 'i':
                updateSensors();
                break;
            case 'a':
                TO_PORT(10);
                logln("10 degrees to port");
                break;
            case 'd':
                TO_SBRD(10);
                logln("10 degrees to starboard");
                break;
            case 's':
                centerRudder();
                logln("Center rudder");
                break;
            case 'q':
                winchTo(current_winch + 5);
                logln("Sheet out");
                break;
            case 'e':
                winchTo(current_winch - 5);
                logln("Sheet in");
                break;
            case 'w':
                centerWinch();
                logln("Center winch");
                break;
            case 'x':
                manual_override = false;
                logln("End manual override");
                break;
        }
    }
}

void setNextWaypoint() {
    logln("Waypoint %d reached.", target_wp);
    
    if ((target_wp == 0 && direction == -1) || (target_wp + 1 == WP_COUNT && direction == 1))
        direction *= -1;
        
    target_wp += direction;
    wp_lat = wp_list[target_wp * 2];
    wp_lon = wp_list[target_wp * 2 + 1];
    
    // wp changed, need to recompute
    wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
    wp_heading = computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon)) * 180 / PI;

    logln("Waypoint %d selected, HTW: %d, DTW: %dm", 
            target_wp,
            ((int16_t) wp_heading), 
            ((int16_t) wp_distance));
}

void doPilot() {
    if (manual_override) {
        processManualCommands();        
        
        return;
    }

    // still want to check this
    if ((gps_speed > MIN_SPEED) && gps_updated) {
        ahrs_offset = angleDiff(ahrs_heading, gps_course, true);
        logln("GPS vs AHRS difference is %d", (int16_t) ahrs_offset * 10);
    }

    fuseHeading();        
    
    // check if we've hit the waypoint
    wp_distance = computeDistance(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon));
    wp_heading = DEG(toCircle(computeBearing(RAD(gps_lat), RAD(gps_lon), RAD(wp_lat), RAD(wp_lon))));
    adjustment_made = false;

    logln("GPS heading: %d, GPS speed (x10): %dkts, HTW: %d, DTW: %dm", 
            ((int16_t) gps_course), 
            ((int16_t) (gps_speed * 10.0)),
            ((int16_t) wp_heading), 
            ((int16_t) wp_distance));

    if (wp_distance < GET_WITHIN)
        setNextWaypoint();

    adjustHeading();
    adjustSails();
}

#ifndef __config_h
#define __config_h

// Build switches, in one place so every file agrees. For the sailing build, comment out
// PILOT_DEBUG and NO_SAIL.

// bench-test the pilot without sensors: the compass is replaced by a simulator that turns with
// the rudder, and the wind vane always reads 0
#define PILOT_DEBUG

// motor-boat mode: power a motor from the winch channel and steer straight to each waypoint,
// skipping the sailing logic and sail trim
#define NO_SAIL

// log to serial only, not to the SD card
#define NO_SD

// magnetic inclination (dip) at the sailing site in degrees, positive in the northern hemisphere. A flat
// compass calibration uses it to work out the vertical offset. NOAA's World Magnetic Model (WMM2025)
// gives 68.4 at the pilot.ino waypoints in Chicago; look up your own site if you sail elsewhere.
#define MAG_INCLINATION 68.4

#endif

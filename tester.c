#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "firmware/trig_fix.h"

#define PI 3.14159265359
#define IRONS 40
#define TRUE 1
#define FALSE 0

// if sign is true, then the value returned will be signed. the sign will be negative if a1 is > a2
float angleDiff(float a1, float a2, uint8_t sign) {
  // 232, 163
    float larger, smaller, mult;
    if (a1 > a2) {
      larger = a1;
      smaller = a2;
          mult = -1;
    } else {
      larger = a2;
      smaller = a1;
          mult = 1;
    }
    // 69
    float d = larger - smaller;
    if (d > 180.0) {
        d = 360.0 - d;
        mult *= -1;
    }

    if (!sign)
        mult = 1.0;

    return mult * d;
}

float toCircle(float value) {
    if (value > 2.0 * PI)
        return value - (2.0 * PI);
    if (value < 0)
      return value + 2.0 * PI;
        
    return value;
}

float toCircleDeg(float value) {
    if (value > 360.0)
        return value - 360.0;
    if (value < 0)
      return value + 360.0;
        
    return value;
}

uint8_t isPast(int start, int amount, int check, uint8_t clockwise) {
    float v = angleDiff(start+amount, check, TRUE);
    
    if (clockwise)
        return v > amount;
    
    return -v > amount;
}

int main (int argc, char* argv[]) {
	int mode = atoi(argv[1]);
	
	switch (mode) {
		case 0: {
			float v = atof(argv[2]);
			printf("transformed is %d. cos(%f) == %f with transform. native is %f\r\n", TO_FIXED(v), v, cos_fix(v), cos(v));
			break;
		}
		case 1: {
			uint16_t v = atoi(argv[2]);
			printf("cos(%d) == %d without transform\r\n", v, _cos_fix(v));
			break;
		}
		case 2: {
			float v = atof(argv[2]);
			printf("transformed is %d. sin(%f) == %f with transform. native is %f\r\n", TO_FIXED(v), v, sin_fix(v), sin(v));
			break;
		}
		case 3: {
			uint16_t v = atoi(argv[2]);
			printf("sin(%d) == %d without transform\r\n", v, _sin_fix(v));
			break;
		}
		case 4: {
			float v = atof(argv[2]);
			float v2 = atof(argv[3]);
			printf("atan2(%f, %f) == %f. native is %f\r\n", v, v2, atan2_fix(v, v2), atan2(v, v2));
			break;
		}
	}
}

//
//
//
// int main(int argc, char* argv[]) {
// 	if (argc != 4)
// 		return -1;
//
// 	float heading = atof(argv[1]);
// 	float wind = atof(argv[2]);
// 	float wp_heading = atof(argv[3]);
//
// 	float world_wind = toCircleDeg(heading + wind);
// 	float irons_check = angleDiff(world_wind, wp_heading, FALSE);
// 	uint8_t beat_to_port = angleDiff(world_wind + IRONS, wp_heading, FALSE) > angleDiff(world_wind - IRONS, wp_heading, FALSE);
//
// 	float requested_heading = toCircleDeg(world_wind + (beat_to_port ? -IRONS : IRONS));
//
// 	printf("%0.00f, %0.00f, %0.00f, %0.00f, %0.00f, %d, %0.00f\n", heading, wind, wp_heading, world_wind, irons_check, beat_to_port, requested_heading);
//
// 	return 0;
// }
//







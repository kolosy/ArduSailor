#define WS1 1
#define WS2 0

#define SIGN_SHIFT 500
#define SENSOR_OFFSET 209.0

#define WIND_ITERATIONS 50

float readSteadyWind() {
	uint16_t ws1 = 0;
	uint16_t ws2 = 0;
	
	for (int i=0; i<WIND_ITERATIONS; i++) {
		ws1 += analogRead(WS1);
		ws2 += analogRead(WS2);
		delay(1);
	}

	return toCircle(-atan2(ws1 / ((float) WIND_ITERATIONS) - SIGN_SHIFT, ws2 / ((float) WIND_ITERATIONS) - SIGN_SHIFT) - (SENSOR_OFFSET * PI / 180.0) + PI);
}

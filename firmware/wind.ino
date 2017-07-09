#define WS1 8
#define WS2 9
#define WIND_EN 33 // p-channel

#define SIGN_SHIFT 500
#define SENSOR_OFFSET 180.0

#define WIND_ITERATIONS 2
#define WIND_PAUSES 5

#include "trig_fix.h"

void windInit() {
    pinMode(WIND_EN, OUTPUT);
    digitalWrite(WIND_EN, HIGH);
}

float readSteadyWind() {
    digitalWrite(WIND_EN, LOW);
    delay(50);

    uint16_t ws1 = 0;
    uint16_t ws2 = 0;
    
    for (int i=0; i<WIND_ITERATIONS; i++) {
        ws1 += analogRead(WS1);
        ws2 += analogRead(WS2);
        delay(WIND_PAUSES);
    }

    digitalWrite(WIND_EN, HIGH);

    return toCircle(-atan2(ws1 / ((float) WIND_ITERATIONS) - SIGN_SHIFT, ws2 / ((float) WIND_ITERATIONS) - SIGN_SHIFT) - (SENSOR_OFFSET * PI / 180.0) + PI);
}

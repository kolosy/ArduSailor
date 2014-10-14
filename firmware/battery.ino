#define R1 324000.0
#define R2 100000.0
#define BATT_V_EN 34 // n-channel
#define BATT_VAL 10

void batteryInit() {
    pinMode(BATT_V_EN, OUTPUT);
    digitalWrite(BATT_V_EN, LOW);
}

float measureVoltage() {
    digitalWrite(BATT_V_EN, HIGH);
    delay(50);

    float voltage = ((float)analogRead(BATT_VAL)) / 1023.0 * 3.3;

    digitalWrite(BATT_V_EN, HIGH);
    return voltage / R2 * (R1+R2);
}

float toCircle(float value) {
	if (value > 2.0 * PI)
		return value - (2.0 * PI);
	if (value < 0)
	  return value + 2.0 * PI;
		
	return value;
}

boolean isPast(int start, int amount, int check, boolean clockwise) {
	float v = angleDiff(start+amount, check, true);
	
	if (clockwise)
		return v > amount;
	
	return -v > amount;
}

// if sign is true, then the value returned will be signed. the sign will be negative if a1 is > a2
float angleDiff(float a1, float a2, boolean sign) {
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
	
	float d = larger - smaller;
	if (d > 180.0) {
		d = 360.0 - d;
		mult *= -1;
	}

	if (!sign)
		mult = 1.0;

	return mult * d;
}

void sleepMillis(int amount) {
	int sleeps = amount / 2000; // max sleep time is 2s, so this is the number of times we'll have to sleep

	while (sleeps-- > 0)
		LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
}

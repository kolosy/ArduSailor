// seconds
#define MENU_TIMEOUT 10

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

void checkInput() {
    // in case we're going too fast. only needed when serial_logging is on
    if (serial_logging && (cycle % WAIT_FOR_COMMAND_EVERY == 0))
        delay(WAIT_FOR_COMMAND_FOR);

    while (Serial.available()) {
        // a '[' signals the start of an RC command
        if (remote_control && (char)Serial.peek() == '[')
            return;

        switch ((char)Serial.read()) {
            case 'o':
            logln(F("Entering manual override"));
            manual_override = true;
            serial_logging = true;
            break;

            case 'l':
            serial_logging = !serial_logging;
            break;

            case 'm':
            doMenu();
            break;
        }
    }
}

void processManualCommands() {
    while (Serial.available()) {
        switch ((char)Serial.read()) {
            case 'i':
                updateSensors(false);
                break;
            case 'a':
                toPort(10);
                logln(F("10 degrees to port"));
                break;
            case 'd':
                toSbord(10);
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
                logln(F("End manual override"));
                break;
            #ifdef NO_SAIL
            case 'm':
                runMotor();
                logln(F("Motor on"));
                break;
            case '.':
                stopMotor();
                logln(F("Motor off"));
                break;
            #endif
        }
    }
}

void getPIDTunings() {
    bool current_sl = serial_logging;
    serial_logging = true;

    double tunings[3];
    getCurrentPIDTunings(tunings);

    logln(F("Current PID tuning values are %d.%d, %d.%d, %d.%d"), FP(tunings[0]), FP(tunings[1]), FP(tunings[2]));

    Serial.println(F("Enter tunings:"));

    Serial.print(F("Kp: "));
    if (!waitForData(5000))
        return;

    tunings[0] = Serial.parseFloat();
    Serial.println(tunings[0], 4);

    Serial.print(F("Ki: "));
    if (!waitForData(5000))
        return;

    tunings[1] = Serial.parseFloat();
    Serial.println(tunings[1], 4);

    Serial.print(F("Kd: "));
    if (!waitForData(5000))
        return;

    tunings[2] = Serial.parseFloat();
    Serial.println(tunings[2], 4);

    updateCurrentPIDTunings(tunings);

    serial_logging = current_sl;
}

void doMenu() {
    Serial.print(F("Welcome to ArduSailor. Menu timeout is "));
    Serial.println(MENU_TIMEOUT);
    Serial.println();

    Serial.println("Current configuration is:");
    Serial.print(F("remote control: ")); Serial.println(remote_control);
    Serial.print(F("manual override: ")); Serial.println(manual_override);

    Serial.println(F("\nPlease select a menu option. \n"));

    Serial.println(F("(a) Automate."));
    Serial.println(F("(o) Auto-calibrate compass."));
    Serial.println(F("(c) Calibrate compass."));
    Serial.println(F("(r) Remote control."));
    Serial.println(F("(w) Change waypoints <NOT IMPLEMENTED>."));
    Serial.println(F("(t) Tune PID."));
    Serial.println(F("(y) Auto-tune PID."));
    Serial.println(F("(u) Stop PID auto-tune."));
    Serial.println(F("(m) Set mag offset."));
    Serial.print(F("\n>"));

    long t = millis();

    while ((millis() - t < (MENU_TIMEOUT * 1000)) && !Serial.available());

    if (Serial.available()) {
        char c = (char) Serial.read();
        switch (c) {
            case 'a':
            remote_control = false;
            manual_override = false;
            break;

            case 'c':
            calibrateMag(true);
            break;

            case 'o':
            runMotor();
            rudderFromCenter(30);
            calibrateMag(false);
            stopMotor();
            centerRudder();
            break;

            case 'r':
            remote_control = true;
            manual_override = false;
            break;

            case 'w':
            // todo
            break;

            case 't':
            getPIDTunings();
            break;

            case 'y':
            tuningPID = true;
            break;

            case 'u':
            tuningPID = false;
            break;

            case 'm':
            getMagOffset();
            break;
        }

        Serial.print(' ');
        Serial.println(c);
    } else
        Serial.println(F("\nMenu timed out."));

    Serial.print(F("remote control: ")); Serial.println(remote_control);
    Serial.print(F("manual override: ")); Serial.println(manual_override);
}


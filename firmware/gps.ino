// Module constants
static const uint32_t VALID_POS_TIMEOUT = 2000; // ms

// Module types
typedef void (*t_nmea_parser)(const char *token);

#define GPS_EN 30

enum t_sentence_type {
    SENTENCE_UNK,
    SENTENCE_GGA,
    SENTENCE_RMC
};


// Module constants
static const t_nmea_parser unk_parsers[] = {
    parse_sentence_type,        // $GPxxx
};

static const t_nmea_parser gga_parsers[] = {
    NULL,                        // $GPGGA
    parse_time,                  // Time
    NULL,                        // Latitude
    NULL,                        // N/S
    NULL,                        // Longitude
    NULL,                        // E/W
    NULL,                        // Fix quality
    NULL,                        // Number of satellites
    NULL,                        // Horizontal dilution of position
    parse_altitude,              // Altitude
    NULL,                        // "M" (mean sea level)
    NULL,                        // Height of GEOID (MSL) above WGS84 ellipsoid
    NULL,                        // "M" (mean sea level)
    NULL,                        // Time in seconds since the last DGPS update
    NULL                         // DGPS station ID number
};

static const t_nmea_parser rmc_parsers[] = {
    NULL,                        // $GPRMC
    parse_time,          // Time
    parse_status,        // A=active, V=void
    parse_lat,              // Latitude,
    parse_lat_hemi,  // N/S
    parse_lon,              // Longitude
    parse_lon_hemi,  // E/W
    parse_speed,            // Speed over ground in knots
    parse_course,        // Track angle in degrees (true)
    NULL,                        // Date (DDMMYY)
    NULL,                        // Magnetic variation
    NULL                            // E/W
};

static const int NUM_OF_UNK_PARSERS = (sizeof(unk_parsers) / sizeof(t_nmea_parser));
static const int NUM_OF_GGA_PARSERS = (sizeof(gga_parsers) / sizeof(t_nmea_parser));
static const int NUM_OF_RMC_PARSERS = (sizeof(rmc_parsers) / sizeof(t_nmea_parser));

// Module variables
static t_sentence_type sentence_type = SENTENCE_UNK;
static bool at_checksum = false;
static unsigned char our_checksum = '$';
static unsigned char their_checksum = 0;
static char token[16];
static int num_tokens = 0;
static unsigned int offset = 0;
static bool active = false;
static char gga_time[7] = "", rmc_time[7] = "";
static char new_time[7];
static uint32_t new_seconds;
static float new_lat;
static float new_lon;
static char new_aprs_lat[9];
static char new_aprs_lon[10];
static float new_course;
static float new_speed;
static float new_altitude;
static bool gps_on = false;


// Module functions
unsigned char from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else if (a >= '0' && a <= '9')
        return a - '0';
    else
        return 0;
}

void parse_sentence_type(const char *token)
{
    if (strcmp(token, "$GPGGA") == 0) {
        sentence_type = SENTENCE_GGA;
    } else if (strcmp(token, "$GPRMC") == 0) {
        sentence_type = SENTENCE_RMC;
    } else {
        sentence_type = SENTENCE_UNK;
    }
}

void parse_time(const char *token)
{
    // Time can have decimals (fractions of a second), but we only take HHMMSS
    strncpy(new_time, token, 6);
    // Terminate string
    new_time[6] = '\0';

    new_seconds =
        ((new_time[0] - '0') * 10 + (new_time[1] - '0')) * 60 * 60UL +
        ((new_time[2] - '0') * 10 + (new_time[3] - '0')) * 60 +
        ((new_time[4] - '0') * 10 + (new_time[5] - '0'));
}

void parse_status(const char *token)
{
    // "A" = active, "V" = void. We shoud disregard void sentences
    if (strcmp(token, "A") == 0)
        active = true;
    else
        active = false;
}

void parse_lat(const char *token)
{
    // Parses latitude in the format "DD" + "MM" (+ ".M{...}M")
    char degs[3];
    if (strlen(token) >= 4) {
        degs[0] = token[0];
        degs[1] = token[1];
        degs[2] = '\0';
        new_lat = atof(degs) + atof(token + 2) / 60;
    }
    // APRS-ready latitude
    strncpy(new_aprs_lat, token, 7);
    new_aprs_lat[7] = '\0';
}

void parse_lat_hemi(const char *token)
{
    if (token[0] == 'S')
        new_lat = -new_lat;
    new_aprs_lat[7] = token[0];
    new_aprs_lon[8] = '\0';
}

void parse_lon(const char *token)
{
    // Longitude is in the format "DDD" + "MM" (+ ".M{...}M")
    char degs[4];
    if (strlen(token) >= 5) {
        degs[0] = token[0];
        degs[1] = token[1];
        degs[2] = token[2];
        degs[3] = '\0';
        new_lon = atof(degs) + atof(token + 3) / 60;
    }
    // APRS-ready longitude
    strncpy(new_aprs_lon, token, 8);
    new_aprs_lon[8] = '\0';
}

void parse_lon_hemi(const char *token)
{
    if (token[0] == 'W')
        new_lon = -new_lon;
    new_aprs_lon[8] = token[0];
    new_aprs_lon[9] = '\0';
}

void parse_speed(const char *token)
{
    new_speed = atof(token);
}

void parse_course(const char *token)
{
    new_course = atof(token);
}

void parse_altitude(const char *token)
{
    new_altitude = atof(token);
}


//
// Exported functions
//
void gpsInit() {
    strcpy(gps_time, "000000");
    strcpy(gps_aprs_lat, "0000.00N");
    strcpy(gps_aprs_lon, "00000.00E");

    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);
    gps_on = true;
}

bool gps_decode(char c)
{
    int ret = false;

    switch(c) {
        case '\r':
        case '\n':
            // End of sentence

            if (num_tokens && our_checksum == their_checksum) {
#ifdef DEBUG_GPS
                log(" (OK!) ");
                log(millis());
#endif
                // Return a valid position only when we've got two rmc and gga
                // messages with the same timestamp.
                switch (sentence_type) {
                    case SENTENCE_UNK:
                        break;      // Keeps gcc happy
                    case SENTENCE_GGA:
                        strcpy(gga_time, new_time);
                        break;
                    case SENTENCE_RMC:
                        strcpy(rmc_time, new_time);
                        break;
                }

                // Valid position scenario:
                //
                // 1. The timestamps of the two previous GGA/RMC sentences must match.
                //
                // 2. We just processed a known (GGA/RMC) sentence. Suppose the
                //      contrary: after starting up this module, gga_time and rmc_time
                //      are both equal (they're both initialized to ""), so (1) holds
                //      and we wrongly report a valid position.
                //
                // 3. The GPS has a valid fix. For some reason, the Venus 634FLPX
                //      reports 24 deg N, 121 deg E (the middle of Taiwan) until a valid
                //      fix is acquired:
                //
                //      $GPGGA,120003.000,2400.0000,N,12100.0000,E,0,00,0.0,0.0,M,0.0,M,,0000*69 (OK!)
                //      $GPGSA,A,1,,,,,,,,,,,,,0.0,0.0,0.0*30 (OK!)
                //      $GPRMC,120003.000,V,2400.0000,N,12100.0000,E,000.0,000.0,280606,,,N*78 (OK!)
                //      $GPVTG,000.0,T,,M,000.0,N,000.0,K,N*02 (OK!)

                if (sentence_type != SENTENCE_UNK &&            // Known sentence?
                        strcmp(gga_time, rmc_time) == 0 &&      // RMC/GGA times match?
                        active) {                                                        // Valid fix?
                    // Atomically merge data from the two sentences
                    strcpy(gps_time, new_time);
                    gps_seconds = new_seconds;
                    gps_lat = new_lat;
                    gps_lon = new_lon;
                    strcpy(gps_aprs_lat, new_aprs_lat);
                    strcpy(gps_aprs_lon, new_aprs_lon);
                    gps_course = new_course;
                    gps_speed = new_speed;
                    gps_altitude = new_altitude;
                    ret = true;
                }
            }
#ifdef DEBUG_GPS
            if (num_tokens)
                logln("");
#endif
            at_checksum = false;                // CR/LF signals the end of the checksum
            our_checksum = '$';              // Reset checksums
            their_checksum = 0;
            offset = 0;                              // Prepare for the next incoming sentence
            num_tokens = 0;
            sentence_type = SENTENCE_UNK;
            break;

        case '*':
            // Handle as ',', but prepares to receive checksum (ie. do not break)
            at_checksum = true;
            our_checksum ^= c;

        case ',':
            // Process token
            token[offset] = '\0';
            our_checksum ^= c;  // Checksum the ',', undo the '*'

            // Parse token
            switch (sentence_type) {
                case SENTENCE_UNK:
                    if (num_tokens < NUM_OF_UNK_PARSERS && unk_parsers[num_tokens])
                        unk_parsers[num_tokens](token);
                    break;
                case SENTENCE_GGA:
                    if (num_tokens < NUM_OF_GGA_PARSERS && gga_parsers[num_tokens])
                        gga_parsers[num_tokens](token);
                    break;
                case SENTENCE_RMC:
                    if (num_tokens < NUM_OF_RMC_PARSERS && rmc_parsers[num_tokens])
                        rmc_parsers[num_tokens](token);
                    break;
            }

            // Prepare for next token
            num_tokens++;
            offset = 0;
#ifdef DEBUG_GPS
            log(c);
#endif
            break;

        default:
            // Any other character
            if (at_checksum) {
                // Checksum value
                their_checksum = their_checksum * 16 + from_hex(c);
            } else {
                // Regular NMEA data
                if (offset < 15) {  // Avoid buffer overrun (tokens can't be > 15 chars)
                    token[offset] = c;
                    offset++;
                    our_checksum ^= c;
                }
            }
#ifdef DEBUG_GPS
            log(c);
#endif
    }
    return ret;
}

void warnGPS() {
    digitalWrite(GPS_EN, LOW);
    gps_on = true;
}

// NOTE: This works well enough on an ATMega2560, with HardwareSerial.cpp hacked to increase the SERIAL_BUFFER_SIZE to 512 (up from 64).
//       This implies that serial buffers now take up half the RAM of a mega (4 serial ports x 2 buffers per port x 512 = 3072). For now,
//       we seem to be ok with the remaining ram. If that changes, the un-lazy thing to do is modify the HarwareSerial code to only
//       increase the rx_buffer size, and only for serial2.
//
//       Not increasing the buffer size can lead to overflows which would either degrade GPS read performance, or kill it entirely.
void serialEvent2() {
  while (Serial2.available()) {
    if (gps_decode(Serial2.read())) {
      last_gps_time = millis();

      if (!high_res_gps) {
        digitalWrite(GPS_EN, HIGH);
        gps_on = false;
      }
    }
  }
}
//
//void updateGPS() {
//    if (!gps_on)
//        warnGPS();
//
//    do {
//        while (Serial2.available() == 0);
//    } while (! gps_decode(Serial2.read()));
//
//    if (!high_res_gps) {
//        digitalWrite(GPS_EN, HIGH);
//        gps_on = false;
//    }
//}

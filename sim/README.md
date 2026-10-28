# ArduSailor simulator

Runs the real firmware (`firmware/`) on your computer against a simulated boat, wind, waves and sensors, and
shows it in a browser. Nothing in the pilot is re-implemented: `build.py` compiles the `.ino` and `.cpp` files
as they are, with a stand-in Arduino core underneath that talks to the simulation instead of hardware.

## Running it

On a Mac, install the command line tools once (they bring `clang` and `python3`):

    xcode-select --install

Then, from the top of the repository:

    python3 sim/run.py

It needs only Python 3 and a C++ compiler. It builds the simulator (a few seconds the first time, skipped when nothing has changed), starts it, and opens
http://localhost:8765. Ctrl-C stops it. Linux works the same way with `g++` or `clang++`.

| Option | |
|---|---|
| `--port N` | serve somewhere other than 8765 |
| `--speed X` | start at X times real time (0 = as fast as it will go) |
| `--no-browser` | don't open a browser window |
| `--reset-eeprom` | start with a blank EEPROM: the firmware's default PID gains, no compass calibration |
| `--rebuild` | rebuild even if nothing changed |

The simulated EEPROM is kept in `sim/.state/eeprom.bin`, so PID gains and compass calibrations survive
restarts, just as they would on the board.

## What's on the screen

- **Course**: the waypoints (drag them to move them), the route, the boat's track, the GPS fixes the firmware
  received, the heading the firmware is asking for and the heading it thinks it has, and the corridor it beats up
  when the waypoint is upwind. Scroll to zoom, drag to pan; *whole course* zooms out to the waypoints.
- **Boat from above**: hull, rudder and boom at their actual angles (the rudder's commanded angle is drawn
  faintly when the servo hasn't caught up), the sail's camber or luffing, true and apparent wind, the sheet's
  limit, and the wind vane.
- **From astern**: heel and the wave surface, with the firmware's own roll reading.
- **Last 60 seconds**: true heading, the firmware's compass heading, the heading it's steering for, and rudder.
- **Simulator vs firmware**: the true state next to what the firmware believes, line by line.
- **Serial**: the firmware's `Serial` output. Type into it like a serial monitor (`m` for the menu, `l` to
  toggle logging, `o` for manual override); the buttons send single keys.

On the right:

- **Pilot (PID)**: set the gains. *Apply* goes through the firmware's `updateCurrentPIDTunings()`, which
  stores them in EEPROM, so they stay after a reboot. The autotune box starts the firmware's PID autotune.
- **Compass calibration**: runs the firmware's own calibration from its menu, either lifting the boat out
  and turning it through every orientation (menu `c`) or circling on the water (menu `o`).
- **Wind, Waves, Boat, Servos & rigging, Sensors**: the simulated world. Wind direction, speed, gusts and
  shifts (and how quickly they come and go); wave height, period and direction; the boat's displacement,
  sail area, stability and weather helm; servo speeds and how the winch servo maps to the boom; compass hard
  iron, sensor noise and lag, GPS error, battery voltage. Changes apply immediately.
- **Firmware settings**: the firmware's tuning `#define`s (`IRONS`, `CORRIDOR`, `MIN_TACK_SPEED`, `GET_WITHIN`,
  `RUDDER_MAX`, `WINCH_MIN`, `AHRS_TRAIL`, ...), made adjustable. Ones marked ↻ are only read in `setup()`
  and take effect after *Reboot Arduino*.
- **Waypoints** and **Start over** (forget every change, or blank the EEPROM).

## How it works

```
firmware/*.ino, *.cpp ──build.py──▶ sketch.cpp ─┐
native/arduino/  (Arduino.h, Servo, Wire, EEPROM, MPU6050 ...)  ├─▶ sim/build/ardusailor-sim ◀─stdin/stdout─▶ run.py ◀─HTTP─▶ web/
native/devices.cpp, world.cpp, main.cpp      ─┘
```

- `build.py` copies the firmware to `sim/build/fw`, joins the `.ino` files the way the Arduino IDE does
  (adding function prototypes), and turns the tuning `#define`s into variables the simulator can change. It
  switches off `PILOT_DEBUG` and `NO_SAIL` in `config.h` (the sailing build); nothing else in the firmware is
  changed.
- `native/arduino/` is the stand-in core. `millis()`, `delay()`, serial output, I2C transfers, `analogRead()`
  and EEPROM writes all advance a simulated clock by roughly what they take on the board, and the world moves
  on while the firmware waits. So the firmware sees time pass much as it would on the board, and running
  faster than real time doesn't change what it sees.
- `native/devices.cpp` is the hardware: the MPU-9250 (DMP packets in a 512-byte FIFO that overflows like the
  real one, and the AK8963 magnetometer behind the MPU's I2C master, answering only when it's addressed and
  triggered the way the datasheet says), the hall-effect wind vane, the GPS's NMEA sentences at 9600 baud, the
  battery divider and the servos, including their power-enable pins.
- `native/world.cpp` is the physics: a ~1 m single-sail yacht with sail lift and stall, keel and rudder,
  hull drag up to hull speed, heel, weather helm, waves and gusty, shifting wind.
- `run.py` serves `web/` and relays between the browser and the simulator.

The simulator also runs without the UI, which is handy for comparing settings:

    python3 sim/build.py
    echo "pid 1 0 0.3" | sim/build/ardusailor-sim --headless --duration 600 --eeprom /tmp/e.bin \
        --set wind_speed=12 --tune IRONS=45 > run.jsonl

It writes one JSON object per line (`frame`, `serial`, `event`, `summary`, ...). The commands it accepts on
stdin are listed at the top of `native/main.cpp`.

## Limitations

- The boat model is plausible for a boat this size, not fitted to measurements of this one. Use it to find
  bugs and compare settings, not to predict exact speeds or turning rates.
- The host's `int` is 32 bits and `double` is 64; on the ATmega2560 they're 16 and 32. Overflow or precision
  bugs that depend on that won't show up here.
- Only waits are timed. Time spent computing on the 16 MHz AVR (floating point especially) isn't counted, so
  the real loop is somewhat slower.
- The GPS always has a fix, there's no land or current, and the magnetometer has hard iron but no soft iron.
- The firmware's motor mode (`NO_SAIL`) isn't simulated. Logging goes to serial only (`NO_SD`, as in
  `config.h`).

The firmware's PID and PID autotune libraries are in `third_party/` (see the README there).

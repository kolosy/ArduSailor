ArduSailor
==========
ArduSailor is an autonomous sail boat project based on the Arduino platform. Using a modified Arduino Mega board (schematics under `layout`), ArduSailor can be programmed with a set of GPS waypoints that it can then sail to.

The platform is built around a simple, two-servo sail boat design, with a winch servo for sail plan control and a rudder servo for steering. The circuit layout included is for a modified Arduino Mega 2560, which includes:

* A 9DOF IMU (an Invensense MPU9250)
* A break-out for a weather vane (this circuit is also included)
* A microSD card
* Two servo breakouts
* Three power regulators (one to provide 6V for the servos, one to provide 3.3V for the rest and one MPPT solar charger for the LiPo)
* A GPS chip (Venus 638FPLx)
* A breakout for a UART-based camera module (currently unused)
* A battery input (currently running from a 3S LiPo battery pack)
* A breakout for the main UART port for logging (on my build I have this routed to a 433MHz radio module)
* ICSP header for programming

The included firmware operates off of a list of GPS waypoints. The pilot steers straight for the next waypoint when it can. When the waypoint is upwind (within `IRONS` of the wind), it beats: close-hauled, zigzagging up a corridor `CORRIDOR` metres either side of the line from where the leg started to the waypoint, tacking at the corridor's edges, until it can lay the waypoint on one tack. To tack, it comes up to just outside the no-go zone (`TACK_MARGIN`) and goes through the wind from there, with way on (`MIN_TACK_SPEED`). If it can't get up to speed there, or its last two tacks ended in irons (`MAX_TACK_TRIES`), it goes round by gybing instead. Whenever the waypoint can be reached on the tack it's on, it steers straight for it. If the boat stops (in irons, or drifting backwards), it lets the sheet out and falls off (`RECOVER_ANGLE`) on whichever tack it's on, steering the other way while it's going backwards. It then stays on that tack, sailing a little free until it's back up to speed, rather than tacking straight back into irons. Outside a beat there is no cross-track correction.

Simulator
=========
`sim/` builds the firmware for your computer and sails it around a simulated course, with the boat, wind, waves and sensors shown in a browser. You can change the conditions, the PID gains and the firmware's settings while it runs. On a Mac (after `xcode-select --install`):

    python3 sim/run.py

See [sim/README.md](sim/README.md).

Status
======
I've built several iterations of the circuit board, and it works reliably. When at speed, the navigation works .. somewhat. My current testing is in a sub-optimal body of water (a long, narrow channel), making certain tests difficult.

Here's what I need to tackle

Short-term
----------
* Finish debugging basic piloting and navigation.
* Add cross-track correction
* Add heel compensation
* Add camera module support

Longer-term
-----------
* Improve the navigation algorithm to include no-go zones
* Add wind speed sensor (hw)

Pie-in-the-sky
--------
I'd like to replace the pilot code entirely with AI. This should be reduce-able to an optimization problem... but that's a bit beyond me right now.

Enjoy!

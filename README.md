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

The included firmware operates off of a list of GPS waypoints, and is a very rudimentary implementation of a greedy algorithm for navigation. The pilot simply tries to point the boat as close to the next waypoint as it can, without falling into irons. Tacks are limited by a timer. A recent addition is stall control - if boat speed falls below a certain limit, it will fall off to beam reach (position itself with the wind coming in at a 90 degree angle) until it comes back up to speed. There is no cross-track correction

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

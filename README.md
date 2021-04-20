# dieslg8
Clear DTCs and hijack gauges with Arduino+CAN

## Disclaimer
* Not for use on any public highway or wherever doing so may be against any relevant laws.
* Always wear your seatbelt.
* If it breaks/hurts/kills you/your car/your something else/etc... not my fault.
  * Harsh disclaimer but it is what it is.
  * See MIT license.
* I make no assertations that I am a professional. I'm just doing this for fun and to learn Arduino.
* It's probably best to use this project as a reference versus fork it. I have no idea what direction it will go in.

## Hardware
* Arduino Uno
* Seeed Studio CANBUS v2 shield
* DB9 -> OBD2 port adapter

## Vehicle
* BMW 2011 E90 335d (type code `PN73`)
* M57 3L turbodiesel
* Bosch EDC17 DDE

## Functions
* NODE: ACC LED hijack is not working, I need to find the relevant bits and I don't have a test mule anymore
* Flash turn signal LEDs in cluster when coolant temp target is reached (a la N54 JB4)
* Gauge sweep on startup
  * Sweeps all 4 gauges, not just speedo and tach
* Log data to CSV on MicroSD card
* Consumption/oil gauge hijack
  * Shows coolant temp on consumption/oil gauge
  * Really not useful on oil-temp-gauge equipped cars
  * Reference table below
* Fuel gauge hijack
  * When throttle is above 49%, the fuel gauge becomes a boost gauge
  * Reference table below
* Clear selective DTCs on KL30 (if enabled, by default it is not)
  * Once the parts are replaced on my test mule, I'll be removing the glow plug and MAF related codes (see table below)
* Clear all DTCs on KL30 (if enabled, by default it is not)

## DTCs cleared
Group | DTC | Description
----- | --- | -----------
EGR | `40D4` | EGR actuator position control
EGR | `485C` | EGR cooler bypass valve control
EGR | `4B39` | EGR actuator control
EGR | `4B73` | EGR engine exhaust heating control
EGR | `4CAE` | EGR position sensor plausibility
MAF | `3FF1` | Mass air flow sensor
MAF | `4596` | Smooth running controller cylinder 3
Glow plugs | `4A1E` | Glow plug activation, cylinder 6
Glow plugs | `4A24` | Glow plug activation, cylinder 5
Glow plugs | `4A2E` | Glow plug activation, cylinder 5

## Fuel gauge hijack
Boost PSI | Gauge %
--------- | ------------
0 psi | 0%
10 psi | 25%
20 psi | 50%
30 psi | 75%
40 psi | 100%

## Consumption/oil gauge hijack
Coolant temp C | Gauge %
--------- | ------------
0 C | 0%
37.5 C | 25%
75 C | 50%
112.5 C | 75%
150 C | 100%

## Reference material(s)
* BMW Tool32
  * D73N57C0.prg
  * KOMB87.prg
* [My other work](https://github.com/kmalinich/node-bmw-ref)

## Greetz
* [@uholeschak](https://github.com/uholeschak)

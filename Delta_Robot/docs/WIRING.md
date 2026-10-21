# Wiring (as read from the firmware — verify against the real machine)

> This table comes from the comments and pin constants in `firmware/Delta_Robot_ESP32/Delta_Robot_ESP32.ino`.
> It was **not** checked against the physical robot. If your wiring differs, the firmware pin arrays are wrong.

## ESP32 pins

| Axis | STEP (PUL) | DIR | ENABLE | Limit switch |
|---|---|---|---|---|
| 0 (A) | GPIO 25 | GPIO 32 | GPIO 5  | GPIO 21 |
| 1 (B) | GPIO 26 | GPIO 33 | GPIO 18 | GPIO 22 |
| 2 (C) | GPIO 27 | GPIO 4  | GPIO 19 | GPIO 23 |

* Each **DM542T** gets PUL+, DIR+ and ENA+ from the ESP32; the matching `−` inputs go to ESP32 ground
  (the project owner states that any driver input not listed in the firmware is tied to ground).
* ENA is **active-low** in the firmware: LOW = driver enabled. The firmware starts with drivers **disabled**.
* **Limit switches** are normally-open, between the GPIO and GND, using the internal pull-up (pressed = LOW).
* The ESP32 is 3.3 V logic. DM542T opto-inputs are usually specified for 5 V; if steps are skipped, check the
  opto-input current/series resistor first.
* Pins avoid the ESP32 boot-strapping pins (0, 2, 12, 15) and flash pins (6–11).

## Motors and mechanics

* 3 × NEMA 23, each with a **47:1 gearbox** (so one arm revolution = 47 motor revolutions).
* Steps per arm revolution = 200 × (DM542T microsteps) × 47. **The microstep DIP-switch setting has not been recorded yet.**
* Homing direction is `+1` on all axes in the current firmware (changed from `−1` by trial). Homing has never been
  validated — see `PHASE1_REVIEW.md`.

## DM542T DIP switches (as reported by the project owner)

Switches **3, 4, 7 and 8 are ON; 1, 2, 5 and 6 are OFF** (assumed identical on all three drivers — please confirm).
Decoded with the standard DM542T table (**check it against the table printed on your driver; clone drivers can differ**):

| Switches | Setting | Meaning |
|---|---|---|
| SW5 OFF, SW6 OFF, SW7 ON, SW8 ON | **3200 pulses per motor revolution** | 16 microsteps × 200 full steps |
| SW1 OFF, SW2 OFF, SW3 ON | about **2.37 A peak** (≈1.7 A RMS) | check against your motors' rated current |
| SW4 ON | normally "full current at standstill" | holds the arms firmly; the motors get warm |

**Steps per arm revolution = 3200 × 47 (gearbox) = 150 400**, i.e. **417.78 steps per degree**.
The current Tkinter GUI's default `Steps/rev = 3200` ignores the gearbox: set it to **150400** until the new GUI (separate
motor-step / microstep / gearbox fields) replaces it. At that resolution a 30° arm move is about 12 500 steps.

## Not wired yet

* **Vacuum gripper** — only an idea; no pin or driver chosen.
* **Hardware emergency stop** — planned: a switch that cuts the driver power supply. Until it exists, keep a hand on the
  power switch whenever the motors are enabled.

## Cameras

* Arducam ToF depth camera (240×180): CSI or USB — **not yet checked**. It needs its own 5 V 2 A supply per the Arducam README.
* 1.0-megapixel USB camera (model no. 202511028880000100): planned for later.

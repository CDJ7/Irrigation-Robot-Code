# ESP32 firmware, version 2 (Phase 7)

File: `firmware/Delta_Robot_ESP32/Delta_Robot_ESP32.ino` (the original is kept in `archive/original_files/`).

**Status: written and tested on a PC only. It has NOT been flashed and has never driven a motor.** It was compiled with `g++` against a
small fake Arduino core (`firmware/tests/shim/Arduino.h`), not with the real ESP32 toolchain, so the Arduino IDE may still report
something the PC compiler did not. Run the PC tests with `sh firmware/tests/run_tests.sh` (53 checks).

## What changed
| Area | Version 1 | Version 2 |
|---|---|---|
| Motion | constant speed, instant start and stop | trapezoidal: starts at `START_RATE`, accelerates at `ACCEL_STEPS_S2`, brakes in time, lands exactly |
| Vacuum | none | `V 1` / `V 0` / `V?` on **GPIO 17, active HIGH**; off at boot, off on `STOP`, off after 30 s (`WARN VACUUM_TIMEOUT`); `V 1` refused while moving |
| Homing | blocked, threw away every command except STOP, a stray byte could stall it ~1 s | serial read without blocking; STOP aborts; other commands are **queued** (up to 4) and run after homing |
| Limit switches | one low reading counts | 3 consecutive low readings; an axis already on its switch is not driven further into it |
| `H` with drivers off | "homed" a machine that never moved | `ERR NOT_ENABLED` |
| `EN` / `Z` while moving | accepted (could drop the motors mid-move) | `ERR BUSY` (use `STOP` first) |
| Timer | broke when `micros()` wrapped (every 71 min): moves stalled | wrap-safe 32-bit arithmetic (the PC test reproduces the old failure and shows it fixed) |
| Garbage input | grew without limit | lines over 80 characters discarded (`ERR LINE_TOO_LONG`) |

All version-1 commands and replies are unchanged, so the GUI works with either firmware. (The GUI's simulated ESP32 still reproduces
the *version 1* quirks on purpose, so the GUI stays safe with both.)

## Acceleration: what it means for `M ... duration_ms`
`duration_ms` is the time at **cruise speed**. With acceleration the real time is longer by about `cruise_rate / ACCEL_STEPS_S2`
seconds (0.2 s at the GUI's default 4000 steps/s; 1.5 s at the 30000 ceiling). The GUI waits `duration + 3 s` for the reply, so this is
covered. A very short move never reaches cruise speed (triangle profile). `STOP` still stops instantly (no braking ramp).

## Values to tune on the machine (all UNVERIFIED starting points, top of the .ino)
`ACCEL_STEPS_S2` 20000, `START_RATE` 300, `MAX_STEP_RATE` 30000 (steps per second of the fastest axis; 417.8 steps = 1 degree of arm),
`HOMING_STEP_DELAY_US` 900, `BACKOFF_STEPS` 200 (only about 0.5 degree of arm: check by eye that the switch released after homing),
`LIMIT_DEBOUNCE_SAMPLES` 3, `VACUUM_MAX_ON_MS` 30000. Lower `ACCEL_STEPS_S2` first if a motor stalls or the arms jerk.

## Vacuum wiring (GPIO 17, active HIGH)
* GPIO 17 -> input of a relay/MOSFET module that switches on with a 3.3 V HIGH. Share GND with the ESP32.
* **Add a 10 kOhm resistor from GPIO 17 to GND**: the pin floats briefly while the ESP32 boots and while flashing; the resistor keeps the vacuum off.
* The pin supplies only a few mA: never drive a pump or relay coil straight from it. Switch the pump from its own supply.
* If your module is active LOW, change `VACUUM_ACTIVE_HIGH` to `false`.
* (GPIO 35, first suggested, is input-only and cannot drive anything.)

## Flashing (YOU do this; I will not flash without asking and being told it is safe)
1. **Make the robot safe first:** cut the motor power supply (the DM542T supply), disconnect or switch off the vacuum pump, and keep the arms clear. The ESP32 only needs USB.
2. Close the Delta Robot GUI (it holds the COM port).
3. Arduino IDE: board *ESP32 Dev Module*, port = your ESP32 (it showed COM5). Open the .ino, press Upload. If it hangs at "Connecting...", hold the BOOT button until it starts.
4. Open the Serial Monitor at 115200 baud. Expect `READY DeltaRobotSlave`.

## Bench test with the motor power still OFF
Send each line in the Serial Monitor (Newline ending): `P?` -> `POS 0 0 0`; `V 1` -> `OK` and a meter on GPIO 17 reads about 3.3 V; `V?` -> `VAC 1`;
`V 0` -> `OK`, 0 V; `V 1` then wait 30 s -> `WARN VACUUM_TIMEOUT`, 0 V; `H` -> `ERR NOT_ENABLED`. Pressing a limit switch by hand should read LOW on its pin.
Only after that, with the motor supply on and the arms clear, try a very small move in the GUI (jog 5 mm) and watch it. Homing needs someone at the power switch.

# Troubleshooting

## Can't reach the Raspberry Pi
* Use the **IP address**, not `deltapi.local` (it was `192.168.137.128` over the Windows shared Ethernet; it can change). On Windows: `arp -a` or `ping 192.168.137.128`.
* The Ethernet cable must go from the PC to the Pi with **Internet Connection Sharing** on for that adapter. Unplugging/replugging re-establishes it.
* If SSH drops ("Connection timed out") the Pi may have lost power or the link: check the Pi's power supply (27 W official), `vcgencmd get_throttled` (0x0 = fine),
  and enable persistent logs (`journalctl --list-boots`) to see what happened before a drop.
* The Pi's repo copy is updated with `scp -r Delta_Robot deltarobot@<ip>:~/`, then fix line endings: `find ~/Delta_Robot -type f \( -name "*.sh" -o -name "*.py" -o -name "*.yaml" \) -exec sed -i 's/\r$//' {} +`
  (symptom of CRLF: `set: Illegal option -`).

## Camera / depth
| Symptom | Cause / fix |
|---|---|
| Broken or noisy pictures | Lens caps on (the dark frames corrupted MJPG). Config uses YUYV 1280x480 at 15 fps. Try another USB 3 port or cable. |
| `Camera busy` / cannot open | Another program has it: `sudo systemctl stop delta-vision`, or a second server is running. |
| Server only about 6 fps | Depth search range or picture size is large. Run `calibrate_stereo.py solve --work-range NEAR FAR` and use the recommended `min_disparity`/`num_disparities`. |
| "UNCALIBRATED" warning | No `calibration/stereo_calib.npz`: do the stereo calibration. |
| Depth is null at the left edge | By design: no depth within one search range of the left edge. Keep the work area right of it. |
| Distance off by several % | Wrong `--square-mm` (measure the printed square!) or a bad calibration. Redo; check baseline vs ruler. |
| Detections show Z = null | Not enough valid depth in the box (`min_valid_fraction`), too close/far (`min_z_mm`/`max_z_mm`), shiny or textureless surface. |
| No boxes at all | `detection.enabled: false` (default) or ultralytics not installed. |
| Boxes are red, not green | Class is not on `allowed_classes`, or you are using the stock model (no strawberry class). |

## GUI
* **Settings page squashed** (tiny boxes): fixed with scrolling pages; make sure you have the latest files. Window minimum is 1100x720.
* **Camera page "connecting"**: wrong address, server not running, firewall. Test `http://<pi-ip>:5000` in a browser first.
* **Buttons grey**: each page says why (not connected, motors off, position unknown, E-STOP active). Read the hint next to the button and the log at the bottom.
* **"Position unknown"** after any raw/manual move, reset, disable or lost reply: press HOME (with a hand on the power switch).
* **E-STOP latched**: click the button again to release, then Enable motors and Home. A software E-STOP is not a safety circuit.
* **Simulation**: tick *Simulated ESP32* (and *Simulated camera (test mode)* on the Camera page) to try everything with no hardware.
* **Harvest Start disabled**: the checklist lists exactly what is missing. Typical: camera -> robot calibration, home angles not calibrated, no strawberry class in the model.

## ESP32 / motors  (cut motor power before debugging wiring)
| Symptom | Cause / fix |
|---|---|
| COM port busy / not listed | Close the other program (Arduino Serial Monitor, old GUI). Replug USB. Install the CP210x/CH340 driver. |
| No `READY DeltaRobotSlave` | Wrong baud (115200), wrong port, or the ESP32 is resetting (brown-out: use a good USB cable/port). |
| Motors do not move | Drivers disabled (`EN 1`), driver power off, ENA polarity (`EN_ACTIVE_LOW`), STEP/DIR wiring, driver current. |
| Moves are 47x too small | Steps/rev not 150400. Check microsteps and the gearbox ratio in Settings. |
| Arm moves the wrong way | Tick **Invert** for that arm (Calibration -> Home && axes -> nudge test). |
| Stalls / grinding / lost steps | Lower `max_step_rate` (Settings) and `ACCEL_STEPS_S2` (firmware); check driver current setting and supply voltage. |
| Homing runs into the stop | Wrong `HOME_DIR`, or switch wired wrong (NO to GND, triggered = LOW). STOP / cut power. After flashing v2: homing needs drivers enabled (`ERR NOT_ENABLED`). |
| Vacuum does nothing | GPIO 17 active HIGH, module needs 3.3 V-compatible input, shared GND, firmware v2 flashed, Harvest "Use the vacuum" ticked. Measure 3.3 V on GPIO 17 after `V 1`. |
| `ERR UNKNOWN_CMD V 1` | The ESP32 still has the old firmware. |
| Reply timeout, "no reply to M..." | Cable, power dips, or a very slow move. The GUI then treats the position as unknown: Home again. |

## Self-tests (find out if the code itself is fine)
`cd pi && python tests/test_server_fake.py` (and the others), `cd gui && python tests/test_core_robot.py` ... and `sh firmware/tests/run_tests.sh`. Each prints `ALL TESTS PASSED`.

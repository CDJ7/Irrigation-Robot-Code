# CLAUDE.md — Strawberry-harvesting delta robot

Read this first in every new session. Keep it updated as work proceeds.

## Goal
Detect strawberries with a stereo camera + YOLO on a Raspberry Pi, and have a PC GUI pick them with a
3-motor delta robot. Final system: **Pi = vision only; PC GUI = the ONLY thing that controls the ESP32.**

## Architecture
```
Pi 5 (deltapi.local, user deltarobot)               Windows PC                         ESP32 (USB to PC now, Wi-Fi later)
 stereo camera /dev/video0                           delta_robot_combined_v3.py (Tk)    3x DM542T -> 3x NEMA23
   -> pi/pi_camera_server.py  --TCP 8485 (J,V,D)-->    camera_client.py                 delta_robot_esp32.ino
      (rectify, SGBM depth, YOLO)  browser :5000        GUI: coords -> IK -> serial  --> M/H/EN/STOP/P?/Z
```
* Pi never sends motor commands. The old `pi/main_harvester.py` (YOLO + serial + placeholder IK) is **retired**;
  the original is in `backup_originals/` for reference only. Do not run it (it holds the camera and moves the robot).
* Planned: ESP32 will move to Wi-Fi later, so the GUI's serial layer should stay behind a small link class
  that could be swapped for TCP.

## Safety rules (always)
* NEVER run anything that moves the robot, and never flash the ESP32, without asking the user and waiting for confirmation.
* Keep GUI safety checks: workspace box, IK-unreachable check, `busy` flag, `xyz_valid` after raw moves, EMERGENCY STOP.
* Automation may only pick detections whose class is in `allowed_classes` (strawberry). Stock `yolov8n.pt` has no
  strawberry class (it sees people) -> automation must refuse to run when the J message says
  `model_has_allowed_class: false` or `calibrated: false`.
* Ask before sudo, installing system packages, rebooting, flashing.
* Back up originals before modifying (done: `backup_originals/`).

## Hardware (known)
* Raspberry Pi 5 8 GB, Raspberry Pi OS 64-bit. Host `deltapi.local`, user `deltarobot`, project in `~/delta_robot/`.
* Camera: **monochrome USB stereo**, `/dev/video0`, frame 640x240 = two 320x240 eyes side by side, delivered as
  3 identical channels. Baseline unmeasured (60 mm placeholder). **No colour camera** -> cannot judge ripeness.
  Exact camera model not yet identified.
* Delta geometry: bicep rf = 254 mm (10 in), forearm re = 342.9 mm (13.5 in), effector radius e = 76.2 mm (3 in),
  **base radius f = 3 in = 76.2 mm for now (final design TBD)**. Workspace box 24x27x24 in (editable in the GUI;
  only ~half of it is actually reachable by IK).
* Motors: 3x NEMA 23 via DM542T drivers wired directly to the ESP32 (PUL/DIR/ENA). Driver inputs not listed in the
  firmware are assumed tied to ground. **Gearbox 47:1 on each axis.** DM542T microstep DIP setting: **still unknown**
  (GUI's old `steps_per_rev = 3200` = 200 x 16, no gearbox). Correct value = 200 x microsteps x 47.
* ESP32 pins (from firmware): STEP 25/26/27, DIR 32/33/4, EN 5/18/19, LIMIT 21/22/23 (NO switch to GND, pull-up).
* Vacuum gripper: only an idea; not wired. Ignore until the user says otherwise.

## Network protocol (Pi -> GUI, TCP 8485) — full spec in the header of `pi/pi_camera_server.py`
Messages: 1-byte tag, 4-byte big-endian length, payload. Per frame the server sends `J`, `V`, `D` in that order.
`J` = UTF-8 JSON (seq, t, size, calibrated, intrinsics, model, model_has_allowed_class, center_probe, detections[]
with class/confidence/allowed/bbox/center/xyz_mm|null...). `V` = rectified left eye JPEG, `D` = colorized disparity JPEG.
Camera frame: X right, Y down, Z out of the lens; units mm. The `V`/`D` after a `J` belong to that `J`'s `seq`.
Old clients that only know V/D still work.

## Status (update this)
| Phase | State |
|---|---|
| 1 Review | done (bug list delivered in chat) |
| 2 Pi server | **code written, tested only with a synthetic camera on a dev VM; NOT yet run on the Pi** |
| 3 Client + GUI camera tab | not started |
| 4 Calibration tools in GUI | not started |
| 5 Automated harvesting | not started |
| 6 Firmware | not started |

Phase 2 tested (dev VM, fake stereo scene, OpenCV 4.14 and 5.0): J/V/D protocol and order, exact depth from a synthetic
pair (420.0 mm expected 420), null depth over holes, allowed-class flag, 2 TCP + 1 browser client at once, stalled
client dropped without affecting others, dataset capture, SIGTERM shutdown, calibration capture+solve on synthetic
boards (baseline 62.06 vs 62.0 true), YoloDetector wrapper against a stand-in API.
**Not verified:** real camera format, real FPS on the Pi, real YOLO (PyTorch could not be installed in the dev sandbox),
real calibration quality, systemd install. Dev-VM fps numbers are meaningless for the Pi.

## Calibration status
* Stereo: **not calibrated** (server runs UNCALIBRATED with nominal 600 px / 60 mm from the old script — untrusted).
* Home / axis offsets / HOME_XYZ: **uncalibrated** (placeholder HOME_XYZ (0,0,-200) needs bicep angle -5.4 deg at f=100; step 0 = limit switch).
* Camera-to-robot transform: **not done**. Base radius f final: TBD. Microsteps: TBD.

## How to run
See `pi/README.md` (deploy, venv, calibration, service, dataset capture, model swap). Quick tests on any machine with
OpenCV + PyYAML:
```
cd pi
python tests/test_server_fake.py
python tests/test_calibration_synthetic.py
python tests/test_yolo_wrapper_stub.py
python pi_camera_server.py --fake      # then open http://localhost:5000
```
GUI (Windows): `pip install pyserial pillow opencv-python numpy`, then `python delta_robot_combined_v3.py`
(currently the unmodified original; Pi address default must be changed to `deltapi.local` in Phase 3).

## Files
* `pi/` — vision server, config.yaml, calibrate_stereo.py, make_checkerboard.py, fake_camera.py, tests, systemd unit.
* `delta_robot_combined_v3.py`, `camera_client.py`, `delta_robot_esp32/delta_robot_esp32.ino` — working copies (unchanged so far).
* `backup_originals/` — untouched originals of every file given by the user (incl. old `main_harvester.py`).
* Repo root (outside this folder) holds an unrelated irrigation-robot sketch (`irrBot-version1.ino`, `2`) that uses
  MQTT (broker 192.168.1.104:1883) and has Wi-Fi credentials committed — not part of the delta robot.

## Open questions
1. DM542T microstep setting (DIP switches).  2. Final base radius f.  3. Camera baseline (ruler) and model.
4. Working distance range of strawberries (sets `num_disparities`).  5. Colour camera later?
6. Vacuum design (pin, driver, polarity) when it exists.

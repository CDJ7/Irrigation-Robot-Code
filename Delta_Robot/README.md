# Delta Robot — strawberry-harvesting delta robot

A 3-armed **delta robot** that is meant to find ripe strawberries with a depth camera and a YOLO
neural network, then pick them. This folder holds everything for that project: the PC control
program, the Raspberry Pi vision server, and the ESP32 motor firmware.

> **Project status: work in progress.** This README describes what is in the folder **today** and
> marks what is still planned. Nothing here has been tested on the real robot by the assistant that
> wrote this folder — see [Status](#status-what-works-and-what-does-not).

## How the pieces fit together

```
   ┌──────────────────────┐  USB stereo camera (found on the Pi);
   │  Cameras: USB stereo │  an Arducam ToF camera would be an
   │  (2 x ~1 MP, gray);  │  optional extra depth source.
   │  ToF = optional      │
   └──────────┬───────────┘
              │ USB
   ┌──────────▼───────────┐   Wi-Fi / Ethernet, TCP port 8485    ┌───────────────────────┐
   │  Raspberry Pi 5      │ ──────────────────────────────────▶ │  Windows PC           │
   │  pi/ : VISION ONLY   │   J = detections (JSON)             │  gui/ : the ONLY      │
   │  capture, depth,     │   V = video frame (JPEG)            │  controller of the    │
   │  YOLO, streaming     │   D = depth picture (JPEG)          │  robot. Converts      │
   │  (never moves motors)│                                     │  camera → robot       │
   └──────────────────────┘                                     │  coordinates, runs    │
                                                                │  inverse kinematics   │
                                                                └───────────┬───────────┘
                                                                            │ USB serial, 115200 baud
                                                                            │ (Wi-Fi planned later)
                                                                ┌───────────▼───────────┐
                                                                │  ESP32                │
                                                                │  firmware/ : makes    │
                                                                │  step pulses, homing  │
                                                                └───────────┬───────────┘
                                                                            │ STEP / DIR / ENA
                                                                ┌───────────▼───────────┐
                                                                │ 3× DM542T drivers →   │
                                                                │ 3× NEMA 23 motors     │
                                                                │ (47:1 gearboxes)      │
                                                                └───────────────────────┘
```

The rule that keeps the system safe and simple: **the Pi only looks, the PC only decides, the ESP32
only moves.** The Pi never sends motor commands, and only the PC talks to the ESP32.

## What is in this folder

| Path | What it is | State |
|---|---|---|
| `README.md` | This file. | — |
| `CLAUDE.md` | Working notes for AI coding assistants (architecture, hardware, decisions, status). Humans can read it too. | kept up to date |
| `.gitignore` | Keeps secrets, model weights, datasets, videos and `third_party/` out of git. | — |
| **`gui/`** | **The program that runs on the Windows PC.** | |
| `gui/Delta_Robot_GUI.py` | Tkinter control window: connect to the ESP32, move to XYZ, jog, home, raw motor control, a *random-point* camera simulator. Contains the delta **inverse kinematics** (`DeltaKinematics`). | **Original, unmodified.** To be rebuilt in PyQt6. |
| `gui/camera_client.py` | Reads the Pi's video stream (`V`/`D` frames). | **Original.** Known problems: it still says "connected" after the Pi disappears, never reconnects, and ignores `J` detections. To be rewritten. |
| **`pi/`** | **The program that runs on the Raspberry Pi 5.** See `pi/README.md` for setup. | |
| `pi/pi_camera_server.py` | Vision server: captures the camera once, computes depth, runs YOLO, and streams to any number of clients (the GUI and/or a browser at `http://deltapi.local:5000`). The network protocol is documented at the top of the file. | Written for the **USB stereo camera**; tested only against a synthetic camera. To be reworked around the **Arducam ToF** camera. |
| `pi/config.yaml` | All settings: camera, depth, YOLO model path, allowed classes, ports. Swapping a trained model is a one-line change. | — |
| `pi/calibrate_stereo.py`, `pi/make_checkerboard.py`, `pi/checkerboard_9x6_25mm_letter.svg` | Stereo-camera calibration (checkerboard) tools. Only needed for the stereo camera, not for a ToF camera. | stereo only |
| `pi/fake_camera.py` | A pretend camera, so the server can be tested without hardware (`python pi_camera_server.py --fake`). | — |
| `pi/tests/` | Self-tests that run on any computer (no Pi, no camera, no robot). | passing |
| `pi/camera_check.py` | **Headless check of the stereo camera**: measures every mode (real frame size, real fps, grayscale?) and saves left/right pictures. | written; tested only with a fake camera |
| `pi/tof/tof_check.py` | **Headless test of the Arducam ToF camera** (works over SSH): prints resolution, intrinsics, frame rate and the distance at the centre pixel, runs a flat-wall check, and saves depth/confidence/amplitude pictures. | written; tested only against a stand-in SDK |
| `pi/tof/setup_venv.sh` | Creates the Python virtual environment and installs the ToF SDK into it (no `sudo`). | not yet run on a Pi |
| `pi/delta-vision.service`, `pi/install_service.sh` | Run the vision server automatically at boot (systemd). **You run the install script yourself; it uses `sudo`.** | not yet run on a Pi |
| `pi/requirements.txt` | Python packages for the Pi (install inside a virtual environment). | — |
| **`firmware/`** | **Code that runs on the ESP32.** | |
| `firmware/Delta_Robot_ESP32/Delta_Robot_ESP32.ino` | Receives text commands over USB, makes the step pulses for the three motors so all arms arrive together, runs homing with limit switches. Open this folder in the Arduino IDE (the sketch must live in a folder of the same name). | **Original, unmodified.** |
| **`docs/`** | Documentation. | |
| `docs/PI_SETUP.md` | Step-by-step Raspberry Pi setup: what to check, installing the ToF SDK, testing the camera, the CSI boot overlay, VNC. | checklist, not yet run on the Pi |
| `docs/PROTOCOL.md` | The serial commands (PC ↔ ESP32) and the network messages (Pi → PC). | — |
| `docs/WIRING.md` | Which ESP32 pin goes where, as read from the firmware. | needs checking against the real wiring |
| `docs/PHASE1_REVIEW.md` | A review of bugs and risks found in the original code, ranked by how dangerous they are. **Read this before running the robot.** | — |
| **`archive/`** | Untouched copies of every original file, kept as a backup. See `archive/README.md`. | — |

Not in git on purpose: `third_party/` (reference copies of the Arducam, Ultralytics and PyQt
projects), model weights (`*.pt`), datasets, videos, and anything with passwords.

## Status: what works and what does not

| Area | State |
|---|---|
| Pi vision server (stereo USB camera, YOLO, JSON detections, multi-client) | Code written; passes self-tests with a **synthetic** camera. **Not yet run on the real Pi or camera.** |
| Camera | The Pi's own inspection found **only a USB stereo camera** ("3D Global Shutter Camera", 2 × ~1 MP) and **no ToF camera**. The stereo server fits it; ToF tools are kept in case a ToF camera is connected later. |
| New PyQt6 GUI | **Not started.** `gui/` is still the original Tkinter program. |
| Calibration tools in the GUI (gearbox, home angles, camera→robot) | **Not started.** |
| Automated harvesting | **Not started.** |
| Firmware improvements (acceleration, vacuum, safer homing) | **Not started.** |
| Strawberry YOLO model | **Does not exist yet.** The stock YOLO models have no strawberry class (they detect people, cars, …). Automated picking must refuse to run with such a model. |

## Safety — read first

The motors are NEMA 23 steppers behind 47:1 gearboxes. They can hurt you.

* **Keep your hand on the power supply switch.** The current GUI's "EMERGENCY STOP" only sends a
  `STOP` text command over USB. If the cable, the PC or the ESP32 hangs, it will not stop anything.
  A real **hardware E-stop that cuts driver power** is planned and strongly recommended.
* **Do not press HOME until you can cut motor power.** The original GUI itself warns that the
  Home button "does NOT work yet", and the homing direction was changed by trial.
* **"Disable Motors" lets the arms fall** if they are holding weight.
* The GUI refuses XYZ moves after a raw/manual motor move until you home again (`xyz_valid`).
  Keep it that way.
* **Camera Tracking in the current GUI moves the robot to random points every second.** It is a
  simulator. The rebuilt GUI will keep simulators only as explicit test modes.
* The current GUI's `Steps/rev` default (3200) ignores the 47:1 gearbox (**use 150400**); XYZ moves would otherwise be 47× too small.
* The current GUI has **no workspace limits**. A bounded "workspace box" existed in an earlier
  version (`archive/original_files/delta_robot_combined_v3.py`) and will be ported back.
* **Automated picking may only target strawberry detections**, and must refuse a model that has no
  strawberry class.

## Running what exists today

**PC (Windows):**
```
pip install pyserial
python gui/Delta_Robot_GUI.py
```

**Raspberry Pi** (vision server, stereo-camera version): follow `pi/README.md`
(virtual environment, `--fake` test first, then the real camera).

**Self-tests** (any computer with Python, OpenCV, PyYAML, NumPy — no hardware needed):
```
cd pi
python tests/test_server_fake.py
python tests/test_calibration_synthetic.py
python tests/test_yolo_wrapper_stub.py
```

**ESP32:** open `firmware/Delta_Robot_ESP32/Delta_Robot_ESP32.ino` in the Arduino IDE, choose your
ESP32 board, and upload. *Flashing changes what the motors do — do it deliberately.*

## Hardware facts used so far

* Raspberry Pi 5 (8 GB), host `deltapi.local`, user `deltarobot`.
* Camera found on the Pi: **USB stereo camera "3D Global Shutter Camera"** (USB ID `32e4:9282`): two global-shutter eyes side by
  side, up to 2560×800 (1280×800 per eye, ~1 MP). It is most likely the "1.0-megapixel USB camera, model no. 202511028880000100".
  **No Arducam ToF camera has been detected** (a USB ToF camera needs its own 5 V / 2 A supply). Mounting: top-down or slightly angled.
* Motors: 3× NEMA 23 with **47:1 gearboxes**, driven by 3× DM542T from an ESP32. DIP switches 3, 4, 7, 8 are ON, which
  (standard DM542T table) means **3200 pulses per motor revolution** → **150 400 steps per arm revolution**
  (417.78 steps/degree). The current Tkinter GUI's default `Steps/rev = 3200` is therefore wrong — use 150400.
* Robot geometry: base radius **f = 3 in (76.2 mm)** for now (final design not decided); arm lengths from the current GUI:
  bicep `rf = 150`, forearm `re = 280`, effector `e = 50` mm. (An older GUI used 254 / 342.9 / 76.2 mm — **measure all of these
  once**, they are the basis of every XYZ move.)
* ESP32 connects to the PC by USB now; Wi-Fi later. The vacuum gripper is only an idea so far.

## Roadmap

1. Review of the original code — done (`docs/PHASE1_REVIEW.md`).
2. Pi set-up (stereo camera path; ToF only if a ToF camera turns up) and remote viewing — *tools and checklist written; waiting to be run on the Pi* (needs `sudo`, possibly a config-file edit and a reboot, only with the owner's approval).
3. Vision server: the stereo version exists (real depth after calibration, YOLO, `J`/`V`/`D` stream, dataset capture); it needs running and calibrating on the Pi. A ToF source would be added only if a ToF camera is connected.
4. New PyQt6 GUI (dark theme, always-visible E-STOP, camera tab, settings saved to a file).
5. Calibration tools (gearbox/microsteps, home angles, camera→robot transform).
6. Automated harvesting state machine (tested in simulation first).
7. Firmware: acceleration, vacuum command, safer homing, debounced limit switches.
8. Full documentation and final push.

## Credits and licences

* **Ultralytics YOLO** — AGPL-3.0. The Pi vision server imports it as a library. AGPL-3.0 has
  obligations when software built on it is shared or offered over a network; decide the licence of
  this project deliberately. *(This is not legal advice.)* Training/export notes will come with the model phase.
* **PyQt-on-camera / "PC CamTouch"** (hoangnv170752) — MIT licence. Used as a *reference* for
  GUI structure and style once the PyQt6 GUI is built.
* **Arducam_tof_camera** (ArduCAM) — the repository has no licence file, so it is used as a
  *reference only*; no code is copied. The `ArducamDepthCamera` Python package is installed from PyPI.
* The original GUI, firmware and camera client are from the `greatroboticslab/deltarobot` project.

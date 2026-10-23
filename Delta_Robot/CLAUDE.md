# CLAUDE.md — Delta Robot (strawberry harvester: depth camera + YOLO)

Read this first in every new session. Keep it updated as work proceeds. Human-friendly overview: `README.md`.

## Goal and architecture
A PC GUI picks strawberries with a 3-motor delta robot. **Pi = vision only. PC GUI = the ONLY thing that controls the ESP32.**
```
cameras -> Pi 5 (deltapi.local, user deltarobot): pi/pi_camera_server.py --TCP 8485 J/V/D--> PC: gui/ --USB serial--> ESP32 -> 3x DM542T -> 3x NEMA23
```
Planned: ESP32 moves to Wi-Fi later -> keep the GUI's serial layer behind a small link class that can be swapped for TCP.

## Safety rules (always)
* NEVER run anything that moves the robot, and NEVER flash the ESP32, without asking and waiting for confirmation.
* Keep/restore GUI safety: workspace box (NOT in the current GUI -> port from archive v3), IK-unreachable check, `busy` flag,
  `xyz_valid` after raw moves, E-STOP visible and prominent on every screen.
* Automation only for allowed strawberry classes; refuse models without one (stock YOLO sees people).
* The GUI "E-STOP" is software only (sends STOP; the user's current "emergency" is the Disable-Motors button, which lets arms drop).
  A hardware E-stop cutting driver power is planned. Never claim the software button is a safety-rated stop.
* Ask before sudo, apt, config-file edits, reboots, flashing, pushing to main / opening PRs.
* No secrets, weights, datasets or videos in git. Simulated ESP32 + simulated camera first for all GUI/automation features.

## Decisions made
* **CAMERA FINDING (supersedes the ToF decision):** the Pi's inspection (user's PowerShell paste) shows NO ToF camera. Only
  `lsusb` 32e4:9282 "3D Global Shutter Camera" (UVC, /dev/video0): dual ~1 MP global-shutter stereo, side-by-side modes
  2560x800/2560x720/1600x600/1280x480/640x240 (MJPG up to 120 fps; YUYV <=15 fps at 1280x480). No CSI camera. Pi = Debian 13
  trixie, Python 3.13.5, SSH over the Ethernet-shared link 192.168.137.128 (use the IP, not deltapi.local; it can change).
  Plan: stereo path, config camera 1280x480 MJPG, re-calibrate at that size. Keep ToF code optional. Still ask the user to
  confirm whether a ToF camera exists. (The user could not install Claude Code on the PC; work proceeds by copy-paste of Pi output.)
* Camera: build around the **Arducam ToF (240x180, HQVGA)** first (user may use USB or ToF; CSI vs USB still unchecked).
  A **1.0 MP USB camera (model no. 202511028880000100, unidentified otherwise)** comes later -> make the capture source pluggable,
  so the detection image can be ToF amplitude now or a USB image + aligned depth later. Mounting: top-down or slightly angled.
* Geometry (user decision): f = 3 in = 76.2 mm (final TBD); rf/re/e = current GUI values 150/280/50 mm (older v3 said 254/342.9/76.2;
  measure once). Make them settings. Gearbox 47:1. DM542T DIP: SW3,4,7,8 ON (others OFF) -> by the standard table 3200 pulses/rev
  (16 microsteps), ~2.37 A peak -> **150 400 steps per arm revolution = 417.78 steps/deg** (verify the DIP table on the driver label;
  confirm all 3 drivers identical). Current Tkinter GUI default Steps/rev=3200 is wrong (47x too small).
* Camera decision (user): **ToF** ("we are doing tof"). Baseline = repo GitHub versions ("use the repo, it will be fine"); still add the
  workspace box (user's safety list requires it). Baseline code = the GitHub versions (user confirmed they are the most current): gui/Delta_Robot_GUI.py (Tkinter, no workspace
  box, no camera tab) and the ESP32 firmware (homing dir +1, `Z` replies HOMED, no busy check on Z/EN).
* Folder is `Delta_Robot/` (capitalised, per user) inside CDJ7/Irrigation-Robot-Code. Don't touch anything else in that repo
  (irrBot-version1.ino and file `2` there are an unrelated MQTT robot with Wi-Fi credentials committed).
* Arducam SDK: PyPI wheel `ArducamDepthCamera` 0.1.24 is self-contained (aarch64 cp38-cp314, needs numpy<2 -> opencv-python-headless<4.12).
  Python API: Connection.CSI|USB, FrameType.DEPTH, DepthData.depth_data(mm)/confidence_data/amplitude_data,
  Control.RANGE/HFLIP/VFLIP/INTRINSIC_FX,FY,CX,CY (value/100). Its Install_dependencies.sh edits /boot/firmware/config.txt, sets
  camera_auto_detect=0, adds a 3rd-party apt repo, uses `sudo pip` -> do not run blindly. Verify on hardware: RANGE units
  (examples disagree: 4000 vs 4), whether HQVGA confidence is meaningful, depth = Z vs radial, image mirroring (must be right-handed).
* Licences: Ultralytics AGPL-3.0; pyqt-on-camera MIT (PC CamTouch Team); Arducam_tof_camera repo has NO licence -> reference only.

## Environment notes
* Sessions so far ran on a cloud Linux VM: **no ssh to the Pi, cannot run anything on the Pi, no Windows**. Anything needing the
  Pi / ToF / ESP32 is untested. References live in `<repo>/third_party/` (excluded via .git/info/exclude; never commit).
* Source repo clone (read only): /home/user/greatroboticslab/deltarobot.

## Status
| Phase | State |
|---|---|
| 1 Review | done -> docs/PHASE1_REVIEW.md |
| (scaffold) Delta_Robot folder + README | done |
| Earlier stereo-USB Pi server (pi/) | written; passes synthetic tests only; NOT run on the Pi |
| 2 Pi set-up (stereo path now; ToF only if one turns up) + VNC | tools written + tested vs stand-in SDK (pi/tof/, docs/PI_SETUP.md); **waiting for the user to run them on the Pi** (no Pi access from here; sudo/boot-config/reboot need approval) |
| 3 ToF vision server | not started |
| 4 PyQt6 GUI | not started |
| 5 Calibration tools | not started |
| 6 Harvest state machine | not started |
| 7 Firmware | not started |
| 8 Docs + final push | partly (README, docs); final push needs user's choice main vs branch/PR |

## Calibration status
Stereo: n/a for ToF. Intrinsics: from SDK (to verify). Home/offsets/HOME_XYZ: uncalibrated (HOME_XYZ placeholder (0,0,-200)).
Camera-to-robot transform: not done. Microsteps, final f, rf/re/e: TBD.

## How to run (today)
See README.md ("Running what exists today") and pi/README.md. Self-tests: `cd pi && python tests/test_server_fake.py` (+ the other two).

## Open questions for the user
Confirm DIP table vs driver label + all 3 drivers identical; ToF on CSI or USB (Pi output of docs/PI_SETUP.md step 1+3); vacuum pin when firmware phase starts;
repo licence; main vs branch+PR for the final push.

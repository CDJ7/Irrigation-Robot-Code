# Phase 1 review — bugs and risks in the original code

Findings from reading `Delta_Robot_GUI.py`, `Delta_Robot_ESP32.ino`, the Pi server/client files and the reference
repositories. **Nothing here was tested on the real robot.** Ranked by how much harm they can do.

## P0 — can hurt people or the machine
1. **The software "EMERGENCY STOP" is not a real stop.** It sends `STOP` over USB serial. If the cable drops, the PC hangs,
   or the ESP32 is inside its blocking homing loop, nothing stops. Add a hardware E-stop that cuts driver power.
2. **Homing is unvalidated** (the GUI header says "The Home Button Does NOT Work yet"; the firmware's homing direction was
   flipped by trial). It can drive arms into their end stops.
3. **The current GUI has no workspace limits** — any typed XYZ goes to the kinematics; only "unreachable" is checked.
   (An older GUI had a workspace box; it should be restored.)
4. **"Camera Tracking" is a random-point generator** that moves the robot every second while enabled.
5. **`EN 0` and `Z` are accepted while moving**; disabling drivers drops a loaded arm; `Z` mid-move corrupts the position.
6. **`OK` is ambiguous**: `EN` also replies `OK`, so clicking Enable/Disable during a move looks like "move finished".
   The GUI then forgets it is busy.
7. **Stale position after errors**: on `ERR STOPPED`, `ERR BUSY`, homing errors the GUI clears its pending state without
   marking the XYZ position invalid. `ERR BUSY` even clears `busy` while the robot is still moving.
8. **`busy` can stick forever**: the GUI sets it before sending; if the port is not connected (or the reply is lost) it
   never clears. There is no timeout.
9. **Uncalibrated geometry**: `steps_per_rev = 3200` ignores the 47:1 gearbox; base radius `f` was a placeholder (now 3 in);
   `rf/re/e` disagree between file versions; `HOME_XYZ` and per-axis offsets are placeholders although step 0 is the
   limit-switch position while kinematics work in real arm angles.

## P1 — vision and communication
10. `pi_camera_server.py` (original): fake depth; no YOLO; opens the camera once **per client**; requests 640×480; systemd
    example uses `/home/pi`; recommends the obsolete `bcm2835-v4l2`.
11. `camera_client.py` (original): `is_connected()` stays true after a drop; no reconnect; `J` messages would be dropped;
    unbounded frame queue; `connect()` blocks the UI thread; no length sanity check.
12. The video overlay (earlier GUI) mapped the workspace box onto the picture instead of using a camera calibration.
13. `main_harvester.py` moves the robot on any YOLO detection (including people), shares the serial port with the GUI,
    has placeholder kinematics, uncalibrated stereo, and `numDisparities=16`.

## P2 — firmware quality
14. No acceleration → missed steps under load.
15. Homing discards every serial line except `STOP`; no limit-switch debouncing; homing does not check drivers are enabled.
16. `micros()` wraps every ~71 minutes; a move started just before the wrap can stall (64-bit tick counter vs 32-bit clock).
17. Moves are straight lines in **step space**, not in XYZ: a "straight" descent is slightly curved.
18. No vacuum command.

## Arducam ToF notes (to verify on the real camera)
* The vendor `Install_dependencies.sh` edits `/boot/firmware/config.txt`, sets `camera_auto_detect=0`, adds a third-party apt
  repository, and uses `sudo pip`. The PyPI wheel `ArducamDepthCamera` is self-contained and installs in a venv (needs `numpy<2`).
* The vendor examples disagree on the `RANGE` control (`4000` vs `4`); the confidence control is offered only for the VGA model;
  depth may be along the optical axis or along the ray; the image may be mirrored. All must be checked on the hardware.
* ToF cameras struggle in direct sunlight and on glossy/wet or very dark surfaces.

## Camera choice
* **(a)** YOLO on the ToF IR amplitude image — one sensor, depth perfectly aligned, but 240×180 and IR only (no ripeness colour).
* **(b)** YOLO on a separate camera + ToF depth — more pixels (and colour if the camera is colour), but needs an extra
  camera-to-ToF calibration. **Decision: start with (a), keep the capture source pluggable so a USB camera can be added.**

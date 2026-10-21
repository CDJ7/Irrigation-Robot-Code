# archive/ — untouched originals

Backups of every file as it was received, before any changes. Nothing here is used by the running
system. Keep it so you can always compare against, or return to, the original code.

`original_files/`

| File | What it is |
|---|---|
| `Delta_Robot_GUI.py` | The Tkinter GUI as it is on GitHub (`greatroboticslab/deltarobot`). Same as `gui/Delta_Robot_GUI.py` until that is rebuilt. |
| `Delta_Robot_ESP32.ino` | The ESP32 firmware as it is on GitHub (homing direction `+1`; `Z` replies `HOMED`). |
| `delta_robot_combined_v3.py` | A larger, **older-or-parallel** GUI version: has the **workspace box**, a simulated depth map / point-of-interest tour, and a "Live Camera (Pi)" tab. Its workspace-box safety check is to be ported into the new GUI. |
| `delta_robot_esp32_older_with_Z_ZEROED.ino` | An earlier firmware: homing direction `-1`, and `Z` refuses to run while moving and replies `ZEROED`. |
| `camera_client.py` | Original GUI-side stream client. |
| `pi_camera_server_original_fake_depth.py` | Original Pi server. Its "depth" is a fake colourised brightness image; it opens the camera once per client. |
| `main_harvester.py` | Old all-in-one Pi script (YOLO + stereo + ESP32 serial + placeholder kinematics). It moves the robot on **any** YOLO detection, including people. **Do not run it.** |

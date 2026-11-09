# Calibration guide

Three separate things must be calibrated, in this order. Skipping one gives wrong millimetres, and the automatic picker refuses to run
until the checklist is green.

| # | What | Where | Why |
|---|---|---|---|
| 1 | **Stereo camera** (lens + the two eyes) | Pi, printed checkerboard | camera pixels -> millimetres |
| 2 | **Robot** (geometry, steps, home angles) | GUI -> Calibration -> Machine / Home && axes | XYZ <-> motor steps |
| 3 | **Camera -> robot** | GUI -> Calibration -> Camera -> robot | a point the camera sees -> where the effector must go |

## 0. Measure the machine (once, with a ruler or calipers)
Everything in the kinematics comes from these. The values in the settings are **guesses** until you measure them.
* **Base radius** (centre of the base to a motor axis): 76.2 mm (3 in) for now.
* **Effector radius** (centre of the effector to a forearm joint): 50 mm, from the old GUI, not verified.
* **Bicep length** (shoulder to elbow): 150 mm. **Forearm length** (elbow to effector joint): 280 mm. The old v3 GUI used 254 / 342.9 mm: measure.
* **Steps per degree of arm** = motor full steps (200) x microsteps x gearbox (47) / 360. DM542T DIP switches 3, 4, 7, 8 ON = 3200 pulses/rev by the
  standard table, giving 150 400 steps per arm turn = **417.78 steps/degree**. **Check the DIP table printed on the driver** and that all three drivers match.
* **Arm angle limits** (theta min / max): the real mechanical range, with margin. Defaults -30..80 degrees are UNVERIFIED.

## 1. Stereo calibration (on the Pi)
Full command list: `pi/README.md` section 5. In short:
1. Print `pi/checkerboard_9x6_25mm_letter.svg` at **100 %**, glue it flat on a rigid board, **measure one square** with a ruler.
2. Stop the service (`sudo systemctl stop delta-vision`), then `python pi_camera_server.py --calibrate-capture`, and watch it in the GUI **Camera** page
   (or `http://<pi>:5000`). Tilt and move the board until it says DONE (30 pairs, all nine picture regions covered). Ctrl+C.
3. `python calibrate_stereo.py solve --square-mm <measured> --work-range 550 1000` (use your near/far distances in mm).
   Aim for stereo RMS under about 0.5 px. Check the baseline against a ruler (the real camera gave 62.2 mm).
4. **Ruler test:** put a flat, textured object at a measured distance and compare with the Camera page's centre distance. More than about 2 % off: redo it.
Precision to expect at 70 cm is roughly 11 mm per pixel of disparity, with typical errors of 3-6 mm. The left edge of the picture (about one search range wide)
has no depth by design.

## 2. Robot calibration (GUI -> Calibration)
**Machine tab:** enter the measured geometry, microsteps and gearbox, then Apply. **Always connect with the motor power switch within reach.**

**Home && axes tab (home wizard):**
1. *Arm directions.* For each arm press Nudge (it turns the arm a few degrees; the GUI asks first). Say whether the arm moved UP or DOWN. An arm that moved up is
   inverted: the wizard ticks Invert for you. The position becomes unknown after a raw move, so home again afterwards.
2. *Home angles.* With the arms resting at the home position (against the limit switches), measure each bicep angle with an angle gauge: 0 degrees = bicep horizontal,
   positive = down. Enter the three numbers, press Compute: you get the **angle offsets** and the **HOME XYZ** from forward kinematics. Use these values, then Apply and save.
3. Home the robot. The log warns if the home XYZ and the offsets disagree.

## 3. Camera -> robot calibration (GUI -> Calibration -> Camera -> robot)
You need: a homed robot, a connected and **stereo-calibrated** camera, and a small high-contrast marker (a dot) on the effector that you can click in the left picture.
1. Press **Suggest points**: it lists 8 well-spread positions inside the workspace box (not all at one height).
2. For each row: **Go** (asks first, then moves the robot), click the marker in the live picture, **Record**. The wizard averages 8 readings and refuses if there is no depth,
   if the readings jump by more than 4 mm, or if the robot moved while measuring.
3. **Solve.** You get the RMS error, the error per point, and warnings (outlier points, all points at one height, a MIRRORED camera frame). Aim for RMS of a few millimetres.
   Remove outliers and solve again. **Accept** saves it (`settings.json`). The Camera page then shows robot X/Y/Z for every detection.
4. Re-do this whenever the camera or the robot base is bumped or moved. **Forget** clears it, and the harvester will refuse to run.
The camera sees the *front surface* of a berry, not its centre: tune `grasp_offset_mm` on the Harvest page on the real machine.

## Quick sanity checks
* Home, then jog +X by 10 mm: does the effector move 10 mm to the +X side? Same for Y and Z (Z up = larger). If a direction is flipped, fix Invert, not the maths.
* Move to the middle of the workspace, then to a corner and back: it should return to the same spot. Drift means lost steps (lower the step rate or acceleration, check driver current).

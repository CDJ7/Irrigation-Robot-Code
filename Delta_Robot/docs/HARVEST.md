# Automatic picking (Phase 6)

Code: `gui/delta_gui/core/harvest.py` (state machine, no Qt, fully tested) and `gui/delta_gui/ui/pages/harvest.py` (the page).
Tests: `gui/tests/test_core_harvest.py` (10 scenarios against the simulated ESP32 + a simulated camera feed) and `gui/tests/test_gui_harvest.py`.
A complete simulated run is in `docs/HARVEST_SIM_LOG.txt` (regenerate with `python gui/tests/demo_harvest_sim.py`).

**Status: tested in simulation only. It has never moved the real robot.** The vacuum commands need the Phase 7 firmware.

## What a run does
1. **Pre-flight checklist** (all must be green before Start is enabled): ESP32 connected, E-STOP not active, motors enabled, position known,
   robot idle, home angles calibrated, camera->robot calibration valid, allowed classes set, camera sending frames, stereo depth
   calibrated, **detection model has a strawberry class**, drop point reachable and inside the workspace box.
2. **Search.** A detection counts only if its class is on the GUI's allowed list *and* the Pi marked it allowed, its confidence and
   depth quality are high enough and it has a 3D position. It must be seen in `stable_frames` consecutive frames within `max_jitter_mm`.
3. **Choose** the stable berry closest to the robot's centre line. Its approach, pick and drop poses are checked with the same
   `check_target` as every manual move (workspace box, reachable, arm-angle limits); a berry that fails is skipped and the reason logged.
4. **Pick sequence:** approach (hover above) -> descend -> vacuum ON + dwell -> lift -> carry to drop -> vacuum OFF + wait. Each move is
   `Robot.move_xyz`, so busy flag, position-valid and drivers-enabled checks all still apply. After each move the position is verified.
5. **Ends** when: max picks reached, nothing found for `idle_timeout_s`, the camera goes silent, any move is refused or does not
   complete, the ESP32 resets or the link drops, the user presses **Stop** (finishes the current step, then vacuum OFF), or the
   **E-STOP** (immediate: `STOP`, then `V 0`, then drivers off; the run is marked STOPPED).

## Safety properties (each has a test)
* Stock YOLO (no strawberry class) -> refused before anything is sent. A person detection can never become a target.
* Both class checks must agree (our list AND the Pi's `allowed` flag).
* The vacuum is always switched off when a run ends for any reason; if the command cannot be sent the log says so loudly.
* `vacuum_enabled` is **off by default** = DRY RUN (moves only). Enabling it asks for confirmation and warns that the current firmware has no `V` command (it would answer `ERR`, and the run stops).
* Start asks for confirmation every time, listing mode, classes and drop point.

## Settings (Harvest page, saved in settings.json under `harvest`)
`allowed_classes`, `min_confidence`, `min_depth_valid_fraction`, `stable_frames`, `max_jitter_mm`, `match_radius_mm`,
`approach_height_mm`, `grasp_offset_mm` (the camera sees the berry's front surface, not its centre), `dwell_s`, `release_s`,
`drop_xyz_mm`, `max_picks`, `idle_timeout_s`, `frame_timeout_s`, `vacuum_enabled`. All are starting guesses.

## Known limits
* No pick verification (no vacuum-pressure sensor): "completed" means "the sequence ran".
* After dropping, the effector stays at the drop point; if it blocks the camera's view, move the drop point or add a ready pose.
* The robot moves in a straight line in motor-step space (no acceleration yet; Phase 7), not a straight line in XYZ.
* Every attempt is appended to `harvest_log.jsonl` next to the settings file.

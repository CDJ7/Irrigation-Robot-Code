"""
settings.py -- every setting and calibration value, saved to ONE json file that loads on start-up.

Location: ~/.delta_robot/settings.json  (set the environment variable DELTA_ROBOT_SETTINGS to use another file).
The file is written atomically (temp file, then rename), so a crash while saving cannot leave a half-written file; an unreadable
file is renamed to *.broken and defaults are used, so the program always starts.
"""
import copy
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional

DEFAULTS: Dict[str, Any] = {
    "version": 1,
    "geometry": {                       # all millimetres. RADII (what you measure), NOT the triangle side lengths.
        "base_radius_mm": 76.2,         # centre of the base -> motor axis (3 in). Final design not decided.
        "effector_radius_mm": 50.0,     # centre of the effector -> forearm joints.  UNVERIFIED (from the old GUI's 'e')
        "bicep_mm": 150.0,              # upper arm: shoulder -> elbow.               UNVERIFIED (from the old GUI)
        "forearm_mm": 280.0,            # forearm: elbow -> effector joint.            UNVERIFIED (from the old GUI)
    },
    "drive": {
        "motor_full_steps": 200,        # NEMA 23: 1.8 deg
        "microsteps": 16,               # DM542T DIP switches 3,4,7,8 on = 3200 pulses/rev (standard table; verify on the driver)
        "gearbox_ratio": 47.0,
        "max_speed_mm_s": 80.0,         # tip speed used to compute move durations
        "max_step_rate": 4000.0,        # steps/second per axis: no acceleration in the firmware yet, so start LOW
    },
    "axes": {
        "offsets_deg": [0.0, 0.0, 0.0],
        "invert": [False, False, False],
        "theta_min_deg": -30.0,         # allowed arm-angle range. UNVERIFIED: set to the real mechanical limits.
        "theta_max_deg": 80.0,
    },
    "home": {"xyz_mm": [0.0, 0.0, -200.0], "calibrated": False},   # placeholder until the home calibration is done
    "workspace_mm": {"x": [-100.0, 100.0], "y": [-100.0, 100.0], "z": [-350.0, -210.0]},   # fully reachable with the defaults
    "safety": {
        "estop_disables_drivers": True,     # E-STOP = STOP, then drivers off (your "motor off" button). Arms may sag.
        "confirm_homing": True,
        "confirm_raw_move_deg": 10.0,       # ask before a raw move bigger than this on any axis
        "reply_margin_s": 3.0,              # extra time allowed beyond a move's duration before it counts as lost
        "home_timeout_s": 240.0,
    },
    "connection": {"port": "", "baud": 115200, "simulate_esp32": False},
    "camera": {"host": "192.168.137.128", "port": 8485, "simulate": False, "approach_height_mm": 60.0,
               "show_effector_crosshair": True},
    "harvest": {                        # automatic picking (Phase 6). Every number here is a starting guess: tune on the real machine.
        "allowed_classes": ["strawberry"],     # ONLY these YOLO classes may ever be picked (stock YOLO has none -> refuses)
        "min_confidence": 0.5,
        "min_depth_valid_fraction": 0.5,       # share of pixels in the box that had a valid depth
        "stable_frames": 5,                    # a berry must be seen in this many consecutive frames ...
        "max_jitter_mm": 6.0,                  # ... at a position that stays within this many mm
        "match_radius_mm": 20.0,               # detections closer than this to a track belong to it
        "approach_height_mm": 60.0,            # hover this far ABOVE the berry before descending
        "grasp_offset_mm": [0.0, 0.0, 0.0],    # added to the berry position (the camera sees the front surface, not the centre)
        "dwell_s": 0.8,                        # wait after vacuum on, before lifting
        "release_s": 0.6,                      # wait after vacuum off at the drop point
        "drop_xyz_mm": [90.0, 0.0, -250.0],    # where picked berries are dropped (must be inside the workspace)
        "max_picks": 5,                        # stop after this many pick attempts (safety limit per run)
        "idle_timeout_s": 30.0,                # stop if no stable target appears for this long
        "frame_timeout_s": 2.0,                # stop if the camera goes silent for this long
        "vacuum_enabled": False,               # False = DRY RUN: all moves, no vacuum commands (the firmware has no vacuum yet)
    },
    "camera_to_robot": None,            # filled by the calibration: {"R": 3x3, "t": 3, "valid": ..., "rms_mm": ...}
    "ui": {"page": "operations"},
}


def default_path() -> Path:
    env = os.environ.get("DELTA_ROBOT_SETTINGS")
    return Path(env) if env else Path.home() / ".delta_robot" / "settings.json"


def _merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    """Defaults first, then whatever the file has; unknown keys are kept, missing keys get their default."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def validate(s: Dict[str, Any]) -> Optional[str]:
    """Return a message describing the first problem, or None if the settings are usable."""
    try:
        g, d, a = s["geometry"], s["drive"], s["axes"]
        for key in ("base_radius_mm", "effector_radius_mm", "bicep_mm", "forearm_mm"):
            if not (0.0 < float(g[key]) < 5000.0):
                return f"geometry.{key} must be between 0 and 5000 mm"
        if float(g["forearm_mm"]) <= float(g["effector_radius_mm"]):
            return "the forearm must be longer than the effector radius"
        if not (int(d["motor_full_steps"]) > 0 and int(d["microsteps"]) > 0 and float(d["gearbox_ratio"]) > 0):
            return "motor steps, microsteps and gearbox ratio must all be positive"
        if not (0.0 < float(d["max_speed_mm_s"]) <= 2000.0 and 0.0 < float(d["max_step_rate"]) <= 200000.0):
            return "max speed / max step rate out of range"
        if float(a["theta_min_deg"]) >= float(a["theta_max_deg"]):
            return "theta_min must be smaller than theta_max"
        if len(a["offsets_deg"]) != 3 or len(a["invert"]) != 3:
            return "axes need exactly three offsets and three invert flags"
        for ax in ("x", "y", "z"):
            lo, hi = s["workspace_mm"][ax]
            if not float(lo) < float(hi):
                return f"workspace {ax}: low must be smaller than high"
        if len(s["home"]["xyz_mm"]) != 3:
            return "home needs X, Y and Z"
        h = s["harvest"]
        if not isinstance(h["allowed_classes"], list) or not all(isinstance(c, str) and c for c in h["allowed_classes"]):
            return "harvest.allowed_classes must be a list of class names"
        if len(h["drop_xyz_mm"]) != 3 or len(h["grasp_offset_mm"]) != 3:
            return "harvest drop position and grasp offset need X, Y and Z"
        if not (0.0 <= float(h["min_confidence"]) <= 1.0 and int(h["stable_frames"]) >= 1 and int(h["max_picks"]) >= 1):
            return "harvest confidence must be 0..1, stable_frames and max_picks at least 1"
        if not (0.0 <= float(h["approach_height_mm"]) <= 300.0):
            return "harvest approach height must be 0..300 mm"
    except (KeyError, TypeError, ValueError) as ex:
        return f"settings are incomplete or malformed ({ex})"
    return None


class Settings:
    def __init__(self, path: Optional[Path] = None):
        self.path = Path(path) if path else default_path()
        self.data: Dict[str, Any] = copy.deepcopy(DEFAULTS)
        self.load_message = ""

    def load(self) -> None:
        if not self.path.exists():
            self.load_message = f"no settings file yet; using defaults ({self.path})"
            return
        try:
            loaded = json.loads(self.path.read_text(encoding="utf-8"))
            merged = _merge(DEFAULTS, loaded)
            problem = validate(merged)
            if problem:
                raise ValueError(problem)
            self.data = merged
            self.load_message = f"loaded {self.path}"
        except (OSError, ValueError, json.JSONDecodeError) as ex:
            broken = self.path.with_suffix(".broken")
            try:
                self.path.replace(broken)
            except OSError:
                pass
            self.data = copy.deepcopy(DEFAULTS)
            self.load_message = f"settings file unusable ({ex}); moved to {broken.name}, using defaults"

    def save(self) -> None:
        problem = validate(self.data)
        if problem:
            raise ValueError(problem)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent), suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(self.data, fh, indent=2)
            os.replace(tmp, self.path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # convenient derived values -------------------------------------------------
    @property
    def steps_per_rev(self) -> float:
        d = self.data["drive"]
        return float(d["motor_full_steps"]) * float(d["microsteps"]) * float(d["gearbox_ratio"])

    @property
    def steps_per_deg(self) -> float:
        return self.steps_per_rev / 360.0

"""
calibration.py -- the logic behind the two guided calibrations (no GUI, no hardware: fully testable).

1. HOME calibration.  After homing, the arms stand at the limit switches and the controller calls that pose "step 0". To turn XYZ
   targets into steps the program must know the REAL angle of each arm at that pose. You measure it with an angle gauge (angle of the
   bicep against horizontal; positive = swung DOWN, negative = raised above horizontal -- the same sign the kinematics use). From the
   three angles:   offset_i = -angle_i      (step 0 <=> angle_i)        and       HOME XYZ = forward_kinematics(angles).

2. ARM DIRECTION.  Does a POSITIVE step move an arm down (invert = False) or up (invert = True)? Each arm is nudged a few degrees and
   you answer which way it went.

3. CAMERA -> ROBOT.  The effector carries a target mark. For each of several robot positions the camera measures where the mark is
   (camera XYZ) and the robot knows where it is (robot XYZ). The best rotation + translation for the pairs is the Kabsch solution
   (transform.py). This module manages the list of pairs, suggests well-spread positions, averages noisy camera readings and judges
   the result.
"""
import itertools
import math
import statistics
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .kinematics import DeltaKinematics, Workspace, joint_limit_violation
from .transform import CameraToRobot, solve_rigid

Triple = Tuple[float, float, float]


# ======================================================================================================================
#   home calibration
# ======================================================================================================================
@dataclass
class HomeResult:
    offsets_deg: List[float]
    home_xyz: Triple
    warnings: List[str] = field(default_factory=list)


def home_calibration(measured_angles_deg: Sequence[float], kin: DeltaKinematics, theta_min: float, theta_max: float,
                     workspace: Optional[Workspace] = None) -> HomeResult:
    """Offsets and HOME XYZ from the three measured home angles. Raises ValueError with a plain message if the angles are impossible."""
    if len(measured_angles_deg) != 3 or not all(math.isfinite(a) for a in measured_angles_deg):
        raise ValueError("enter all three measured angles")
    a = [float(v) for v in measured_angles_deg]
    why = joint_limit_violation(a, theta_min, theta_max)
    if why:
        raise ValueError(f"{why}. Check the sign (positive = arm below horizontal) or widen the arm-angle limits on the Machine tab.")
    xyz = kin.forward(*a)
    if xyz is None:
        raise ValueError("these three angles cannot belong to one pose: the forearms would not meet at a point. Check the measurements "
                         "and the arm lengths / radii.")
    warnings: List[str] = []
    spread = max(a) - min(a)
    if spread > 3.0:
        warnings.append(f"the three home angles differ by {spread:.1f} deg. All three arms normally stand at the same angle at home; "
                        "re-measure, or check that all limit switches trigger at the same position.")
    if abs(xyz[0]) > 5 or abs(xyz[1]) > 5:
        warnings.append(f"the home pose is {math.hypot(xyz[0], xyz[1]):.0f} mm off the centre line.")
    if workspace is not None and not workspace.contains(*xyz):
        warnings.append("the home position lies outside the workspace box. That is allowed (the robot homes there), but XYZ moves "
                        "must stay inside the box.")
    return HomeResult([-v for v in a], (xyz[0], xyz[1], xyz[2]), warnings)


def nudge_steps(deg: float, steps_per_deg: float) -> int:
    return int(round(deg * steps_per_deg))


def invert_from_answer(moved_down: bool) -> bool:
    """The arm was nudged with POSITIVE steps. If it moved DOWN, positive steps = positive angle: no inversion needed."""
    return not moved_down


# ======================================================================================================================
#   camera -> robot calibration
# ======================================================================================================================
@dataclass
class CalPoint:
    robot_xyz: Triple
    cam_xyz: Triple
    n_samples: int = 1
    spread_mm: float = 0.0
    note: str = ""


class ProbeCollector:
    """Averages repeated camera readings of the same point: the median per axis, plus the spread, so a noisy reading is noticed."""
    def __init__(self, n: int = 8, max_spread_mm: float = 4.0):
        self.n, self.max_spread = n, max_spread_mm
        self.samples: List[Triple] = []

    def feed(self, xyz: Optional[Sequence[float]]) -> None:
        if xyz is not None and all(math.isfinite(v) for v in xyz):
            self.samples.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))

    @property
    def done(self) -> bool:
        return len(self.samples) >= self.n

    def result(self) -> Tuple[Triple, float]:
        if not self.samples:
            raise ValueError("no valid camera readings (no depth at that spot?)")
        med = tuple(statistics.median(s[i] for s in self.samples) for i in range(3))
        spread = max(math.dist(s, med) for s in self.samples)
        return (med[0], med[1], med[2]), spread

    def acceptable(self) -> Tuple[bool, str]:
        med, spread = self.result()
        if spread > self.max_spread:
            return False, (f"the readings jump around by up to {spread:.1f} mm. Click the middle of a high-contrast mark that does not "
                           f"move, with good light, and try again.")
        return True, ""


@dataclass
class SolveReport:
    transform: CameraToRobot
    errors_mm: List[float]
    warnings: List[str]

    @property
    def rms_mm(self) -> float:
        return float(self.transform.rms_mm)


class CameraCalibrationSession:
    MIN_POINTS = 4

    def __init__(self) -> None:
        self.points: List[CalPoint] = []

    def add(self, robot_xyz: Sequence[float], cam_xyz: Sequence[float], n_samples: int = 1, spread_mm: float = 0.0) -> CalPoint:
        p = CalPoint(tuple(float(v) for v in robot_xyz), tuple(float(v) for v in cam_xyz), n_samples, spread_mm)  # type: ignore[arg-type]
        self.points.append(p)
        return p

    def remove(self, index: int) -> None:
        del self.points[index]

    def clear(self) -> None:
        self.points.clear()

    def solve(self) -> SolveReport:
        if len(self.points) < self.MIN_POINTS:
            raise ValueError(f"need at least {self.MIN_POINTS} points (8 or more recommended); have {len(self.points)}")
        cam = [p.cam_xyz for p in self.points]
        rob = [p.robot_xyz for p in self.points]
        tf, errs = solve_rigid(cam, rob)
        warnings: List[str] = []
        extents = np.ptp(np.asarray(rob), axis=0)
        if sorted(extents)[1] < 40.0:
            warnings.append("the points are spread over less than 40 mm in two of the three directions; spread them over the whole "
                            "working volume, including different heights, for an accurate result.")
        if extents[2] < 30.0:
            warnings.append("all points are at nearly the same height. Add points at a clearly different Z.")
        med = float(np.median(errs))
        for i, e in enumerate(errs):
            if e > max(3.0 * med, 6.0) and len(errs) > self.MIN_POINTS:
                warnings.append(f"point {i + 1} has an error of {e:.1f} mm, much more than the others (median {med:.1f} mm): "
                                "remove it and solve again, or re-measure it.")
        if tf.rms_mm > 25.0:
            warnings.append(f"the error is very large ({tf.rms_mm:.0f} mm). Most likely causes: the camera image is MIRRORED (a mirrored "
                            "camera frame cannot be matched by a rotation), wrong points were clicked, or the stereo calibration is off.")
        elif tf.rms_mm > 6.0:
            warnings.append(f"the error ({tf.rms_mm:.1f} mm RMS) is larger than a strawberry pick tolerates; check the points.")
        return SolveReport(tf, [float(e) for e in errs], warnings)


def suggest_points(check_target, workspace: Workspace, n: int = 8, shrink: float = 0.7) -> List[Triple]:
    """Well-spread robot positions for the calibration: the corners of a shrunken workspace box (so the effector is never driven to the
    edge), pulled toward the centre until each is allowed by `check_target(x, y, z) -> reason or None`, ordered to keep the travel short."""
    cx, cy, cz = (sum(a) / 2 for a in (workspace.x, workspace.y, workspace.z))
    hx, hy, hz = ((a[1] - a[0]) / 2 * shrink for a in (workspace.x, workspace.y, workspace.z))
    pts: List[Triple] = []
    for sx, sy, sz in itertools.product((-1, 1), repeat=3):
        t = 1.0
        while t > 0.05:
            p = (cx + sx * hx * t, cy + sy * hy * t, cz + sz * hz * t)
            if check_target(*p) is None:
                pts.append(p)
                break
            t -= 0.1
    pts = pts[:max(n, 4)]
    ordered, rest = [pts[0]] if pts else [], pts[1:]
    while rest:
        nxt = min(rest, key=lambda q: math.dist(q, ordered[-1]))
        ordered.append(nxt)
        rest.remove(nxt)
    return ordered

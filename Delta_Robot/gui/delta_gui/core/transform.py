"""
transform.py -- the rigid transform between the CAMERA frame and the ROBOT frame.

  p_robot = R @ p_camera + t          (R: 3x3 rotation, t: translation in mm)

It is found by touching the effector to points the camera can see and recording the pair (camera XYZ, robot XYZ) for each; the
best rotation and translation for the pairs is the classic Kabsch / SVD solution. (The guided procedure in the GUI comes in the
calibration phase; this file holds the maths and is fully tested.)  Camera frame: X right, Y down, Z out of the lens, in mm.
"""
import datetime
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class CameraToRobot:
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))
    valid: bool = False                 # False = never calibrated: do not trust or use
    rms_mm: Optional[float] = None
    n_points: int = 0
    date: str = ""

    def to_robot(self, p_cam: Sequence[float]) -> np.ndarray:
        return self.R @ np.asarray(p_cam, float) + self.t

    def to_camera(self, p_robot: Sequence[float]) -> np.ndarray:
        return self.R.T @ (np.asarray(p_robot, float) - self.t)

    def project(self, p_robot: Sequence[float], fx: float, fy: float, cx: float, cy: float) -> Optional[Tuple[float, float]]:
        """Where a robot-frame point appears in the camera picture (pixels), or None if it is behind the camera."""
        x, y, z = self.to_camera(p_robot)
        if z <= 1.0:
            return None
        return fx * x / z + cx, fy * y / z + cy

    def to_dict(self) -> dict:
        return {"R": self.R.tolist(), "t": self.t.tolist(), "valid": self.valid, "rms_mm": self.rms_mm,
                "n_points": self.n_points, "date": self.date}

    @staticmethod
    def from_dict(d: Optional[dict]) -> "CameraToRobot":
        if not d:
            return CameraToRobot()
        try:
            R, t = np.array(d["R"], float).reshape(3, 3), np.array(d["t"], float).reshape(3)
            if not (np.all(np.isfinite(R)) and np.all(np.isfinite(t))):
                raise ValueError
            return CameraToRobot(R, t, bool(d.get("valid")), d.get("rms_mm"), int(d.get("n_points", 0)), d.get("date", ""))
        except (KeyError, ValueError, TypeError):
            return CameraToRobot()


def solve_rigid(cam_pts: Sequence[Sequence[float]], robot_pts: Sequence[Sequence[float]]) -> Tuple[CameraToRobot, np.ndarray]:
    """Best rigid transform (rotation + translation, NO scaling or mirroring) taking camera points onto robot points.
    Returns (transform, per-point error in mm). Needs >= 3 points that are not on one line (>= 4 recommended, spread out)."""
    A, B = np.asarray(cam_pts, float), np.asarray(robot_pts, float)
    if A.shape != B.shape or A.ndim != 2 or A.shape[1] != 3 or len(A) < 3:
        raise ValueError("need matching lists of at least 3 XYZ points")
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    U, S, Vt = np.linalg.svd(H)
    if S[1] < 1e-6 * max(S[0], 1.0):
        raise ValueError("the points lie on a line; spread them out in more than one direction")
    d = np.sign(np.linalg.det(Vt.T @ U.T))          # guard against a reflection (a mirrored solution)
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cb - R @ ca
    errs = np.linalg.norm((A @ R.T + t) - B, axis=1)
    tf = CameraToRobot(R, t, True, float(np.sqrt(np.mean(errs ** 2))), len(A),
                       datetime.datetime.now().isoformat(timespec="seconds"))
    return tf, errs

"""
stereo_common.py -- checkerboard helpers shared by calibrate_stereo.py (offline solver) and the server's live
calibration-capture mode (calib_capture.py). No dependency on the server, so there is no circular import.
"""
from typing import Optional, Tuple

import cv2
import numpy as np

SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)


def parse_pattern(s: str) -> Tuple[int, int]:
    c, r = s.lower().split("x")
    return int(c), int(r)


def find_corners(gray: np.ndarray, pattern: Tuple[int, int], fast: bool = False) -> Optional[np.ndarray]:
    """Returns refined corners as a float32 array of shape (N,1,2), or None.
    `fast` is for the live capture loop. (Shape is normalized because different OpenCV
    versions return slightly different array shapes.)"""
    if not fast and hasattr(cv2, "findChessboardCornersSB"):
        # The "SB" detector is more robust on small, blurry images -- try it first.
        ok, c = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return np.asarray(c, np.float32).reshape(-1, 1, 2)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast:
        flags |= cv2.CALIB_CB_FAST_CHECK
    ok, c = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    c = np.asarray(c, np.float32).reshape(-1, 1, 2)
    return np.asarray(cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), SUBPIX), np.float32).reshape(-1, 1, 2)


def object_points(pattern: Tuple[int, int], square_mm: float) -> np.ndarray:
    """3D corner positions in the board's own frame (Z=0 plane), in mm."""
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_mm
    return objp


def pair_is_consistent(cl: np.ndarray, cr: np.ndarray) -> bool:
    """Both eyes must label corners in the same order. For a camera pair with the left eye on
    the left, a point appears further RIGHT in the left image than in the right image, so
    the mean (uL - uR) must be positive. A flipped board would give the wrong sign."""
    return float(np.mean(cl[:, 0, 0] - cr[:, 0, 0])) > 0.0

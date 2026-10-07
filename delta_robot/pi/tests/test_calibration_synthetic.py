"""
Tests calibrate_stereo.py (capture + solve) and the depth math using a SYNTHETIC checkerboard
seen by two virtual cameras with KNOWN parameters. Run:  python tests/test_calibration_synthetic.py

Ground truth used: focal 260/262 px, baseline 62 mm, right eye rotated ~1 degree relative to the
left, 320x240 per eye. After solving we check the recovered baseline, and that real 3D points
reconstructed from (u, v, disparity) with the Q matrix land where they should.
"""
import argparse
import os
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import calibrate_stereo as cs                      # noqa: E402
import pi_camera_server as srv                     # noqa: E402

W, H = 320, 240
K1 = np.array([[260, 0, 161.0], [0, 260, 119.0], [0, 0, 1]], np.float64)
K2 = np.array([[262, 0, 158.5], [0, 262, 121.0], [0, 0, 1]], np.float64)
rvec_true = np.radians([0.6, -0.5, 0.4])
R_TRUE = cv2.Rodrigues(rvec_true)[0]               # left-camera coords -> right-camera coords
T_TRUE = np.array([-62.0, 0.3, 0.2])               # right camera is 62 mm to the right of the left
SQ = 25.0
PATTERN = (9, 6)


def board_bitmap(px_per_mm=6, margin_mm=15):
    nx, ny = PATTERN[0] + 1, PATTERN[1] + 1
    img = np.full((int((ny * SQ + 2 * margin_mm) * px_per_mm), int((nx * SQ + 2 * margin_mm) * px_per_mm)), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                x0 = int((margin_mm + i * SQ) * px_per_mm)
                y0 = int((margin_mm + j * SQ) * px_per_mm)
                img[y0:y0 + int(SQ * px_per_mm), x0:x0 + int(SQ * px_per_mm)] = 0
    return img, px_per_mm, margin_mm


BMP, PPM, MARGIN = board_bitmap()


def render(K, R_cam, t_cam, rng):
    """Image of the board whose pose in the camera frame is (R_cam, t_cam). Board frame origin =
    the first inner corner, so bitmap (px) = (board_mm + margin + SQ) * PPM."""
    Hm = K @ np.column_stack([R_cam[:, 0], R_cam[:, 1], t_cam])       # board (mm) -> image
    S = np.array([[1 / PPM, 0, -(MARGIN + SQ)], [0, 1 / PPM, -(MARGIN + SQ)], [0, 0, 1]])  # bitmap px -> board mm
    img = cv2.warpPerspective(BMP, Hm @ S, (W, H), flags=cv2.INTER_AREA, borderValue=170)
    img = cv2.GaussianBlur(img, (3, 3), 0.6)
    return np.clip(img.astype(np.int16) + rng.integers(-3, 4, img.shape), 0, 255).astype(np.uint8)


class ScriptedCamera:
    """Feeds capture() a list of side-by-side frames, then raises KeyboardInterrupt (= user Ctrl+C)."""
    def __init__(self, frames):
        self.frames = list(frames)

    def read(self):
        if not self.frames:
            raise KeyboardInterrupt
        return self.frames.pop(0)


def make_frames(n, rng):
    frames = []
    for _ in range(n):
        yaw, pitch, roll = np.radians(rng.uniform(-30, 30, 3))
        Rb = cv2.Rodrigues(np.array([pitch, yaw, roll]))[0]
        # board center position in left camera frame (mm)
        center = np.array([rng.uniform(-60, 60), rng.uniform(-40, 40), rng.uniform(330, 520)])
        # board origin = first inner corner = center - Rb @ (half board size of the INNER grid)
        half = np.array([(PATTERN[0] - 1) * SQ / 2, (PATTERN[1] - 1) * SQ / 2, 0])
        tb = center - Rb @ half
        left = render(K1, Rb, tb, rng)
        Rr, tr = R_TRUE @ Rb, R_TRUE @ tb + T_TRUE
        right = render(K2, Rr, tr, rng)
        both = np.hstack([left, right])
        frames.append(np.dstack([both, both, both]))
    return frames


def main():
    rng = np.random.default_rng(3)
    tmp = tempfile.mkdtemp()
    img_dir = os.path.join(tmp, "images")
    cfgpath = os.path.join(os.path.dirname(HERE), "config.yaml")

    # ---- capture step with the real capture() code ----
    frames = make_frames(60, rng)
    ns = argparse.Namespace(board="9x6", square_mm=SQ, dir=img_dir, config=cfgpath, count=30,
                            interval=0.0, min_move=3.0)
    cs.capture(ns, camera=ScriptedCamera(frames))
    saved = len([f for f in os.listdir(img_dir) if f.endswith("_L.png")])
    print("pairs saved by capture():", saved)
    assert saved >= 20, "capture should save most synthetic views"

    # ---- solve step ----
    out = os.path.join(tmp, "calibration", "stereo_calib.npz")
    ns = argparse.Namespace(board="9x6", square_mm=SQ, dir=img_dir, config=cfgpath, out=out, alpha=0.0)
    assert cs.solve(ns) == 0
    d = np.load(out)
    baseline = float(-d["P2"][0, 3] / d["P2"][0, 0])
    print(f"recovered baseline {baseline:.2f} mm (truth 62.0); rms {float(d['rms']):.3f}px")
    assert abs(baseline - 62.0) < 0.6, baseline
    assert float(d["rms"]) < 0.5

    # ---- depth math: known 3D points -> rectified pixels -> Q -> XYZ ----
    rect = srv.StereoRectifier({"stereo": {"calibration_file": out, "nominal_focal_px": 1,
                                           "nominal_baseline_mm": 1}}, (W, H))
    assert rect.calibrated
    for X in ([50, -20, 500], [-80, 30, 350], [0, 0, 800], [100, 60, 300]):
        X = np.array(X, np.float64)
        pl, _ = cv2.projectPoints(X.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K1, None)
        Xr = R_TRUE @ X + T_TRUE
        pr, _ = cv2.projectPoints(Xr.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K2, None)
        ul = cv2.undistortPoints(pl.reshape(1, 1, 2), K1, None, R=d["R1"], P=d["P1"]).reshape(2)
        ur = cv2.undistortPoints(pr.reshape(1, 1, 2), K2, None, R=d["R2"], P=d["P2"]).reshape(2)
        assert abs(ul[1] - ur[1]) < 0.3, "rows must match after rectification"
        disp = ul[0] - ur[0]
        got = np.array(rect.to_xyz(ul[0], ul[1], disp))
        want = d["R1"] @ X                              # XYZ is in the RECTIFIED left frame
        err = np.linalg.norm(got - want)
        print(f"  true {X} -> got {np.round(got, 1)} (expected {np.round(want, 1)})  err {err:.1f} mm")
        # With the TRUE parameters the error is 0.0000 mm (checked separately), so what remains is
        # calibration estimation noise (~0.3 px of disparity from 30 views), which grows with Z.
        # A sign/convention bug would give errors of tens of percent, so 2.5% still proves the math.
        assert err < 0.025 * np.linalg.norm(want) + 2.0, "3D error too large"
    print("\nALL CALIBRATION TESTS PASSED")


if __name__ == "__main__":
    main()

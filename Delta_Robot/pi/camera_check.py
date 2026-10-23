"""
camera_check.py -- headless check of the USB stereo camera (works over SSH, no GUI).

    python camera_check.py                 # measures every common mode, prints a table
    python camera_check.py --save 1280x480 # also saves left.png / right.png / pair.png for that mode

The camera sends BOTH eyes in one picture, side by side (e.g. 1280x480 = two 640x480 images). For each mode
this prints the size the camera really delivers, the real frame rate, whether the picture is truly grayscale,
and a rough horizontal shift between the eyes. Pictures are saved to camera_check_out/ so you can copy them to
your PC and look at them:
        scp deltarobot@192.168.137.128:Delta_Robot/pi/camera_check_out/*.png .

IMPORTANT: only one program can use the camera. Stop the vision server first if it is running
(sudo systemctl stop delta-vision, or Ctrl+C).

Trade-off to keep in mind: more pixels = finer depth, but the stereo matcher and YOLO get slower (roughly 4x the work
for 2x the width on a Pi CPU). The table's frame rate is only the CAMERA's; the whole pipeline will be slower.
"""
import argparse
import os
import sys
import time
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pi_camera_server import V4L2Camera, split_eyes   # noqa: E402

# (frame width, frame height, pixel format) -- the modes this camera advertises.
MODES: List[Tuple[int, int, str]] = [
    (640, 240, "MJPG"), (1280, 480, "MJPG"), (1600, 600, "MJPG"),
    (2560, 720, "MJPG"), (2560, 800, "MJPG"), (1280, 480, "YUYV"),
]


def measure(make_camera: Callable[[dict], object], device: str, w: int, h: int, fourcc: str, seconds: float):
    """Open the camera in one mode, read frames for `seconds`, return a dict of facts (or an error string)."""
    cfg = {"device": device, "width": w, "height": h, "fourcc": fourcc, "fps": 0,
           "buffer_size": 1, "auto_exposure": None}
    cam = make_camera(cfg)
    if not cam.open():
        return {"error": "cannot open the camera (is another program using it?)"}
    try:
        for _ in range(5):                      # warm-up: first frames are often dark/odd
            cam.read()
        frames, t0, last = 0, time.time(), None
        while time.time() - t0 < seconds:
            f = cam.read()
            if f is not None:
                last, frames = f, frames + 1
        elapsed = time.time() - t0
        if last is None:
            return {"error": "camera opened but delivered no frames"}
        gray_like = True
        if last.ndim == 3 and last.shape[2] >= 3:
            gray_like = bool(np.abs(last[:, :, 0].astype(int) - last[:, :, 1].astype(int)).max() <= 2
                             and np.abs(last[:, :, 0].astype(int) - last[:, :, 2].astype(int)).max() <= 2)
        left, right = split_eyes(last)
        shift = cv2.phaseCorrelate(left.astype(np.float32), right.astype(np.float32))[0][0]
        return {"w": last.shape[1], "h": last.shape[0], "fps": frames / elapsed, "gray": gray_like,
                "eye": f"{left.shape[1]}x{left.shape[0]}", "bright_l": float(left.mean()),
                "bright_r": float(right.mean()), "shift": float(shift), "frame": last}
    finally:
        cam.close()


def run(argv=None, make_camera: Optional[Callable[[dict], object]] = None) -> int:
    ap = argparse.ArgumentParser(description="Headless stereo camera check")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--seconds", type=float, default=3.0, help="how long to measure each mode")
    ap.add_argument("--save", metavar="WxH", help="save left/right/pair pictures from this mode, e.g. 1280x480")
    ap.add_argument("--fourcc", default="MJPG", help="pixel format for --save (MJPG or YUYV)")
    ap.add_argument("--only", metavar="WxH", help="measure only this mode (with --fourcc)")
    ap.add_argument("--out", default=os.path.join(HERE, "camera_check_out"))
    a = ap.parse_args(argv)
    make_camera = make_camera or (lambda cfg: V4L2Camera(cfg))

    modes = MODES
    if a.only:
        w, h = (int(v) for v in a.only.lower().split("x"))
        modes = [(w, h, a.fourcc.upper())]
    elif a.save:
        w, h = (int(v) for v in a.save.lower().split("x"))
        modes = [(w, h, a.fourcc.upper())]

    print(f"{'mode asked':<16}{'delivered':<12}{'each eye':<10}{'fps':>7}  {'grayscale':<10}{'eye shift(px)':>14}  note")
    saved = None
    for (w, h, fc) in modes:
        r = measure(make_camera, a.device, w, h, fc, a.seconds)
        label = f"{w}x{h} {fc}"
        if "error" in r:
            print(f"{label:<16}{'--':<12}{'--':<10}{'--':>7}  {'--':<10}{'--':>14}  {r['error']}")
            continue
        note = "" if (r["w"], r["h"]) == (w, h) else "SIZE DIFFERS FROM REQUEST"
        print(f"{label:<16}{str(r['w']) + 'x' + str(r['h']):<12}{r['eye']:<10}{r['fps']:>7.1f}  "
              f"{'yes' if r['gray'] else 'NO (colour?)':<10}{r['shift']:>14.1f}  {note}")
        saved = r
    if a.save and saved:
        os.makedirs(a.out, exist_ok=True)
        left, right = split_eyes(saved["frame"])
        cv2.imwrite(os.path.join(a.out, "left.png"), left)
        cv2.imwrite(os.path.join(a.out, "right.png"), right)
        pair = cv2.cvtColor(np.hstack([left, right]), cv2.COLOR_GRAY2BGR)
        for y in range(0, pair.shape[0], 40):         # horizontal lines: matching points should sit on the same row
            cv2.line(pair, (0, y), (pair.shape[1], y), (0, 255, 0), 1)
        cv2.imwrite(os.path.join(a.out, "pair.png"), pair)
        print(f"\nSaved left.png, right.png, pair.png to {a.out}/")
    print("\nReading the table: pick the largest mode whose real fps is comfortably above what the vision pipeline can "
          "process (start with 1280x480). 'eye shift' is a rough left/right offset of the scene, not a calibration.")
    return 0


if __name__ == "__main__":
    sys.exit(run())

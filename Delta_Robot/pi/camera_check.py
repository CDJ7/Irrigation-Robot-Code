"""
camera_check.py -- headless check of the USB stereo camera (works over SSH, no GUI).

    python camera_check.py                      # every mode at the camera's default rate, 30 fps and 15 fps
    python camera_check.py --only 1280x480      # just one frame size
    python camera_check.py --fps 0,30,60        # which frame rates to ask for (0 = leave the camera's default)
    python camera_check.py --save 1280x480 --fps 30   # also save left.png / right.png / pair.png

The camera sends BOTH eyes in one picture, side by side (1280x480 = two 640x480 images). For each combination this
prints the size really delivered, the frame rate the camera agreed to, the REAL measured rate, and how many
"Corrupt JPEG" messages appeared. Those messages (normally a wall of text) mean frames arrive broken -- usually a
USB bandwidth, cable or power problem, or too high a frame rate. They are counted instead of printed.

Pictures go to camera_check_out/; copy them to your PC to look:
        scp deltarobot@192.168.137.128:Delta_Robot/pi/camera_check_out/*.png .

IMPORTANT: only one program can use the camera. Stop the vision server first if it is running.
"""
import argparse
import contextlib
import os
import sys
import tempfile
import time
from typing import Callable, Dict, List, Optional, Tuple

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


@contextlib.contextmanager
def capture_stderr_fd():
    """Temporarily send everything written to file descriptor 2 (including messages from C libraries such as the
    JPEG decoder) into a temp file, so we can count it instead of flooding the screen."""
    sys.stderr.flush()
    saved = os.dup(2)
    tmp = tempfile.TemporaryFile(mode="w+b")
    os.dup2(tmp.fileno(), 2)
    try:
        yield tmp
    finally:
        sys.stderr.flush()
        os.dup2(saved, 2)
        os.close(saved)


def measure(make_camera: Callable[[dict], object], device: str, w: int, h: int, fourcc: str, fps: int,
            seconds: float) -> Dict:
    """Open the camera in one mode, read frames for `seconds`, and return facts (or an error)."""
    cfg = {"device": device, "width": w, "height": h, "fourcc": fourcc, "fps": fps,
           "buffer_size": 1, "auto_exposure": None}
    result: Dict = {}
    with capture_stderr_fd() as err:
        cam = make_camera(cfg)
        if not cam.open():
            result = {"error": "cannot open the camera (is another program using it?)"}
        else:
            try:
                cap = getattr(cam, "cap", None)
                result["agreed_fps"] = float(cap.get(cv2.CAP_PROP_FPS)) if cap is not None else float("nan")
                for _ in range(5):                      # warm-up: first frames are often odd
                    cam.read()
                frames, t0, last = 0, time.time(), None
                while time.time() - t0 < seconds:
                    f = cam.read()
                    if f is not None:
                        last, frames = f, frames + 1
                elapsed = time.time() - t0
                if last is None:
                    result["error"] = "camera opened but delivered no frames"
                else:
                    gray = True
                    if last.ndim == 3 and last.shape[2] >= 3:
                        c0 = last[:, :, 0].astype(int)
                        gray = bool(np.abs(c0 - last[:, :, 1]).max() <= 2 and np.abs(c0 - last[:, :, 2]).max() <= 2)
                    left, _ = split_eyes(last)
                    result.update({"w": last.shape[1], "h": last.shape[0], "fps": frames / elapsed,
                                   "gray": gray, "eye": f"{left.shape[1]}x{left.shape[0]}", "frame": last})
            finally:
                cam.close()
        err.flush()
        err.seek(0)
        text = err.read().decode("utf-8", "replace")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    result["corrupt"] = sum("Corrupt JPEG" in ln for ln in lines)
    result["other_msgs"] = sorted({ln.strip() for ln in lines if "Corrupt JPEG" not in ln})[:3]
    return result


def run(argv=None, make_camera: Optional[Callable[[dict], object]] = None) -> int:
    ap = argparse.ArgumentParser(description="Headless stereo camera check")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--seconds", type=float, default=3.0, help="how long to measure each combination")
    ap.add_argument("--fps", default="0,30,15", help="comma list of frame rates to ask for (0 = camera default)")
    ap.add_argument("--only", metavar="WxH", help="test only this frame size (uses --fourcc)")
    ap.add_argument("--fourcc", default="MJPG", help="pixel format for --only / --save (MJPG or YUYV)")
    ap.add_argument("--save", metavar="WxH", help="save left/right/pair pictures from this size (first --fps value)")
    ap.add_argument("--out", default=os.path.join(HERE, "camera_check_out"))
    a = ap.parse_args(argv)
    make_camera = make_camera or (lambda cfg: V4L2Camera(cfg))
    fps_list = [int(v) for v in a.fps.split(",")]

    modes = MODES
    pick = a.only or a.save
    if pick:
        w, h = (int(v) for v in pick.lower().split("x"))
        modes = [(w, h, a.fourcc.upper())]
    if a.save:
        fps_list = fps_list[:1]

    head = f"{'mode':<14}{'asked fps':>9}{'agreed':>8}{'real fps':>9}{'delivered':>11}{'each eye':>10}{'corrupt':>8}  {'gray':<5} note"
    print(head)
    print("-" * len(head))
    saved = None
    for (w, h, fc) in modes:
        for fps in fps_list:
            r = measure(make_camera, a.device, w, h, fc, fps, a.seconds)
            label = f"{w}x{h} {fc}"
            asked = "default" if fps == 0 else str(fps)
            if "error" in r:
                print(f"{label:<14}{asked:>9}{'--':>8}{'--':>9}{'--':>11}{'--':>10}{r['corrupt']:>8}  {'--':<5} {r['error']}")
                continue
            notes = []
            if (r["w"], r["h"]) != (w, h):
                notes.append("SIZE DIFFERS FROM REQUEST")
            if r["corrupt"]:
                notes.append("BROKEN FRAMES")
            agreed = "n/a" if np.isnan(r["agreed_fps"]) else f"{r['agreed_fps']:.0f}"
            print(f"{label:<14}{asked:>9}{agreed:>8}{r['fps']:>9.1f}{str(r['w']) + 'x' + str(r['h']):>11}"
                  f"{r['eye']:>10}{r['corrupt']:>8}  {'yes' if r['gray'] else 'NO':<5} {' '.join(notes)}")
            for m in r["other_msgs"]:
                print(f"      message: {m}")
            saved = r
    if a.save and saved and "frame" in saved:
        os.makedirs(a.out, exist_ok=True)
        left, right = split_eyes(saved["frame"])
        cv2.imwrite(os.path.join(a.out, "left.png"), left)
        cv2.imwrite(os.path.join(a.out, "right.png"), right)
        pair = cv2.cvtColor(np.hstack([left, right]), cv2.COLOR_GRAY2BGR)
        for y in range(0, pair.shape[0], 40):         # matching points should sit on the same row
            cv2.line(pair, (0, y), (pair.shape[1], y), (0, 255, 0), 1)
        cv2.imwrite(os.path.join(a.out, "pair.png"), pair)
        print(f"\nSaved left.png, right.png, pair.png to {a.out}/")
    print("\nHow to read it: 'corrupt' should be 0. Pick the largest size whose real fps is comfortably above what the "
          "vision pipeline can process (10-15 is plenty to start) with zero broken frames. 'agreed' is the rate the camera "
          "accepted; 'real' is what was actually delivered.")
    return 0


if __name__ == "__main__":
    sys.exit(run())

"""
calib_capture.py -- LIVE stereo-calibration capture, used by `python pi_camera_server.py --calibrate-capture`.

While you move the checkerboard in front of the camera, every frame is checked here: is the whole board visible in BOTH
eyes, is it holding still, has it moved enough since the last saved pair? If yes, the pair is saved automatically. The
pictures streamed to the GUI / browser show the detected corners and a short instruction, so you can watch your own
progress on the PC instead of working blind over SSH.

Saved files are the same ones calibrate_stereo.py solves from: pair_000_L.png / pair_000_R.png (raw grayscale eyes).
"""
import glob
import os
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from stereo_common import find_corners, pair_is_consistent

REGION_NAMES = [["top-left", "top-centre", "top-right"],
                ["middle-left", "middle-centre", "middle-right"],
                ["bottom-left", "bottom-centre", "bottom-right"]]


class CalibrationCapturer:
    def __init__(self, out_dir: str, pattern: Tuple[int, int] = (9, 6), target: int = 30, interval: float = 1.5,
                 min_move_px: float = 12.0, stable_px: float = 1.5):
        self.out_dir, self.pattern, self.target = out_dir, pattern, target
        self.interval, self.min_move, self.stable_px = interval, min_move_px, stable_px
        os.makedirs(out_dir, exist_ok=True)
        self.saved = len(glob.glob(os.path.join(out_dir, "pair_*_L.png")))     # continue after earlier runs
        self.start_saved = self.saved
        self._prev: Optional[np.ndarray] = None          # corners of the previous frame (stability check)
        self._last_saved: Optional[np.ndarray] = None    # corners of the last saved pair (movement check)
        self._last_save_t = 0.0
        self.coverage = np.zeros((3, 3), int)
        self.widths: List[float] = []

    # ------------------------------------------------------------------ helpers
    def _hint(self, found_l: bool, found_r: bool, stable: bool, moved: bool) -> str:
        if self.saved >= self.target:
            return "DONE - enough pairs. Press Ctrl+C on the Pi, then run: python calibrate_stereo.py solve --square-mm <size>"
        if not (found_l and found_r):
            side = "left" if not found_l and found_r else "right" if found_l and not found_r else "either"
            return f"Board not found in the {side} eye: show the WHOLE board to both eyes, flat, evenly lit, no glare"
        if not stable:
            return "Hold the board still..."
        if not moved:
            return "Move or tilt the board to a new position"
        return "Saving..."

    def _coverage_hint(self) -> str:
        empty = [REGION_NAMES[r][c] for r in range(3) for c in range(3) if self.coverage[r, c] == 0]
        parts = []
        if empty and self.saved >= 3:
            parts.append("still needed: " + ", ".join(empty[:3]))
        if len(self.widths) >= 8 and max(self.widths) / max(min(self.widths), 1.0) < 1.35:
            parts.append("vary the distance (some near, some far)")
        return "; ".join(parts)

    def _save(self, left: np.ndarray, right: np.ndarray, cl: np.ndarray, now: float, shape) -> None:
        cv2.imwrite(os.path.join(self.out_dir, f"pair_{self.saved:03d}_L.png"), left)
        cv2.imwrite(os.path.join(self.out_dir, f"pair_{self.saved:03d}_R.png"), right)
        self.saved += 1
        self._last_saved, self._last_save_t = cl.copy(), now
        h, w = shape
        cx, cy = float(cl[:, 0, 0].mean()), float(cl[:, 0, 1].mean())
        self.coverage[min(2, int(cy / h * 3)), min(2, int(cx / w * 3))] += 1
        self.widths.append(float(np.ptp(cl[:, 0, 0])))

    # -------------------------------------------------------------------- main
    def process(self, left: np.ndarray, right: np.ndarray, now: Optional[float] = None
                ) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """Look at one stereo frame. Returns (left picture, right picture, info) -- pictures are BGR with corners drawn."""
        now = time.time() if now is None else now
        cl = find_corners(left, self.pattern, fast=True)
        cr = find_corners(right, self.pattern, fast=True) if cl is not None else None
        found_l, found_r = cl is not None, cr is not None
        both = found_l and found_r and pair_is_consistent(cl, cr)

        stable = moved = False
        if both:
            if self._prev is not None and self._prev.shape == cl.shape:
                stable = float(np.mean(np.linalg.norm(cl - self._prev, axis=2))) < self.stable_px
            self._prev = cl.copy()
            moved = self._last_saved is None or float(np.mean(np.linalg.norm(cl - self._last_saved, axis=2))) > self.min_move
            if (stable and moved and now - self._last_save_t >= self.interval and self.saved < self.target):
                self._save(left, right, cl, now, left.shape[:2])
        else:
            self._prev = None

        vis_l, vis_r = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR), cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
        if found_l:
            cv2.drawChessboardCorners(vis_l, self.pattern, cl, True)
        if found_r:
            cv2.drawChessboardCorners(vis_r, self.pattern, cr, True)
        colour = (0, 200, 0) if both else (0, 140, 255) if (found_l or found_r) else (0, 0, 255)
        hint = self._hint(found_l, found_r, stable, moved)
        for img in (vis_l, vis_r):
            cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1), colour, 3)
        cv2.putText(vis_l, f"saved {self.saved}/{self.target}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
        info = {"saved": self.saved, "target": self.target, "new_this_run": self.saved - self.start_saved,
                "found_left": found_l, "found_right": found_r, "stable": stable,
                "hint": hint, "coverage_hint": self._coverage_hint(), "done": self.saved >= self.target}
        return vis_l, vis_r, info

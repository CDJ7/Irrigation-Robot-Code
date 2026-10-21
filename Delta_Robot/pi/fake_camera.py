"""
fake_camera.py -- a synthetic stereo camera + detector for TESTING without hardware.

The scene is a textured background plus one rectangular "object" that sits closer to the
camera (bigger disparity). The right eye is built by shifting the left eye by exactly
those disparities, so the true depth is known and tests can check the pipeline's answer:

    Z = focal_px * baseline_mm / disparity     (uncalibrated nominal numbers in config.yaml)
"""

import numpy as np


class FakeStereoCamera:
    def __init__(self, width=640, height=240, bg_disp=8, fg_disp=40, fg_box=(120, 80, 200, 160),
                 seed=1):
        self.w, self.h = width, height
        self.eye_w = width // 2
        self.bg_disp, self.fg_disp, self.fg_box = bg_disp, fg_disp, fg_box
        rng = np.random.default_rng(seed)
        # Random texture, blurred a little so block matching has something to lock onto.
        import cv2
        base = rng.integers(0, 256, (self.h, self.eye_w + 100), dtype=np.uint8)
        self.tex = cv2.GaussianBlur(base, (3, 3), 0)
        self.rng = rng
        self.opened = False

    def open(self) -> bool:
        self.opened = True
        return True

    def close(self) -> None:
        self.opened = False

    def read(self):
        w = self.eye_w
        left = self.tex[:, 50:50 + w].copy()
        right = np.zeros_like(left)
        # Background: right(x - d) = left(x)
        d = self.bg_disp
        right[:, 0:w - d] = left[:, d:w] if d else left
        right[:, w - d:] = self.tex[:, 50 + w - d - d:50 + w - d] if d else 0
        # Foreground object: same texture but shifted by a bigger disparity.
        x1, y1, x2, y2 = self.fg_box
        d = self.fg_disp
        right[y1:y2, x1 - d:x2 - d] = left[y1:y2, x1:x2]
        # A little sensor noise so every frame is not identical.
        noise = self.rng.integers(-2, 3, left.shape)
        left = np.clip(left.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        frame = np.hstack([left, right])
        return np.dstack([frame, frame, frame])    # camera gives 3 identical channels


class FakeDetector:
    """Pretends to be a YOLO model: reports the fake object as a strawberry AND a 'person'
    (to prove the allowed-class flag works), and one box over a textureless hole."""
    names = {0: "strawberry", 1: "person"}

    def __init__(self, fg_box=(120, 80, 200, 160)):
        self.fg_box = fg_box

    def detect(self, left_gray):
        x1, y1, x2, y2 = self.fg_box
        return [
            {"class_id": 0, "class": "strawberry", "confidence": 0.9, "bbox": (x1, y1, x2, y2)},
            {"class_id": 1, "class": "person", "confidence": 0.6, "bbox": (x1, y1, x2, y2)},
            # Box in the corner where the right eye has no data -> depth should be null
            {"class_id": 0, "class": "strawberry", "confidence": 0.5, "bbox": (2, 2, 14, 14)},
        ]

"""Tests camera_check.py with a FAKE camera (no hardware). Run: python tests/test_camera_check_stub.py"""
import contextlib, io, os, sys, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import camera_check as cc   # noqa: E402


class FakeCam:
    def __init__(self, cfg): self.cfg = cfg; self.rng = np.random.default_rng(0)
    def open(self): return self.cfg["width"] != 2560            # pretend the biggest mode is refused
    def close(self): pass
    def read(self):
        w, h = self.cfg["width"], self.cfg["height"]
        if (w, h) == (1600, 600): w, h = 1280, 480             # pretend this mode silently gives another size
        eye = self.rng.integers(0, 255, (h, w // 2), np.uint8)
        both = np.hstack([eye, np.roll(eye, -5, axis=1)])
        return np.dstack([both] * 3)


out = tempfile.mkdtemp()
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    rc = cc.run(["--seconds", "0.2", "--out", out], make_camera=FakeCam)
text = buf.getvalue(); print(text)
assert rc == 0
assert "1280x480 MJPG" in text and "640x480" in text and "yes" in text
assert "cannot open the camera" in text                       # 2560 modes reported, not crashed
assert "SIZE DIFFERS FROM REQUEST" in text
with contextlib.redirect_stdout(io.StringIO()):
    cc.run(["--save", "1280x480", "--seconds", "0.2", "--out", out], make_camera=FakeCam)
for f in ("left.png", "right.png", "pair.png"):
    assert os.path.exists(os.path.join(out, f)), f
print("ALL TESTS PASSED")

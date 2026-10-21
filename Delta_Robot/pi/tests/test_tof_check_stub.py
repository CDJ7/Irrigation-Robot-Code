"""
Tests tof_check.py against a STAND-IN for the Arducam SDK (no camera needed). This checks OUR logic
(stats, masking, flat-wall report, saved files, error hints) -- NOT the real camera.
Run:  python tests/test_tof_check_stub.py
"""
import io, os, sys, tempfile, types, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tof"))
import tof_check  # noqa: E402


class Code:
    def __init__(self, name): self.name = name
    def __eq__(self, o): return (o == 0) if self.name == "ArducamSuccess" else False
    __hash__ = None


def make_stub(open_result="ArducamSuccess", wall_mm=600.0):
    ac = types.SimpleNamespace()
    ac.__version__ = "stub"
    ac.Connection = types.SimpleNamespace(CSI="CSI", USB="USB")
    ac.FrameType = types.SimpleNamespace(DEPTH="DEPTH")
    ac.Control = types.SimpleNamespace(RANGE="RANGE", HFLIP="HFLIP", VFLIP="VFLIP", INTRINSIC_FX="FX",
                                       INTRINSIC_FY="FY", INTRINSIC_CX="CX", INTRINSIC_CY="CY")

    class DepthData:
        def __init__(self):
            rng = np.random.default_rng(0)
            self.depth_data = (wall_mm + rng.normal(0, 3, (180, 240))).astype(np.float32)
            self.confidence_data = np.full((180, 240), 200, np.float32)
            self.confidence_data[:, :20] = 5          # unreliable left border
            self.depth_data[:, :20] = 0
            self.amplitude_data = rng.uniform(0, 1000, (180, 240)).astype(np.float32)
    ac.DepthData = DepthData

    class Cam:
        controls = {"RANGE": 4000, "FX": 21000, "FY": 21000, "CX": 12000, "CY": 9000, "HFLIP": 0, "VFLIP": 0}
        def open(self, conn, idx=0): return Code(open_result)
        def start(self, t): return Code("ArducamSuccess")
        def stop(self): return Code("ArducamSuccess")
        def close(self): return Code("ArducamSuccess")
        def setControl(self, c, v): self.controls[c] = v; return Code("ArducamSuccess")
        def getControl(self, c): return self.controls[c]
        def getCameraInfo(self): return types.SimpleNamespace(device_type=types.SimpleNamespace(name="HQVGA"), width=240, height=180)
        def requestFrame(self, t): return DepthData()
        def releaseFrame(self, f): return Code("ArducamSuccess")
    ac.ArducamCamera = Cam
    return ac


def run(argv, ac):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = tof_check.main(argv, ac=ac)
    return rc, buf.getvalue()


out = tempfile.mkdtemp()
rc, text = run(["--connection", "csi", "--frames", "5", "--out", out], make_stub())
print(text)
assert rc == 0
assert "CENTER PIXEL distance: 6" in text, "center should read ~600 mm"
assert "INTRINSIC_FX (/100)  : 210.0" in text
for f in ("depth_color.png", "confidence.png", "amplitude.png", "depth_mm.npy"):
    assert os.path.exists(os.path.join(out, f)), f
d = np.load(os.path.join(out, "depth_mm.npy")); assert d.shape == (180, 240)
assert "Valid pixels" in text and "Flat-wall test" in text

rc, text = run(["--connection", "usb", "--out", out], make_stub(open_result="ArducamNoPermission"))
assert rc == 1 and "Permission denied" in text and "Nothing was changed" in text
rc, text = run(["--connection", "auto", "--out", out], make_stub(open_result="ArducamSystemError"))
assert rc == 1 and "boot overlay" in text and "open(USB) failed" in text
print("ALL TESTS PASSED")

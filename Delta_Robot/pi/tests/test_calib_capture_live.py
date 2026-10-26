"""
END-TO-END test of the live calibration capture (no hardware): synthetic checkerboard frames -> server in
--calibrate-capture mode -> pairs saved automatically -> real solver -> recovered baseline vs the known truth.
Run:  python tests/test_calib_capture_live.py
"""
import json
import os
import socket
import struct
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, HERE)
import calibrate_stereo as cs                         # noqa: E402
import pi_camera_server as srv                        # noqa: E402
from calib_capture import CalibrationCapturer         # noqa: E402
import test_calibration_synthetic as syn              # noqa: E402  (renders boards seen by two cameras with known geometry)
import argparse


class HoldingCamera:
    """Shows each synthetic board pose for `hold` frames (a person holding the board still), then the next pose."""
    def __init__(self, frames, hold=5):
        self.frames, self.hold, self.i, self.n = frames, hold, 0, 0
    def open(self): return True
    def close(self): pass
    def read(self):
        f = self.frames[(self.n // self.hold) % len(self.frames)]
        self.n += 1
        return f


def recv_exact(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c: raise ConnectionError("closed")
        buf += c
    return buf


def read_group(s):
    j = None; got = ""
    while len(got) < 3 or j is None or "V" not in got or "D" not in got:
        h = recv_exact(s, 5); tag = chr(h[0]); payload = recv_exact(s, struct.unpack(">I", h[1:])[0]); got += tag
        if tag == "J": j = json.loads(payload)
    return j


def main():
    rng = np.random.default_rng(7)
    tmp = tempfile.mkdtemp()
    out_dir = os.path.join(tmp, "images")
    frames = syn.make_frames(24, rng)
    cfg = srv.load_config(os.path.join(PI, "config.yaml"))
    cfg["network"].update(tcp_port=0, http_port=None, target_fps=40, send_timeout_s=2.0)
    cfg["dataset"]["enabled"] = False
    server = srv.PiCameraServer(cfg, HoldingCamera(frames), None, cfg["dataset"])
    server.pipeline.calib = CalibrationCapturer(out_dir, (9, 6), target=20, interval=0.0, min_move_px=3.0)
    server.start()
    try:
        s = socket.create_connection(("127.0.0.1", server.tcp_port), timeout=5)
        saw = []
        t_end = time.time() + 40
        while time.time() < t_end:
            j = read_group(s)
            saw.append(j)
            if j["calib"]["done"]:
                break
        last = saw[-1]
        assert last["mode"] == "calibration_capture" and last["detections"] == [] and last["calibrated"] is False
        assert last["calib"]["saved"] >= 20, last["calib"]
        hints = {x["calib"]["hint"] for x in saw}
        print("hints shown to the user:", sorted(h[:50] for h in hints))
        assert any("Hold" in h or "Move" in h or "Saving" in h for h in hints)
        assert last["calib"]["hint"].startswith("DONE")
        s.close()
    finally:
        server.stop()
    n = len([f for f in os.listdir(out_dir) if f.endswith("_L.png")])
    print(f"pairs on disk: {n}")
    assert n >= 20

    # now the real solver on what the live capture saved
    ns = argparse.Namespace(board="9x6", square_mm=syn.SQ, dir=out_dir, config=os.path.join(PI, "config.yaml"),
                            out=os.path.join(tmp, "calibration", "stereo_calib.npz"), alpha=0.0)
    assert cs.solve(ns) == 0
    d = np.load(ns.out)
    baseline = float(-d["P2"][0, 3] / d["P2"][0, 0])
    print(f"baseline recovered from LIVE-captured pairs: {baseline:.2f} mm (truth 62.0)")
    assert abs(baseline - 62.0) < 0.8
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

"""
End-to-end test of pi_camera_server.py with the FAKE camera (no hardware needed).
Run:  python tests/test_server_fake.py

Checks: J/V/D protocol and ordering, sequence numbers, true depth recovered from the
synthetic stereo pair, null depth for boxes over holes, allowed-class flag, several
clients at once (2 TCP + 1 browser), a stalled client not hurting the others,
and dataset capture.
"""
import json
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import urllib.request

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import pi_camera_server as srv                      # noqa: E402
from fake_camera import FakeStereoCamera, FakeDetector   # noqa: E402


def make_cfg(tmp):
    cfg = srv.load_config(os.path.join(os.path.dirname(HERE), "config.yaml"))
    cfg["network"].update(tcp_port=0, http_port=0, target_fps=30, send_timeout_s=1.0)
    cfg["stereo"].update(calibration_file="/nonexistent", nominal_focal_px=280.0,
                         nominal_baseline_mm=60.0, num_disparities=64)
    cfg["dataset"].update(enabled=True, dir=tmp, interval_s=0.2)
    return cfg


def recv_exact(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            raise ConnectionError("closed")
        buf += c
    return buf


def read_msg(s):
    h = recv_exact(s, 5)
    return chr(h[0]), recv_exact(s, struct.unpack(">I", h[1:])[0])


def read_frame_group(s):
    """Reads messages until one J,V,D group is complete. Returns (json, vimg, dimg, order)."""
    j = v = d = None
    order = ""
    while not (j and v is not None and d is not None):
        tag, payload = read_msg(s)
        order += tag
        if tag == "J":
            j = json.loads(payload.decode("utf-8"))
        elif tag == "V":
            v = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
        elif tag == "D":
            d = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    return j, v, d, order


def main():
    tmp = tempfile.mkdtemp()
    cfg = make_cfg(tmp)
    cam = FakeStereoCamera()
    server = srv.PiCameraServer(cfg, cam, FakeDetector(), cfg["dataset"])
    server.start()
    try:
        # ---- 1. protocol + depth accuracy -------------------------------------
        c1 = socket.create_connection(("127.0.0.1", server.tcp_port), timeout=5)
        j, v, d, order = read_frame_group(c1)
        assert order == "JVD", f"order must be J,V,D, got {order}"
        assert (j["width"], j["height"]) == (320, 240) and v.shape[:2] == (240, 320)
        assert d.shape == (240, 320, 3)
        assert j["calibrated"] is False and j["units"] == "mm"
        assert j["model_has_allowed_class"] is True   # fake model knows 'strawberry'
        print("protocol OK, first J:", {k: j[k] for k in ("seq", "calibrated", "server_fps")})

        dets = j["detections"]
        straw = [x for x in dets if x["class"] == "strawberry" and x["bbox"][0] > 100][0]
        person = [x for x in dets if x["class"] == "person"][0]
        hole = [x for x in dets if x["bbox"][0] < 5][0]
        expected_z = 280.0 * 60.0 / 40.0      # f*B/d = 420 mm
        x, y, z = straw["xyz_mm"]
        print(f"strawberry xyz_mm={straw['xyz_mm']} disparity={straw['disparity']} "
              f"(expected Z={expected_z:.0f})")
        assert abs(z - expected_z) < 0.05 * expected_z, "depth off by more than 5%"
        # u=160,v=120 is the image center -> X and Y should be ~0 with the fake principal point
        assert abs(x) < 15 and abs(y) < 15, (x, y)
        assert straw["allowed"] is True and person["allowed"] is False
        assert hole["xyz_mm"] is None, "box over a hole must give null depth"
        print("depth accuracy, allowed flag, null-on-hole OK")
        assert "center_probe" in j and j["center_probe"]["xyz_mm"] is not None
        print("center_probe:", j["center_probe"])

        # ---- 2. several clients at once; seq strictly increasing, J matches V --
        c2 = socket.create_connection(("127.0.0.1", server.tcp_port), timeout=5)
        url = f"http://127.0.0.1:{server.http_port}"
        snap = urllib.request.urlopen(url + "/snapshot.jpg", timeout=5).read()
        img = cv2.imdecode(np.frombuffer(snap, np.uint8), cv2.IMREAD_COLOR)
        assert img.shape[:2] == (240, 640), img.shape       # annotated left | depth
        status = json.loads(urllib.request.urlopen(url + "/status", timeout=5).read())
        print("browser snapshot OK; /status:", status)

        seqs1, seqs2 = [], []
        t0 = time.time()
        for _ in range(30):
            j1, *_ = read_frame_group(c1)
            seqs1.append(j1["seq"])
            j2, *_ = read_frame_group(c2)
            seqs2.append(j2["seq"])
        dt = time.time() - t0
        assert seqs1 == sorted(set(seqs1)) and seqs2 == sorted(set(seqs2)), "seq must increase"
        print(f"2 TCP clients received 30 frame groups each in {dt:.1f}s "
              f"(client1 saw seq {seqs1[0]}..{seqs1[-1]})")

        # ---- 3. a stalled client must not freeze the others --------------------
        stalled = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        stalled.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)   # must be set BEFORE connect
        stalled.connect(("127.0.0.1", server.tcp_port))                  # ...and we never read from it
        t0 = time.time()
        n0 = read_frame_group(c1)[0]["seq"]
        for _ in range(20):
            last = read_frame_group(c1)[0]["seq"]
        print(f"with a stalled client attached: 20 more groups in {time.time()-t0:.1f}s")
        assert last > n0
        # the stalled client should get dropped after send_timeout_s (1 s in this test)
        t_end = time.time() + 4.0
        while time.time() < t_end:            # keep the two healthy clients reading
            read_frame_group(c1)
            read_frame_group(c2)
        assert server.hub.tcp_clients <= 2, f"stalled client still attached: {server.hub.tcp_clients}"
        print("stalled client dropped OK; tcp_clients =", server.hub.tcp_clients)

        # ---- 4. disconnect handling -------------------------------------------
        c2.close()
        read_frame_group(c1)
        time.sleep(0.5)
        read_frame_group(c1)
        assert server.hub.tcp_clients == 1, server.hub.tcp_clients
        print("client disconnect handled OK")

        # ---- 5. dataset capture -------------------------------------------------
        time.sleep(1.0)
        files = [f for f in os.listdir(tmp) if f.endswith(".png")]
        assert len(files) >= 3, files
        im = cv2.imread(os.path.join(tmp, files[0]), cv2.IMREAD_UNCHANGED)
        assert im.ndim == 2 and im.shape == (240, 320), "dataset images must be 320x240 grayscale"
        print(f"dataset capture OK: {len(files)} grayscale PNGs")

        # ---- 6. FPS + idle behaviour --------------------------------------------
        c1.close()
        time.sleep(1.5)
        assert server.hub.consumers() == 0
        print("Final server fps (fake camera, this VM, NOT a Pi number):",
              round(server.pipeline.fps, 1))
        print("\nALL TESTS PASSED")
    finally:
        server.stop()


if __name__ == "__main__":
    main()

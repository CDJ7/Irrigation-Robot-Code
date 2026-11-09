"""
Tests the PROBE request ("measure the 3D position at this pixel") end to end: real server (fake camera) + the real camera client.
Run:  python gui/tests/test_camera_probe.py       (needs numpy, opencv, PyYAML)
"""
import os
import queue
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "gui")); sys.path.insert(0, os.path.join(ROOT, "pi"))
import pi_camera_server as srv                       # noqa: E402
from fake_camera import FakeDetector, FakeStereoCamera   # noqa: E402
from camera_client import CameraClient               # noqa: E402


def make_server(port=0):
    cfg = srv.load_config(os.path.join(ROOT, "pi", "config.yaml"))
    cfg["network"].update(tcp_port=port, http_port=None, target_fps=30, send_timeout_s=1.0)
    cfg["stereo"].update(calibration_file="/nonexistent", nominal_focal_px=280.0, nominal_baseline_mm=60.0, num_disparities=64, min_disparity=0)
    cfg["depth"].update(min_z_mm=100.0, max_z_mm=3000.0)
    cfg["dataset"]["enabled"] = False
    s = srv.PiCameraServer(cfg, FakeStereoCamera(), FakeDetector(), cfg["dataset"]); s.start()
    return s


def latest_det(q, seconds=1.0, want=None):
    end, last = time.time() + seconds, None
    while time.time() < end:
        try:
            item = q.get(timeout=0.1)
        except queue.Empty:
            continue
        if item[0] == "DET":
            last = item[1]
            if want is None or want(last):
                return last
    return last


def wait(pred, t=5.0):
    end = time.time() + t
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def main():
    server = make_server(); port = server.tcp_port
    q = queue.Queue(maxsize=50); c = CameraClient(q, recv_timeout=2.0)
    c.start("127.0.0.1", port); assert wait(c.is_connected)
    det = latest_det(q); assert det and "probe" not in det, "no probe key unless requested"

    c.send_probe(160, 120)                                     # inside the foreground object (disparity 40 -> Z = 280*60/40)
    det = latest_det(q, 3, lambda d: "probe" in d)
    p = det["probe"]; assert (p["u"], p["v"]) == (160, 120), p
    assert p["xyz_mm"] is not None and abs(p["xyz_mm"][2] - 420.0) < 8, p
    print("1. probe inside the object: Z =", p["xyz_mm"][2], "mm (expected 420) OK")

    c.send_probe(250, 200)                                     # background (disparity 8 -> Z = 2100)
    det = latest_det(q, 3, lambda d: d.get("probe", {}).get("u") == 250)
    p = det["probe"]; assert p["xyz_mm"] is not None and abs(p["xyz_mm"][2] - 2100.0) < 150, p
    print("2. probe on the background: Z =", p["xyz_mm"][2], "mm (expected ~2100) OK")

    for u, v in ((40, 200), (2, 2)):                           # the left-edge band (as wide as the search range) has no stereo depth
        c.send_probe(u, v)
        det = latest_det(q, 3, lambda d, u=u: d.get("probe", {}).get("u") == u)
        assert det["probe"]["xyz_mm"] is None and det["probe"]["valid_fraction"] < 0.5, det["probe"]
    print("3. probe where there is no depth (left-edge band): xyz_mm null, no crash OK")

    c.clear_probe()
    det = latest_det(q, 3, lambda d: "probe" not in d); assert "probe" not in det
    print("4. 'PROBE off' removes the key OK")

    # garbage is ignored, the stream keeps going
    c._send_line("PROBE x y"); c._send_line("HELLO"); c._send_line("PROBE 99999 5"); c._send_line("A" * 2000)
    det = latest_det(q, 2); assert det and "probe" not in det and c.is_connected()
    print("5. malformed / oversized requests are ignored OK")

    # reconnect: the stored probe is re-sent automatically
    c.send_probe(160, 120)
    latest_det(q, 3, lambda d: "probe" in d)
    server.stop()
    assert wait(lambda: not c.is_connected(), 6)
    server2 = make_server(port)
    assert wait(c.is_connected, 15)
    det = latest_det(q, 4, lambda d: "probe" in d); assert det and "probe" in det, "probe must survive a reconnect"
    print("6. probe re-sent after the server came back OK")

    # a closing client is noticed straight away (the reader thread), and the probe is forgotten with the last client
    c.stop()
    assert wait(lambda: server2.hub.tcp_clients == 0, 3), server2.hub.tcp_clients
    assert server2.hub.probe is None
    print("7. client close detected at once; probe cleared with the last client OK")
    server2.stop()
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

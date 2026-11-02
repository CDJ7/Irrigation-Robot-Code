"""
Tests the NEW camera_client.py against the real pi_camera_server.py running with the FAKE camera (no hardware).
Run:  python gui/tests/test_camera_client.py     (needs numpy, opencv-python, PyYAML)

Checks: connect + status, V/D pictures matched to their J (same seq), honest is_connected() after the server dies,
automatic reconnect when it comes back, bounded queue, unreachable host handling, prompt stop().
"""
import os
import queue
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))                 # .../Delta_Robot
sys.path.insert(0, os.path.join(ROOT, "gui"))
sys.path.insert(0, os.path.join(ROOT, "pi"))
import pi_camera_server as srv                                # noqa: E402
from fake_camera import FakeStereoCamera, FakeDetector        # noqa: E402
from camera_client import CameraClient                        # noqa: E402


def make_server(port=0):
    cfg = srv.load_config(os.path.join(ROOT, "pi", "config.yaml"))
    cfg["network"].update(tcp_port=port, http_port=None, target_fps=30, send_timeout_s=1.0)
    cfg["stereo"].update(calibration_file="/nonexistent", nominal_focal_px=280.0, nominal_baseline_mm=60.0,
                         num_disparities=64)
    cfg["dataset"]["enabled"] = False
    s = srv.PiCameraServer(cfg, FakeStereoCamera(), FakeDetector(), cfg["dataset"])
    s.start()
    return s


def drain(q, seconds):
    items, end = [], time.time() + seconds
    while time.time() < end:
        try:
            items.append(q.get(timeout=0.1))
        except queue.Empty:
            pass
    return items


def wait_for(cond, timeout, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def main():
    server = make_server()
    port = server.tcp_port
    q = queue.Queue(maxsize=50)
    client = CameraClient(q, recv_timeout=2.0)

    # 1. connect, status, pictures matched to detections --------------------------------------------------
    client.start("127.0.0.1", port)
    assert wait_for(client.is_connected, 5), f"never connected: {client.status}"
    items = drain(q, 1.5)
    kinds = {i[0] for i in items}
    assert {"STATUS", "DET", "V", "D"} <= kinds, kinds
    assert any(i[0] == "STATUS" and i[1]["state"] == "connected" for i in items)
    det_seqs = {i[1]["seq"] for i in items if i[0] == "DET"}
    v = [i for i in items if i[0] == "V"]
    assert v and all(x[2] in det_seqs or x[2] is not None for x in v)
    first_v = v[-1]
    assert first_v[1].shape == (240, 320, 3) and first_v[1].dtype.name == "uint8"
    matches = [i for i in items if i[0] == "V" and i[2] in det_seqs]
    assert matches, "V pictures must carry the seq of their J"
    det = [i[1] for i in items if i[0] == "DET"][-1]
    assert det["calibrated"] is False and len(det["detections"]) == 3
    print("1. connect + J/V/D matching OK:", client.status)

    # 2. server dies -> client must NOT claim to be connected --------------------------------------------
    server.stop()
    assert wait_for(lambda: not client.is_connected(), 6), "still 'connected' after the server died"
    st = client.status
    assert st["state"] in ("disconnected", "connecting"), st
    print("2. honest status after drop OK:", st)

    # 3. server returns -> automatic reconnect ------------------------------------------------------------
    server2 = make_server(port)
    assert wait_for(client.is_connected, 15), f"did not reconnect: {client.status}"
    assert any(i[0] == "V" for i in drain(q, 1.0))
    print("3. automatic reconnect OK")

    # 4. bounded queue: slow consumer must not grow memory ---------------------------------------------------
    small = queue.Queue(maxsize=5)
    c2 = CameraClient(small, recv_timeout=2.0)
    c2.start("127.0.0.1", port)
    time.sleep(2.0)                                    # never read -> would be hundreds of items if unbounded
    assert small.qsize() <= 5, small.qsize()
    newest = []
    while not small.empty():
        newest.append(small.get_nowait())
    assert any(i[0] in ("V", "D", "DET") for i in newest), "queue should hold the NEWEST data"
    t0 = time.time()
    c2.stop()
    assert time.time() - t0 < 2.5, "stop() too slow"
    assert not c2.is_connected()
    print("4. bounded queue + fast stop OK")

    # 5. unreachable host: no crash, no hang, honest status ---------------------------------------------------
    q3 = queue.Queue(maxsize=10)
    c3 = CameraClient(q3, connect_timeout=1.0)
    c3.start("127.0.0.1", 9)                            # nothing listens on port 9
    assert wait_for(lambda: c3.status["state"] == "disconnected", 5), c3.status
    assert not c3.is_connected() and "cannot connect" in c3.status["message"]
    t0 = time.time(); c3.stop(); assert time.time() - t0 < 2.5
    print("5. unreachable host handled:", c3.status)

    client.stop()
    server2.stop()
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

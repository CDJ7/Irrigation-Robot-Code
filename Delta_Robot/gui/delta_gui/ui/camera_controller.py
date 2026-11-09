"""
camera_controller.py -- everything about the camera stream, kept OFF the GUI thread.

  * CameraClient (a background thread) connects, reconnects and decodes the pictures
  * a QThread (_Pump) moves what it produces to Qt signals, so the pages just connect to signals
  * "simulated camera" mode starts the real Pi server in this process with a fake camera, so the whole path (J/V/D messages,
    reconnect, detections) is exercised without a Pi
"""
import os
import queue
import sys
from typing import Optional

from PyQt6.QtCore import QObject, QThread, pyqtSignal

from camera_client import CameraClient

PI_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "pi"))


class _Pump(QThread):
    status = pyqtSignal(dict)
    detections = pyqtSignal(dict)
    video = pyqtSignal(object, object)      # RGB numpy array, seq
    depth = pyqtSignal(object, object)

    def __init__(self, q: "queue.Queue"):
        super().__init__()
        self.q = q
        self._stop = False

    def run(self) -> None:
        while not self._stop:
            try:
                item = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            kind = item[0]
            if kind == "STATUS":
                self.status.emit(item[1])
            elif kind == "DET":
                self.detections.emit(item[1])
            elif kind == "V":
                self.video.emit(item[1], item[2])
            elif kind == "D":
                self.depth.emit(item[1], item[2])

    def stop(self) -> None:
        self._stop = True
        self.wait(2000)


class CameraController(QObject):
    status = pyqtSignal(dict)
    detections = pyqtSignal(dict)
    video = pyqtSignal(object, object)
    depth = pyqtSignal(object, object)

    def __init__(self):
        super().__init__()
        self.q: "queue.Queue" = queue.Queue(maxsize=12)         # bounded: the oldest frames are dropped if the GUI is slow
        self.client = CameraClient(self.q)
        self._pump = _Pump(self.q)
        for sig in ("status", "detections", "video", "depth"):
            getattr(self._pump, sig).connect(getattr(self, sig))
        self._pump.start()
        self._sim_server = None

    @property
    def simulated_running(self) -> bool:
        return self._sim_server is not None

    def start(self, host: str, port: int) -> None:
        self.stop()
        self.client.start(host, port)

    def start_simulated(self) -> None:
        """Run the real Pi server in-process with a fake camera and connect to it (test mode; needs the pi/ folder)."""
        self.stop()
        if PI_DIR not in sys.path:
            sys.path.insert(0, PI_DIR)
        import pi_camera_server as srv                      # noqa: E402
        from fake_camera import FakeDetector, FakeStereoCamera  # noqa: E402
        cfg = srv.load_config(os.path.join(PI_DIR, "config.yaml"))
        cfg["network"].update(tcp_host="127.0.0.1", tcp_port=0, http_port=None, target_fps=15)
        cfg["stereo"].update(calibration_file="/nonexistent", nominal_focal_px=726.0, nominal_baseline_mm=62.2,
                             num_disparities=64, min_disparity=0)
        cfg["depth"].update(min_z_mm=100.0, max_z_mm=3000.0)
        cfg["dataset"]["enabled"] = False
        self._sim_server = srv.PiCameraServer(cfg, FakeStereoCamera(), FakeDetector(), cfg["dataset"])
        self._sim_server.start()
        self.client.start("127.0.0.1", self._sim_server.tcp_port)

    def stop(self) -> None:
        self.client.stop()
        if self._sim_server is not None:
            self._sim_server.stop()
            self._sim_server = None

    def shutdown(self) -> None:
        self.stop()
        self._pump.stop()

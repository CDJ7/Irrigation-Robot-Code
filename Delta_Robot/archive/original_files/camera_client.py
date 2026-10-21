"""
camera_client.py
-------------------
Client side of the Pi camera link. Imported by the main GUI
(delta_robot_combined_v3.py) -- NOT run standalone.

Connects over plain TCP to pi_camera_server.py running on the Raspberry
Pi, reads the length-prefixed frame stream, decodes JPEG bytes with
OpenCV, converts to RGB, and pushes (tag, numpy_array) onto a queue for
the GUI's Tkinter main loop to display. All decoding happens in a
background thread so the GUI never blocks waiting on the network.

Wire protocol (matches pi_camera_server.py):
    Repeating frames, each:
        1 byte  : tag -- b'V' (video) or b'D' (depth, already colorized)
        4 bytes : big-endian uint32 length of the JPEG payload
        N bytes : JPEG-encoded image data

Requires: pip install opencv-python numpy
(pillow is required by the GUI side for display, not by this module)
"""

import socket
import struct
import threading
import queue
from typing import Optional

import numpy as np
import cv2


class CameraClient:
    def __init__(self, frame_queue: "queue.Queue"):
        self.frame_queue = frame_queue
        self.sock: Optional[socket.socket] = None
        self._stop_flag = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self, host: str, port: int, timeout: float = 5.0) -> None:
        self.stop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.settimeout(None)  # blocking reads once connected; thread is daemonized
        self.sock = sock
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_flag.set()
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def is_connected(self) -> bool:
        return self.sock is not None

    # ---------------------------------------------------
    def _recv_exact(self, n: int) -> Optional[bytes]:
        """Reads exactly n bytes from the socket, or returns None if the
        connection closed/errored partway through."""
        buf = b""
        while len(buf) < n:
            if self._stop_flag.is_set() or self.sock is None:
                return None
            try:
                chunk = self.sock.recv(n - len(buf))
            except OSError:
                return None
            if not chunk:
                return None  # connection closed
            buf += chunk
        return buf

    def _recv_loop(self) -> None:
        while not self._stop_flag.is_set():
            header = self._recv_exact(5)
            if header is None:
                break
            tag = chr(header[0])
            (length,) = struct.unpack(">I", header[1:5])
            payload = self._recv_exact(length)
            if payload is None:
                break

            jpg_array = np.frombuffer(payload, dtype=np.uint8)
            img_bgr = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)
            if img_bgr is None:
                continue
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            if tag in ("V", "D"):
                try:
                    self.frame_queue.put_nowait((tag, img_rgb))
                except queue.Full:
                    pass  # drop frame if the GUI is falling behind

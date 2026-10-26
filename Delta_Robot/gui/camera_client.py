"""
camera_client.py -- the GUI side of the Pi camera link.   (Replaces the original, which is kept in archive/.)

Connects to pi_camera_server.py over TCP, reads the message stream, decodes the pictures and puts the results on a
queue that the GUI polls. It uses ONLY threads and plain Python (no Tkinter/Qt), so the same file works for the
current Tkinter GUI and for the planned PyQt6 GUI. Nothing here can send a command to the robot.

What it does that the original did not:
  * the connection state is REAL: it is "connected" only while data really flows (`is_connected()`, `status`)
  * it reconnects by itself after a drop (1 s, 2 s, 3 s ... up to 5 s between attempts)
  * a link that goes silent for `recv_timeout` seconds is treated as dead
  * it understands the 'J' (detections) message and tells you which 'V'/'D' pictures belong to it
  * the queue is bounded: if the GUI is slow, the OLDEST items are dropped, never the program's memory

Message stream (see the top of pi/pi_camera_server.py for the full description):
    1 byte tag | 4 byte big-endian length | payload
    'J' = JSON detections, 'V' = JPEG left-eye picture, 'D' = JPEG colourised depth picture
For every frame the server sends J, V, D in that order, so the V and D that follow a J carry that J's "seq".

Items put on the queue (always tuples; the first element says what it is):
    ("STATUS", {"state": "connecting"|"connected"|"disconnected"|"stopped", "message": str})
    ("DET",    {...the whole J message: seq, t, calibrated, model, detections, ...})
    ("V",      rgb_array, seq)        left-eye picture, RGB uint8 (H, W, 3);  seq may be None
    ("D",      rgb_array, seq)        depth picture,    RGB uint8 (H, W, 3)

Requires: numpy, opencv-python   (pip install numpy opencv-python)
"""
import json
import queue
import socket
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np

MAX_MESSAGE_BYTES = 32 * 1024 * 1024   # a "length" bigger than this means the stream is garbled -> reconnect


class CameraClient:
    def __init__(self, frame_queue: "queue.Queue", recv_timeout: float = 5.0, connect_timeout: float = 4.0):
        self.frame_queue = frame_queue
        self.recv_timeout = recv_timeout
        self.connect_timeout = connect_timeout
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status = {"state": "stopped", "message": "not started"}
        self._last_seq: Optional[int] = None
        self.frames_received = 0

    # ---------------------------------------------------------------- public
    def start(self, host: str, port: int) -> None:
        """Begin connecting (returns immediately; never blocks the GUI). Safe to call again to change address."""
        self.stop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(host, port), daemon=True, name="camera-client")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close_socket()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2.0)
        self._thread = None
        self._set_status("stopped", "disconnected by user")

    def is_connected(self) -> bool:
        """True only while the link is really up (it turns False as soon as the connection drops)."""
        with self._lock:
            return self._status["state"] == "connected"

    @property
    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    # --------------------------------------------------------------- helpers
    def _set_status(self, state: str, message: str = "") -> None:
        with self._lock:
            if self._status["state"] == state and self._status["message"] == message:
                return
            self._status = {"state": state, "message": message}
        self._put(("STATUS", {"state": state, "message": message}))

    def _put(self, item) -> None:
        """Put on the queue; if it is full, drop the OLDEST item(s) so the newest always gets in."""
        for _ in range(100):
            try:
                self.frame_queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass

    def _close_socket(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            for fn in (lambda: sock.shutdown(socket.SHUT_RDWR), sock.close):
                try:
                    fn()
                except OSError:
                    pass

    def _recv_exact(self, n: int) -> bytes:
        """Read exactly n bytes or raise ConnectionError (closed, timed out, or told to stop)."""
        chunks, got = [], 0
        while got < n:
            if self._stop.is_set():
                raise ConnectionError("stopped")
            sock = self._sock
            if sock is None:
                raise ConnectionError("socket closed")
            try:
                chunk = sock.recv(min(n - got, 65536))
            except socket.timeout:
                raise ConnectionError(f"no data for {self.recv_timeout:g} s")
            except OSError as ex:
                raise ConnectionError(str(ex))
            if not chunk:
                raise ConnectionError("server closed the connection")
            chunks.append(chunk)
            got += len(chunk)
        return b"".join(chunks)

    # ------------------------------------------------------------ the thread
    def _run(self, host: str, port: int) -> None:
        attempt = 0
        while not self._stop.is_set():
            self._set_status("connecting", f"connecting to {host}:{port} ...")
            try:
                sock = socket.create_connection((host, port), timeout=self.connect_timeout)
                sock.settimeout(self.recv_timeout)
                self._sock = sock
            except OSError as ex:
                attempt += 1
                self._set_status("disconnected", f"cannot connect to {host}:{port} ({ex}); retrying")
                self._sleep(min(attempt, 5))
                continue
            attempt = 0
            self._last_seq = None
            try:
                self._receive_loop()
            except ConnectionError as ex:
                if not self._stop.is_set():
                    self._set_status("disconnected", f"connection lost: {ex}; reconnecting")
            finally:
                self._close_socket()
            if not self._stop.is_set():
                attempt += 1
                self._sleep(min(attempt, 5))

    def _sleep(self, seconds: float) -> None:
        self._stop.wait(seconds)

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            header = self._recv_exact(5)
            tag = chr(header[0])
            (length,) = struct.unpack(">I", header[1:5])
            if length > MAX_MESSAGE_BYTES:
                raise ConnectionError(f"garbled stream (message length {length})")
            payload = self._recv_exact(length)
            # Only now do we know data really flows -> "connected". (A server that accepts but sends nothing
            # never shows as connected.)
            self._set_status("connected", "receiving data")

            if tag == "J":
                try:
                    data = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                self._last_seq = data.get("seq")
                self._put(("DET", data))
            elif tag in ("V", "D"):
                img_bgr = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img_bgr is None:
                    continue
                self.frames_received += 1
                self._put((tag, cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB), self._last_seq))
            # unknown tags are ignored on purpose, so newer servers keep working with older clients

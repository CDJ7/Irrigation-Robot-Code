"""
pi_camera_server.py
----------------------
Runs ON the Raspberry Pi (NOT on the main GUI computer). Captures video,
computes a depth map, and streams both to any connected client
(delta_robot_combined_v3.py, via camera_client.py) over a plain TCP
socket.

This script is standalone by design -- the main GUI can't launch code on
a separate physical device over the network without SSH/remote-exec
tooling, which isn't set up here. Start this manually on the Pi, or set
it up to run automatically at boot (see the systemd notes at the bottom
of this file).

REQUIRES ON THE PI:
    pip install opencv-python-headless numpy
    (headless avoids pulling in GUI libs the Pi doesn't need for this)

CAMERA INPUT:
    Uses cv2.VideoCapture(0) by default, which works for most USB
    webcams and, on many Raspberry Pi OS versions, the Pi Camera Module
    via the libcamera V4L2 compatibility layer. If that doesn't pick up
    your camera:
      - Legacy Pi camera stack: try `sudo modprobe bcm2835-v4l2` first.
      - Newer libcamera-only setups: you may need `libcamera-vid` piped
        into this script, or the `picamera2` library's array capture
        instead of cv2.VideoCapture. That's a straightforward swap in
        capture_frame() below once you tell me which camera you're on.

DEPTH INPUT -- IMPORTANT:
    compute_depth_map() below is a PLACEholder. It does NOT compute real
    depth -- it fakes a depth-like image from brightness so the pipeline
    is testable end-to-end before you wire up real depth hardware.
    Replace it with your actual sensor/algorithm, for example:
      - Intel RealSense: `pyrealsense2`, read the aligned depth frame.
      - Stereo camera pair: `cv2.StereoBM_create()` / `StereoSGBM_create()`
        disparity between two synced cameras.
      - A dedicated depth camera (OAK-D, etc.): use its SDK's depth output.
    Whatever you use, just return a single-channel (or already-colorized
    3-channel) numpy array the same size as the video frame from that
    function, and the rest of this script doesn't need to change.
"""

import socket
import struct
import threading
import time

import numpy as np
import cv2

# =========================================================
# CONFIG -- edit these for your setup
# =========================================================
LISTEN_HOST = "0.0.0.0"   # listen on all interfaces
LISTEN_PORT = 8485        # must match the GUI's "Port" field
CAMERA_INDEX = 0          # cv2.VideoCapture index -- adjust if needed
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 15
JPEG_QUALITY = 70         # 0-100, lower = smaller/faster but blockier


def capture_frame(cap: cv2.VideoCapture):
    """Grabs one video frame. Swap this out if you're not using
    cv2.VideoCapture (e.g. picamera2's capture_array())."""
    ok, frame = cap.read()
    if not ok:
        return None
    return frame


def compute_depth_map(frame_bgr: np.ndarray) -> np.ndarray:
    """PLACEHOLDER depth computation -- see the module docstring.
    Currently: grayscale -> blurred -> colorized, purely for testing
    the video pipeline. This is NOT real depth data."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 0)
    depth_colored = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    return depth_colored


def encode_jpeg(img_bgr: np.ndarray, quality: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


def send_frame(conn: socket.socket, tag: str, jpeg_bytes: bytes) -> None:
    header = tag.encode("ascii") + struct.pack(">I", len(jpeg_bytes))
    conn.sendall(header + jpeg_bytes)


def handle_client(conn: socket.socket, addr) -> None:
    print(f"[pi_camera_server] Client connected: {addr}")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("[pi_camera_server] ERROR: could not open camera. "
              "Check CAMERA_INDEX and that the camera is enabled.")
        conn.close()
        return

    frame_interval = 1.0 / TARGET_FPS
    try:
        while True:
            start = time.monotonic()

            frame = capture_frame(cap)
            if frame is None:
                print("[pi_camera_server] WARNING: frame grab failed, retrying...")
                time.sleep(0.1)
                continue

            depth_frame = compute_depth_map(frame)

            video_jpg = encode_jpeg(frame, JPEG_QUALITY)
            depth_jpg = encode_jpeg(depth_frame, JPEG_QUALITY)

            send_frame(conn, "V", video_jpg)
            send_frame(conn, "D", depth_jpg)

            elapsed = time.monotonic() - start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except (BrokenPipeError, ConnectionResetError, OSError):
        print(f"[pi_camera_server] Client disconnected: {addr}")
    finally:
        cap.release()
        conn.close()


def main():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((LISTEN_HOST, LISTEN_PORT))
    server_sock.listen(1)
    print(f"[pi_camera_server] Listening on {LISTEN_HOST}:{LISTEN_PORT} ...")

    try:
        while True:
            conn, addr = server_sock.accept()
            # One client at a time is plenty for this use case; each
            # connection gets its own camera handle/thread.
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("[pi_camera_server] Shutting down.")
    finally:
        server_sock.close()


if __name__ == "__main__":
    main()

# =========================================================
# RUNNING THIS AT BOOT (optional) -- systemd service example
# =========================================================
# Save as /etc/systemd/system/pi-camera-server.service on the Pi:
#
#   [Unit]
#   Description=Delta Robot Pi Camera Server
#   After=network.target
#
#   [Service]
#   ExecStart=/usr/bin/python3 /home/pi/pi_camera_server.py
#   Restart=on-failure
#   User=pi
#
#   [Install]
#   WantedBy=multi-user.target
#
# Then: sudo systemctl enable --now pi-camera-server.service

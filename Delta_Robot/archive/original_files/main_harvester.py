# main_harvester.py
# Full system: Camera + YOLO + Stereo Depth + ESP32 Serial
# Camera is GRAYSCALE — optimized for monochrome stereo camera
# View stream: http://deltapi.local:5000
# Run: python3 main_harvester.py

import cv2
import numpy as np
import threading
import time
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from ultralytics import YOLO

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ============================================================
# SETTINGS — Only change values in this section!
# ============================================================
CAMERA_PATH  = "/dev/video0"
SERIAL_PORT  = '/dev/ttyUSB0'  # Check with: ls /dev/ttyUSB*
BAUD_RATE    = 115200

# Measure the physical distance between the two camera lenses
# with a ruler, then put that number here in millimeters
BASELINE     = 60.0     # mm  ← MEASURE AND REPLACE THIS!
FOCAL_LENGTH = 600.0    # px  ← replace after calibration

HARVEST_Z    = -150.0   # mm — depth to descend for harvesting
HARVEST_WAIT = 1.5      # seconds — vacuum hold time

STREAM_PORT  = 5000     # http://deltapi.local:5000
CONFIDENCE   = 0.3      # YOLO confidence threshold (30%)
# ============================================================

# ── Serial connection ─────────────────────────────────────
print("Connecting to ESP32...")
ser = None
if SERIAL_AVAILABLE:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)
        print("ESP32 connected!")
    except Exception as e:
        print(f"ESP32 not found: {e}")
        print("Running in TEST MODE — serial commands printed only")
else:
    print("pyserial not installed — TEST MODE")

# ── YOLO ──────────────────────────────────────────────────
print("Loading YOLO...")
model = YOLO('yolov8n.pt')
print("YOLO ready!")

# ── Camera ────────────────────────────────────────────────
cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
if not cap.isOpened():
    print("ERROR: Cannot open camera")
    exit(1)
# Camera is grayscale — no color settings needed
# Just set exposure to auto
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
print("Camera opened! (grayscale mode)")

# ── Stereo engine ─────────────────────────────────────────
# StereoBM works best with grayscale — this camera is perfect for it
stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5)

# ── Shared globals ────────────────────────────────────────
latest_frame  = None
stream_lock   = threading.Lock()
system_status = "Starting..."

# ============================================================
# INVERSE KINEMATICS
# Replace the 3 placeholder lines with your real IK code!
# ============================================================
def inverse_kinematics(x_mm, y_mm, z_mm):
    """
    Input : XYZ position in mm
    Output: motor step counts (steps1, steps2, steps3)
    Paste your existing GUI IK calculation here!
    """
    # ↓↓ REPLACE these 3 lines with your real IK code ↓↓
    steps1 = int(x_mm * 10)    # placeholder!
    steps2 = int(y_mm * 10)    # placeholder!
    steps3 = int(z_mm * 10)    # placeholder!
    # ↑↑ replace up to here ↑↑
    return steps1, steps2, steps3

# ============================================================
# CORE FUNCTIONS
# ============================================================
def send_command(cmd):
    """Send text command to ESP32. Prints in test mode."""
    if ser is None:
        print(f"[TEST] Send: {cmd}")
        return "OK"
    ser.write((cmd + '\n').encode())
    resp = ser.readline().decode().strip()
    print(f"ESP32: {resp}")
    return resp

def go_home():
    global system_status
    system_status = "Homing..."
    print("Homing...")
    send_command("H")
    time.sleep(4)
    system_status = "Home done — scanning"
    print("Homed!")

def move_to(x_mm, y_mm, z_mm):
    global system_status
    s1, s2, s3 = inverse_kinematics(x_mm, y_mm, z_mm)
    cmd = f"M {s1} {s2} {s3} 1500"
    system_status = f"Moving X:{x_mm:.0f} Y:{y_mm:.0f} Z:{z_mm:.0f}mm"
    print(f"Cmd: {cmd}")
    send_command(cmd)

def get_depth(disparity_map, px, py):
    """
    Get real-world XYZ in mm from pixel location.
    Returns (x,y,z) in mm or (None,None,None) if failed.
    """
    h, w = disparity_map.shape[:2]
    # Make sure pixel is inside image bounds
    if not (0 <= py < h and 0 <= px < w):
        return None, None, None
    d = disparity_map[py, px]
    if d <= 0:
        return None, None, None
    # Core stereo formula
    z = (BASELINE * FOCAL_LENGTH) / d
    x = (px - w / 2) * z / FOCAL_LENGTH
    y = (py - h / 2) * z / FOCAL_LENGTH
    return x, y, z

# ============================================================
# MAIN PROCESSING LOOP
# ============================================================
def processing_loop():
    global latest_frame, system_status
    os.makedirs("/home/deltarobot/harvest_log", exist_ok=True)
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
        frame_count += 1

        # ── Split stereo frame ────────────────────────────
        # Full frame is 640x240 — left eye is left 320px
        mid   = frame.shape[1] // 2        # = 320
        left  = frame[:, :mid]             # left  eye (320x240)
        right = frame[:, mid:]             # right eye (320x240)

        # ── Stereo depth ──────────────────────────────────
        # Camera outputs grayscale as 3-channel — take just 1 channel
        # This is the fix for the FAKE COLOR issue!
        gray_l = left[:, :, 0]            # take red channel = grayscale
        gray_r = right[:, :, 0]           # same for right

        disparity = (stereo.compute(gray_l, gray_r)
                          .astype(np.float32) / 16.0)

        # ── YOLO detection ────────────────────────────────
        # YOLO needs BGR input — convert grayscale to BGR
        left_bgr = cv2.cvtColor(gray_l, cv2.COLOR_GRAY2BGR)
        results  = model(left_bgr, conf=CONFIDENCE, verbose=False)
        annotated = results[0].plot()

        # ── Make disparity visible as color overlay ───────
        # Normalize disparity to 0-255 and colorize
        disp_vis = cv2.normalize(
            disparity, None, 0, 255,
            cv2.NORM_MINMAX, cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        # Blend depth map onto annotated frame (30% opacity)
        # Depth overlay disabled — uncomment to re-enable
        # annotated = cv2.addWeighted(
        #           annotated, 0.75, disp_color, 0.25, 0)

        # ── Process each detection ────────────────────────
        boxes = results[0].boxes
        system_status = ("Scanning — no objects"
                         if len(boxes) == 0
                         else f"{len(boxes)} object(s) found")

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            cx   = int((x1 + x2) / 2)
            cy   = int((y1 + y2) / 2)
            name = model.names[int(box.cls)]
            conf = float(box.conf) * 100

            rx, ry, rz = get_depth(disparity, cx, cy)

            if rx is not None:
                # Draw XYZ label above box
                label = (f"{name} {conf:.0f}%  "
                         f"X:{rx:.0f} Y:{ry:.0f} Z:{rz:.0f}mm")
                cv2.putText(annotated, label,
                            (int(x1), max(int(y1) - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 2)
                system_status = (f"{name} {conf:.0f}% | "
                                 f"X:{rx:.0f} Y:{ry:.0f} "
                                 f"Z:{rz:.0f}mm")

                # Move robot → harvest → home
                move_to(rx, ry, HARVEST_Z)
                time.sleep(HARVEST_WAIT)
                go_home()

                # Save harvest log image
                log = (f"/home/deltarobot/harvest_log/"
                       f"harvest_{frame_count:05d}.jpg")
                cv2.imwrite(log, annotated)
                print(f"Log: {log}")
            else:
                cv2.putText(annotated,
                            f"{name} {conf:.0f}% | depth N/A",
                            (int(x1), max(int(y1) - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 200, 200), 2)

        # ── Status bar at bottom ──────────────────────────
        h, w = annotated.shape[:2]
        cv2.rectangle(annotated, (0, h-28), (w, h), (0,0,0), -1)
        cv2.putText(annotated, system_status,
                    (6, h-8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)

        # ── Encode for web stream ─────────────────────────
        _, jpeg = cv2.imencode(
            '.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with stream_lock:
            latest_frame = jpeg.tobytes()

        time.sleep(0.04)

# ============================================================
# WEB SERVER
# ============================================================
class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            html = (
                "<html><head><title>Delta Robot</title>"
                "<style>"
                "body{background:#111;color:#0f0;"
                "font-family:monospace;"
                "display:flex;flex-direction:column;"
                "align-items:center;margin:0;padding:16px;}"
                "img{border:2px solid #0f0;"
                "width:640px;max-width:100%;}"
                "p{color:#666;font-size:12px;margin:4px;}"
                "</style></head><body>"
                "<h2>Delta Robot - YOLO + Depth</h2>"
                "<img src='/stream'/>"
                "<p>Blue boxes = YOLO detections</p>"
                "<p>Yellow text = name + XYZ in mm</p>"
                "<p>Color overlay = depth map "
                "(red=close / blue=far)</p>"
                "</body></html>"
            ).encode('ascii')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(html)

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header(
                'Content-Type',
                'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with stream_lock:
                        f = latest_frame
                    if f:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(
                            b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(f)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError):
                pass

# ============================================================
# START
# ============================================================
t = threading.Thread(target=processing_loop, daemon=True)
t.start()

server = HTTPServer(('0.0.0.0', STREAM_PORT), StreamHandler)
print("")
print("=" * 45)
print("  Delta Robot started!")
print(f"  Browser: http://deltapi.local:{STREAM_PORT}")
print("  Ctrl+C to stop")
print("=" * 45)

go_home()

try:
    server.serve_forever()
except KeyboardInterrupt:
    cap.release()
    if ser:
        ser.close()
    print("Stopped.")

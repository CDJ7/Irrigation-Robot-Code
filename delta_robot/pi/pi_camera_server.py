"""
pi_camera_server.py  --  runs ON the Raspberry Pi. VISION ONLY.
==================================================================
Captures the stereo camera, computes REAL depth, runs YOLO, and streams the
results to any number of clients. It never talks to the ESP32 and never moves
the robot: the GUI on the PC is the only thing that controls the motors.

WIRE PROTOCOL (TCP, default port 8485)  -- all clients get the same stream
------------------------------------------------------------------------
Every message is:   1 byte tag | 4 byte big-endian length N | N bytes payload

    'J'  payload = UTF-8 JSON  (detections for ONE frame, see below)
    'V'  payload = JPEG of the rectified LEFT eye (grayscale as BGR), 320x240
    'D'  payload = JPEG of the colorized disparity ("depth") map, same size

For every processed frame the server sends   J, V, D   back to back, in that
order. So the 'V' and 'D' that follow a 'J' belong to that J: they have the same
frame sequence number ("seq" inside the JSON). Old clients that only know 'V'
and 'D' keep working: they ignore tags they do not know.

JSON of a 'J' message:
    {
      "seq": 1234,                 frame counter (increases by 1 per processed frame)
      "t": 1730000000.123,         Pi clock, seconds since epoch, when the frame was captured
      "width": 320, "height": 240, size of the V / D images (pixel coordinates below)
      "calibrated": true,          false = nominal numbers, NOT trustworthy for picking
      "units": "mm",
      "camera_frame": "X right, Y down, Z out of the lens (left eye)",
      "intrinsics": {"fx":..,"fy":..,"cx":..,"cy":..,"baseline_mm":..},
      "model": "yolov8n.pt",
      "model_has_allowed_class": false,   true only if the model can output an allowed class
      "allowed_classes": ["strawberry"],
      "server_fps": 14.2,
      "timing_ms": {"capture":..,"stereo":..,"yolo":..,"total":..},
      "center_probe": {"xyz_mm": [X,Y,Z] or null, "disparity": ..},   depth at the image center,
                                   handy for checking the calibration against a ruler
      "detections": [
        {"class": "strawberry", "class_id": 0, "confidence": 0.87, "allowed": true,
         "bbox": [x1,y1,x2,y2],          pixels in the V image
         "center": [u,v],                pixel center of the box
         "xyz_mm": [X,Y,Z] or null,      null when depth is invalid (holes, out of range)
         "disparity": 41.5 or null,      median disparity used, in pixels
         "depth_valid_fraction": 0.8}    share of the patch that had valid disparity
      ]
    }

BROWSER VIEWER (optional, default port 5000):  http://deltapi.local:5000
    /          page with the live stream      /stream   MJPEG (annotated + depth)
    /snapshot.jpg  one annotated frame        /status   JSON with fps etc.

DESIGN
------
* ONE thread (VisionPipeline) owns the camera and does all the work. Any number
  of TCP/browser clients just read the newest finished frame ("latest wins"), so a
  slow client can never make the camera lag or slow other clients down. A client that
  stops reading for send_timeout_s is dropped.
* While nobody is connected the pipeline idles at a low rate to save CPU.
* Depth: both eyes are rectified with the calibration from calibrate_stereo.py, then
  disparity is computed (StereoSGBM). YOLO runs on the rectified left eye, so a box's
  pixels line up exactly with the disparity map.

RUN:   source ~/delta_robot/venv/bin/activate
       python pi_camera_server.py                       # normal
       python pi_camera_server.py --capture-dataset     # also save training images
       python pi_camera_server.py --fake                # synthetic camera, for testing
"""

import argparse
import json
import logging
import os
import signal
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

log = logging.getLogger("pi_camera_server")
HERE = os.path.dirname(os.path.abspath(__file__))


# =========================================================
#   CONFIG
# =========================================================
def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def resolve(path: str) -> str:
    """Relative paths in config.yaml are relative to this folder."""
    return path if os.path.isabs(path) else os.path.join(HERE, path)


# =========================================================
#   CAMERA
# =========================================================
class V4L2Camera:
    """Opens the stereo camera ONCE. read() returns the full side-by-side frame."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.cap: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        c = self.cfg
        cap = cv2.VideoCapture(c["device"], cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        if c.get("fourcc"):
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*c["fourcc"]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, c["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c["height"])
        if c.get("fps"):
            cap.set(cv2.CAP_PROP_FPS, c["fps"])
        if c.get("buffer_size"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, c["buffer_size"])
        if c.get("auto_exposure") is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, c["auto_exposure"])
        self.cap = cap
        return True

    def read(self) -> Optional[np.ndarray]:
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def split_eyes(frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Side-by-side frame -> (left, right) single-channel grayscale images.
    This camera is monochrome but delivers 3 identical channels, so channel 0 is enough."""
    if frame.ndim == 3:
        frame = frame[:, :, 0]
    mid = frame.shape[1] // 2
    return np.ascontiguousarray(frame[:, :mid]), np.ascontiguousarray(frame[:, mid:])


# =========================================================
#   STEREO: rectification + disparity + depth from a disparity
# =========================================================
class StereoRectifier:
    """Warps both eyes so that matching points lie on the SAME image row (that is what
    the matcher assumes), and holds the Q matrix that turns (u, v, disparity) into mm.

    With no calibration file it passes images through unchanged and builds an
    approximate Q from nominal numbers; `calibrated` is then False."""

    def __init__(self, cfg: dict, eye_size: Tuple[int, int]):
        self.eye_size = eye_size  # (width, height) of ONE eye
        self.calibrated = False
        self.maps = None
        s = cfg["stereo"]
        path = resolve(s["calibration_file"])
        if os.path.exists(path):
            self._load(path)
        else:
            log.warning("No stereo calibration at %s -> UNCALIBRATED mode "
                        "(run calibrate_stereo.py). Depth numbers are NOT trustworthy.", path)
        if not self.calibrated:
            f, B = float(s["nominal_focal_px"]), float(s["nominal_baseline_mm"])
            w, h = eye_size
            cx, cy = w / 2.0, h / 2.0
            # Same layout cv2.stereoRectify produces for an ideal rectified pair.
            self.Q = np.array([[1, 0, 0, -cx],
                               [0, 1, 0, -cy],
                               [0, 0, 0, f],
                               [0, 0, 1.0 / B, 0]], dtype=np.float64)
            self.intrinsics = {"fx": f, "fy": f, "cx": cx, "cy": cy, "baseline_mm": B}

    def _load(self, path: str) -> None:
        d = np.load(path)
        size = tuple(int(v) for v in d["image_size"])
        if size != self.eye_size:
            log.error("Calibration is for %s images but the camera gives %s -> ignoring it. "
                      "Re-run calibrate_stereo.py at the current resolution.", size, self.eye_size)
            return
        m1 = cv2.initUndistortRectifyMap(d["K1"], d["D1"], d["R1"], d["P1"], size, cv2.CV_16SC2)
        m2 = cv2.initUndistortRectifyMap(d["K2"], d["D2"], d["R2"], d["P2"], size, cv2.CV_16SC2)
        self.maps = (m1, m2)
        self.Q = d["Q"].astype(np.float64)
        P1, P2 = d["P1"], d["P2"]
        self.intrinsics = {"fx": float(P1[0, 0]), "fy": float(P1[1, 1]),
                           "cx": float(P1[0, 2]), "cy": float(P1[1, 2]),
                           "baseline_mm": float(-P2[0, 3] / P2[0, 0])}
        self.calibrated = True
        log.info("Loaded stereo calibration %s (rms=%.3f px, baseline=%.1f mm)",
                 path, float(d["rms"]), self.intrinsics["baseline_mm"])

    def rectify(self, left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.maps is None:
            return left, right
        (m1a, m1b), (m2a, m2b) = self.maps
        return (cv2.remap(left, m1a, m1b, cv2.INTER_LINEAR),
                cv2.remap(right, m2a, m2b, cv2.INTER_LINEAR))

    def to_xyz(self, u: float, v: float, d: float) -> Tuple[float, float, float]:
        """Pixel (u, v) with disparity d -> camera-frame X, Y, Z in mm (what
        cv2.reprojectImageTo3D does for one pixel, using the Q matrix)."""
        p = self.Q @ np.array([u, v, d, 1.0])
        return float(p[0] / p[3]), float(p[1] / p[3]), float(p[2] / p[3])


class DisparityMatcher:
    """Wraps StereoSGBM / StereoBM. compute() returns disparity in float pixels."""

    def __init__(self, s: dict):
        self.min_disp = int(s["min_disparity"])
        self.num_disp = int(s["num_disparities"])
        if self.num_disp % 16 != 0:
            raise ValueError("stereo.num_disparities must be a multiple of 16")
        bs = int(s["block_size"])
        if s["matcher"] == "bm":
            self.m = cv2.StereoBM_create(numDisparities=self.num_disp, blockSize=bs)
            self.m.setMinDisparity(self.min_disp)
            self.m.setUniquenessRatio(int(s["uniqueness_ratio"]))
            self.m.setSpeckleWindowSize(int(s["speckle_window"]))
            self.m.setSpeckleRange(int(s["speckle_range"]))
            self.m.setDisp12MaxDiff(int(s["disp12_max_diff"]))
            self.m.setPreFilterCap(int(s["pre_filter_cap"]))
        else:
            self.m = cv2.StereoSGBM_create(
                minDisparity=self.min_disp, numDisparities=self.num_disp, blockSize=bs,
                P1=8 * bs * bs, P2=32 * bs * bs,   # smoothness penalties (usual rule of thumb)
                disp12MaxDiff=int(s["disp12_max_diff"]),
                uniquenessRatio=int(s["uniqueness_ratio"]),
                speckleWindowSize=int(s["speckle_window"]),
                speckleRange=int(s["speckle_range"]),
                preFilterCap=int(s["pre_filter_cap"]),
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)   # 3-way: cheaper than full HH, good enough
        self.min_valid = float(s["min_valid_disparity"])

    def compute(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        # The matcher returns fixed-point numbers: real disparity = value / 16.
        return self.m.compute(left, right).astype(np.float32) / 16.0

    def valid_mask(self, disp: np.ndarray) -> np.ndarray:
        return disp >= max(self.min_valid, self.min_disp)


def colorize_disparity(disp: np.ndarray, matcher: DisparityMatcher) -> np.ndarray:
    """Fixed-scale colormap (does not flicker like min/max normalization). Red = near,
    blue = far, black = no data."""
    scaled = np.clip((disp - matcher.min_disp) / max(matcher.num_disp, 1) * 255.0, 0, 255)
    color = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_JET)
    color[~matcher.valid_mask(disp)] = 0
    return color


def box_depth(disp: np.ndarray, matcher: DisparityMatcher, rect: StereoRectifier,
              box: Tuple[float, float, float, float], dcfg: dict) -> Dict:
    """Depth of one detection: MEDIAN of the valid disparities in the middle of its box
    (holes ignored), then converted to mm. Returns a dict, xyz_mm is None if unusable."""
    h, w = disp.shape
    x1, y1, x2, y2 = box
    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    pw, ph = (x2 - x1) * dcfg["patch_fraction"], (y2 - y1) * dcfg["patch_fraction"]
    xa, xb = int(max(0, round(u - pw / 2))), int(min(w, round(u + pw / 2) + 1))
    ya, yb = int(max(0, round(v - ph / 2))), int(min(h, round(v + ph / 2) + 1))
    out = {"xyz_mm": None, "disparity": None, "depth_valid_fraction": 0.0}
    if xb <= xa or yb <= ya:
        return out
    patch = disp[ya:yb, xa:xb]
    vals = patch[matcher.valid_mask(patch)]
    out["depth_valid_fraction"] = round(float(vals.size) / patch.size, 3)
    if vals.size < dcfg["min_pixels"] or out["depth_valid_fraction"] < dcfg["min_valid_fraction"]:
        return out
    d = float(np.median(vals))
    out["disparity"] = round(d, 2)
    X, Y, Z = rect.to_xyz(u, v, d)
    if not (dcfg["min_z_mm"] <= Z <= dcfg["max_z_mm"]) or not np.isfinite(Z):
        return out
    out["xyz_mm"] = [round(X, 1), round(Y, 1), round(Z, 1)]
    return out


# =========================================================
#   DETECTOR
# =========================================================
class YoloDetector:
    """Ultralytics YOLO wrapper. The model file is one line in config.yaml, so swapping in
    a custom-trained strawberry model (.pt or an exported NCNN folder) needs no code change."""

    def __init__(self, dcfg: dict):
        from ultralytics import YOLO  # imported here so --fake / tests do not need it
        self.cfg = dcfg
        self.model_path = dcfg["model"]
        path = resolve(self.model_path) if os.path.exists(resolve(self.model_path)) else self.model_path
        log.info("Loading YOLO model %s ...", path)
        self.model = YOLO(path)
        names = self.model.names
        self.names: Dict[int, str] = dict(names) if isinstance(names, dict) else dict(enumerate(names))
        # Warm-up so the first real frame is not slow.
        self.model(np.zeros((240, 320, 3), np.uint8), imgsz=dcfg["imgsz"], verbose=False)

    def detect(self, left_gray: np.ndarray) -> List[Dict]:
        bgr = cv2.cvtColor(left_gray, cv2.COLOR_GRAY2BGR)  # YOLO expects 3 channels
        r = self.model(bgr, conf=self.cfg["confidence"], imgsz=self.cfg["imgsz"],
                       max_det=self.cfg["max_detections"], verbose=False)[0]
        out = []
        for b in r.boxes:
            cid = int(b.cls)
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            out.append({"class_id": cid, "class": self.names.get(cid, str(cid)),
                        "confidence": float(b.conf), "bbox": (x1, y1, x2, y2)})
        return out


# =========================================================
#   SHARED STATE between the pipeline thread and the clients
# =========================================================
class Bundle:
    """One finished frame, already encoded, ready to hand to every client."""
    def __init__(self, seq: int, ts: float):
        self.seq = seq
        self.ts = ts
        self.wire: Optional[bytes] = None        # J + V + D for TCP clients
        self.mjpeg: Optional[bytes] = None       # annotated image for browsers
        self.status: Dict = {}


def frame_msg(tag: bytes, payload: bytes) -> bytes:
    return tag + struct.pack(">I", len(payload)) + payload


class Hub:
    """Latest-frame-wins mailbox. The pipeline publishes; clients wait for newer frames."""
    def __init__(self):
        self.cond = threading.Condition()
        self.bundle: Optional[Bundle] = None
        self.tcp_clients = 0
        self.http_clients = 0
        self.stopped = False

    def publish(self, b: Bundle) -> None:
        with self.cond:
            self.bundle = b
            self.cond.notify_all()

    def wait_newer(self, last_seq: int, timeout: float) -> Optional[Bundle]:
        with self.cond:
            self.cond.wait_for(lambda: self.stopped or (self.bundle and self.bundle.seq > last_seq),
                               timeout)
            if self.bundle and self.bundle.seq > last_seq:
                return self.bundle
            return None

    def adjust(self, kind: str, delta: int) -> None:
        with self.cond:
            if kind == "tcp":
                self.tcp_clients += delta
            else:
                self.http_clients += delta

    def consumers(self) -> int:
        return self.tcp_clients + self.http_clients

    def stop(self) -> None:
        with self.cond:
            self.stopped = True
            self.cond.notify_all()


# =========================================================
#   THE ONE VISION THREAD
# =========================================================
class VisionPipeline(threading.Thread):
    def __init__(self, cfg: dict, hub: Hub, camera, detector, dataset_cfg: dict):
        super().__init__(daemon=True, name="vision")
        self.cfg, self.hub, self.camera, self.detector = cfg, hub, camera, detector
        self.dataset = dataset_cfg
        self.stop_flag = threading.Event()
        self.seq = 0
        self.fps = 0.0
        self.camera_ok = False
        self.dataset_count = 0
        self._last_dataset_t = 0.0
        self.allowed = {c.lower() for c in cfg["detection"]["allowed_classes"]}
        self.rect: Optional[StereoRectifier] = None
        self.matcher = DisparityMatcher(cfg["stereo"])

    # ---- helpers -------------------------------------------------------
    def _open_camera(self) -> bool:
        while not self.stop_flag.is_set():
            if self.camera.open():
                return True
            log.error("Cannot open camera %s. Is another program (e.g. the old "
                      "main_harvester.py) holding it?  Check:  fuser -v /dev/video0",
                      self.cfg["camera"]["device"])
            self.stop_flag.wait(2.0)
        return False

    def _model_has_allowed(self) -> bool:
        if self.detector is None:
            return False
        return any(n.lower() in self.allowed for n in self.detector.names.values())

    def _maybe_save_dataset(self, left_img: np.ndarray, now: float) -> None:
        d = self.dataset
        if not d.get("enabled"):
            return
        if now - self._last_dataset_t < d["interval_s"]:
            return
        if d["max_images"] and self.dataset_count >= d["max_images"]:
            return
        folder = resolve(d["dir"])
        os.makedirs(folder, exist_ok=True)
        name = time.strftime("img_%Y%m%d_%H%M%S", time.localtime(now)) + f"_{int(now * 1000) % 1000:03d}.png"
        cv2.imwrite(os.path.join(folder, name), left_img)   # lossless PNG, grayscale
        self.dataset_count += 1
        self._last_dataset_t = now
        if self.dataset_count % 10 == 1:
            log.info("Dataset: %d images saved in %s", self.dataset_count, folder)

    def _draw(self, left: np.ndarray, depth_color: np.ndarray, dets: List[Dict],
              calibrated: bool, fps: float) -> np.ndarray:
        img = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
        for d in dets:
            x1, y1, x2, y2 = [int(v) for v in d["bbox"]]
            color = (0, 255, 0) if d["allowed"] else (0, 0, 255)  # red = NOT allowed to pick
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 1)
            txt = f"{d['class']} {d['confidence']*100:.0f}%"
            if d["xyz_mm"]:
                txt += " Z%.0f" % d["xyz_mm"][2]
            else:
                txt += " no depth"
            cv2.putText(img, txt, (x1, max(y1 - 3, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        status = f"{fps:.1f} fps" + ("" if calibrated else "  UNCALIBRATED")
        cv2.putText(img, status, (4, img.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 255) if calibrated else (0, 0, 255), 1)
        return np.hstack([img, depth_color])

    # ---- main loop -----------------------------------------------------
    def run(self) -> None:
        net, cfgd = self.cfg["network"], self.cfg["depth"]
        q = [int(cv2.IMWRITE_JPEG_QUALITY), int(net["jpeg_quality"])]
        if not self._open_camera():
            return
        fails = 0
        frame_period = 1.0 / net["target_fps"]
        fps_t0, fps_n = time.monotonic(), 0
        last_report = time.monotonic()

        while not self.stop_flag.is_set():
            t_start = time.monotonic()
            frame = self.camera.read()
            if frame is None:
                fails += 1
                if fails >= 30:   # camera stalled/unplugged: reopen it
                    log.error("Camera gave no frames, reopening...")
                    self.camera_ok = False
                    self.camera.close()
                    if not self._open_camera():
                        return
                    fails = 0
                else:
                    time.sleep(0.05)
                continue
            fails = 0
            self.camera_ok = True
            t_cap = time.monotonic()
            capture_ts = time.time()

            wanted = self.hub.consumers() > 0 or self.dataset.get("enabled")
            if not wanted:                    # nobody watching: just drain the camera slowly
                self.stop_flag.wait(max(0.0, 1.0 / net["idle_fps"] - (time.monotonic() - t_start)))
                continue

            left, right = split_eyes(frame)
            if self.rect is None:             # created on first frame so we know the eye size
                self.rect = StereoRectifier(self.cfg, (left.shape[1], left.shape[0]))
            left, right = self.rect.rectify(left, right)
            self._maybe_save_dataset(left, capture_ts)

            t0 = time.monotonic()
            disp = self.matcher.compute(left, right)
            t_stereo = time.monotonic()

            dets: List[Dict] = []
            if self.detector is not None:
                for r in self.detector.detect(left):
                    allowed = r["class"].lower() in self.allowed
                    if self.cfg["detection"]["report_only_allowed"] and not allowed:
                        continue
                    info = box_depth(disp, self.matcher, self.rect, r["bbox"], cfgd)
                    x1, y1, x2, y2 = r["bbox"]
                    dets.append({
                        "class": r["class"], "class_id": r["class_id"],
                        "confidence": round(r["confidence"], 3), "allowed": allowed,
                        "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                        "center": [int(round((x1 + x2) / 2)), int(round((y1 + y2) / 2))],
                        **info})
            t_yolo = time.monotonic()
            hh, ww = disp.shape
            probe = box_depth(disp, self.matcher, self.rect,
                              (ww / 2 - 30, hh / 2 - 30, ww / 2 + 30, hh / 2 + 30), cfgd)

            self.seq += 1
            b = Bundle(self.seq, capture_ts)
            depth_color = colorize_disparity(disp, self.matcher)
            n_tcp, n_http = self.hub.tcp_clients, self.hub.http_clients
            timing = {"capture": round((t_cap - t_start) * 1000, 1),
                      "stereo": round((t_stereo - t0) * 1000, 1),
                      "yolo": round((t_yolo - t_stereo) * 1000, 1)}
            timing["total"] = round((time.monotonic() - t_start) * 1000, 1)
            meta = {
                "seq": self.seq, "t": round(capture_ts, 3),
                "width": left.shape[1], "height": left.shape[0],
                "calibrated": self.rect.calibrated, "units": "mm",
                "camera_frame": "X right, Y down, Z out of the lens (left eye)",
                "intrinsics": {k: round(v, 3) for k, v in self.rect.intrinsics.items()},
                "model": self.cfg["detection"]["model"] if self.detector else None,
                "model_has_allowed_class": self._model_has_allowed(),
                "allowed_classes": sorted(self.allowed),
                "server_fps": round(self.fps, 1), "timing_ms": timing,
                "center_probe": {"xyz_mm": probe["xyz_mm"], "disparity": probe["disparity"]},
                "detections": dets}
            b.status = {k: meta[k] for k in ("seq", "calibrated", "model", "model_has_allowed_class",
                                             "server_fps", "timing_ms")}
            b.status.update({"center_probe": meta["center_probe"], "camera_ok": True, "tcp_clients": n_tcp, "http_clients": n_http,
                             "dataset_images": self.dataset_count, "detections": len(dets)})

            if n_tcp > 0:
                ok_v, jv = cv2.imencode(".jpg", left, q)       # JPEG encoder accepts gray
                ok_d, jd = cv2.imencode(".jpg", depth_color, q)
                if ok_v and ok_d:
                    # J goes right BEFORE its V (and D) so a client can match them.
                    b.wire = (frame_msg(b"J", json.dumps(meta).encode("utf-8"))
                              + frame_msg(b"V", jv.tobytes()) + frame_msg(b"D", jd.tobytes()))
            if n_http > 0:
                ok, jm = cv2.imencode(".jpg", self._draw(left, depth_color, dets, self.rect.calibrated,
                                                         self.fps), q)
                if ok:
                    b.mjpeg = jm.tobytes()
            self.hub.publish(b)

            # FPS (smoothed over ~1 s) and throttle to target_fps.
            fps_n += 1
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                self.fps = fps_n / (now - fps_t0)
                fps_t0, fps_n = now, 0
            if now - last_report >= 10.0:
                last_report = now
                log.info("%.1f fps | stereo %.0f ms | yolo %.0f ms | tcp=%d http=%d | calibrated=%s",
                         self.fps, timing["stereo"], timing["yolo"], n_tcp, n_http, self.rect.calibrated)
            spare = frame_period - (time.monotonic() - t_start)
            if spare > 0:
                self.stop_flag.wait(spare)
        self.camera.close()


# =========================================================
#   NETWORK: TCP clients and the browser viewer
# =========================================================
def serve_tcp_client(conn: socket.socket, addr, hub: Hub, send_timeout: float) -> None:
    log.info("TCP client connected: %s", addr)
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # Small send buffer (~5 frames): if a client is slow, data waits here only briefly, so it
    # can never receive seconds-old frames; and a client that stops reading hits the timeout fast.
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
    conn.settimeout(send_timeout)      # a stuck client raises socket.timeout -> dropped
    hub.adjust("tcp", +1)
    last_seq = 0
    try:
        while not hub.stopped:
            b = hub.wait_newer(last_seq, 1.0)
            if b is None:
                continue
            last_seq = b.seq
            if b.wire is None or time.time() - b.ts > 2.0:
                continue                # not encoded for TCP yet, or stale: wait for the next one
            conn.sendall(b.wire)
    except OSError:
        pass                            # disconnect or timeout
    finally:
        hub.adjust("tcp", -1)
        conn.close()
        log.info("TCP client gone: %s", addr)


def make_http_handler(hub: Hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _wait_mjpeg(self, last: int) -> Optional[Bundle]:
            while not hub.stopped:
                b = hub.wait_newer(last, 1.0)
                if b is not None and b.mjpeg is not None and time.time() - b.ts < 2.0:
                    return b
                if b is not None:
                    last = b.seq
            return None

        def do_GET(self):
            if self.path == "/":
                html = (b"<html><head><title>Delta Robot Vision</title><style>"
                        b"body{background:#111;color:#0f0;font-family:monospace;text-align:center}"
                        b"img{width:960px;max-width:100%;border:2px solid #0f0}</style></head><body>"
                        b"<h3>Left eye + detections (left) | depth: red=near blue=far (right)</h3>"
                        b"<img src='/stream'/><p>green box = allowed class, red box = NOT allowed. "
                        b"Z in mm.</p></body></html>")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(html)
            elif self.path == "/status":
                b = hub.bundle
                body = json.dumps(b.status if b else {"camera_ok": False}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith("/snapshot"):
                hub.adjust("http", +1)
                try:
                    b = self._wait_mjpeg(0)
                finally:
                    hub.adjust("http", -1)
                if b is None:
                    self.send_error(503)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.end_headers()
                self.wfile.write(b.mjpeg)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                hub.adjust("http", +1)
                last = 0
                try:
                    while not hub.stopped:
                        b = self._wait_mjpeg(last)
                        if b is None:
                            break
                        last = b.seq
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(b.mjpeg)
                        self.wfile.write(b"\r\n")
                except OSError:
                    pass
                finally:
                    hub.adjust("http", -1)
            else:
                self.send_error(404)
    return Handler


# =========================================================
#   THE SERVER OBJECT (also used by the tests)
# =========================================================
class PiCameraServer:
    def __init__(self, cfg: dict, camera, detector, dataset_cfg: Optional[dict] = None):
        self.cfg = cfg
        self.hub = Hub()
        self.pipeline = VisionPipeline(cfg, self.hub, camera, detector, dataset_cfg or cfg["dataset"])
        self.tcp_sock: Optional[socket.socket] = None
        self.http: Optional[ThreadingHTTPServer] = None
        self.tcp_port = 0
        self.http_port = 0

    def start(self) -> None:
        n = self.cfg["network"]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((n["tcp_host"], n["tcp_port"]))
        s.listen(8)
        s.settimeout(0.5)
        self.tcp_sock, self.tcp_port = s, s.getsockname()[1]
        threading.Thread(target=self._accept_loop, daemon=True, name="tcp-accept").start()
        if n["http_port"] is not None:   # null in config.yaml = browser viewer off
            self.http = ThreadingHTTPServer((n["tcp_host"], n["http_port"]), make_http_handler(self.hub))
            self.http.daemon_threads = True
            self.http_port = self.http.server_address[1]
            threading.Thread(target=self.http.serve_forever, daemon=True, name="http").start()
        self.pipeline.start()
        log.info("Listening: TCP %d%s", self.tcp_port,
                 f", browser http://deltapi.local:{self.http_port}" if self.http else "")

    def _accept_loop(self) -> None:
        while not self.hub.stopped:
            try:
                conn, addr = self.tcp_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=serve_tcp_client,
                             args=(conn, addr, self.hub, self.cfg["network"]["send_timeout_s"]),
                             daemon=True).start()

    def stop(self) -> None:
        self.hub.stop()
        self.pipeline.stop_flag.set()
        if self.http:
            self.http.shutdown()
            self.http.server_close()
        if self.tcp_sock:
            self.tcp_sock.close()
        self.pipeline.join(timeout=3.0)


def main() -> None:
    ap = argparse.ArgumentParser(description="Delta robot Pi vision server (vision only)")
    ap.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    ap.add_argument("--fake", action="store_true", help="synthetic camera + detector (no hardware)")
    ap.add_argument("--no-yolo", action="store_true", help="depth only, no detection")
    ap.add_argument("--capture-dataset", nargs="?", const="", default=None, metavar="DIR",
                    help="save left-eye grayscale images for training (optional folder)")
    ap.add_argument("--interval", type=float, help="seconds between dataset images")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    dataset_cfg = dict(cfg["dataset"])
    if args.capture_dataset is not None:
        dataset_cfg["enabled"] = True
        if args.capture_dataset:
            dataset_cfg["dir"] = args.capture_dataset
    if args.interval:
        dataset_cfg["interval_s"] = args.interval

    if args.fake:
        from fake_camera import FakeStereoCamera, FakeDetector
        camera, detector = FakeStereoCamera(), FakeDetector()
        cfg["stereo"]["calibration_file"] = "/nonexistent"   # fake camera is uncalibrated
    else:
        camera = V4L2Camera(cfg["camera"])
        detector = None if (args.no_yolo or not cfg["detection"]["enabled"]) else YoloDetector(cfg["detection"])

    server = PiCameraServer(cfg, camera, detector, dataset_cfg)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())   # systemd stop
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    server.start()
    try:
        while not stop.is_set():
            stop.wait(0.5)
    finally:
        log.info("Shutting down.")
        server.stop()


if __name__ == "__main__":
    main()

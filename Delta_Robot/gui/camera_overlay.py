"""
camera_overlay.py -- drawing helpers for the camera view (no GUI toolkit needed, so they can be tested anywhere).

Boxes and labels are drawn from the detections the Pi sends in its 'J' message. Pixel coordinates in a detection refer to
the 'V' picture of the SAME frame (matched by "seq"), so the boxes line up with the image.
"""
from typing import Dict, List, Tuple

import cv2
import numpy as np

GREEN = (60, 220, 60)     # a class that is allowed to be picked
RED = (235, 60, 60)       # detected, but NOT an allowed class (e.g. a person with the stock YOLO model)
GREY = (170, 170, 170)


def detection_label(d: Dict) -> str:
    """Short text for a box: class, confidence and the camera-frame position in millimetres (or 'no depth')."""
    text = f"{d.get('class', '?')} {100 * float(d.get('confidence', 0)):.0f}%"
    xyz = d.get("xyz_mm")
    if xyz:
        text += f"  X{xyz[0]:.0f} Y{xyz[1]:.0f} Z{xyz[2]:.0f}"
    else:
        text += "  no depth"
    return text


def draw_detections(img_rgb: np.ndarray, detections: List[Dict]) -> np.ndarray:
    """Return a COPY of the picture with a box and label per detection."""
    out = img_rgb.copy()
    h, w = out.shape[:2]
    for d in detections or []:
        try:
            x1, y1, x2, y2 = [int(round(v)) for v in d["bbox"]]
        except (KeyError, TypeError, ValueError):
            continue
        color = GREEN if d.get("allowed") else RED
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = detection_label(d)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = y1 - 6 if y1 - th - 8 > 0 else min(h - 4, y2 + th + 6)       # label above the box if there is room
        tx = max(0, min(x1, w - tw - 2))
        cv2.rectangle(out, (tx - 1, ty - th - 3), (tx + tw + 1, ty + 3), (0, 0, 0), -1)
        cv2.putText(out, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def fit_inside(img: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    """Shrink (never enlarge) a picture so it fits in max_w x max_h, keeping its shape."""
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale >= 1.0:
        return img
    return cv2.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))), interpolation=cv2.INTER_AREA)


def status_colour(state: str) -> str:
    return {"connected": "green", "connecting": "#b35c00"}.get(state, "red")


def summarize_frame(det: Dict) -> Tuple[str, bool]:
    """(text for the info line, is_trustworthy) from a J message. Not trustworthy = uncalibrated depth."""
    if det.get("mode") == "calibration_capture":
        c = det.get("calib", {})
        txt = f"CALIBRATION CAPTURE: saved {c.get('saved', 0)}/{c.get('target', '?')} pairs | {c.get('hint', '')}"
        if c.get("coverage_hint"):
            txt += f" | {c['coverage_hint']}"
        return txt, bool(c.get("found_left") and c.get("found_right"))
    calibrated = bool(det.get("calibrated"))
    fps = det.get("server_fps", 0)
    model = det.get("model") or "no detector"
    txt = f"frame {det.get('seq', '?')} | Pi {fps:.1f} fps | model: {model}"
    if not calibrated:
        txt += " | UNCALIBRATED: depth values are NOT trustworthy"
    if det.get("model") and not det.get("model_has_allowed_class"):
        txt += " | model has no allowed (strawberry) class"
    return txt, calibrated

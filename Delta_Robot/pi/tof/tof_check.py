"""
tof_check.py -- headless test of the Arducam ToF depth camera (no GUI needed, works over SSH).

    python tof_check.py                      # tries CSI, then USB
    python tof_check.py --connection usb
    python tof_check.py --range 4000         # then try again with --range 4 (see note below)

What it does
  1. opens the camera, prints what the SDK reports (resolution, device type, intrinsics, range),
  2. grabs a burst of frames and measures the frame rate,
  3. prints the DISTANCE AT THE CENTER PIXEL (median of a small patch, low-confidence pixels ignored),
  4. saves pictures you can copy to your PC (scp) and look at:
        depth_color.png  colourised depth (black = no valid depth)
        confidence.png   the sensor's confidence image
        amplitude.png    the infrared "brightness" image  <- the image YOLO would be trained on (option a)
        depth_mm.npy     raw depth numbers of the last frame

Why it prints so much: several things in the vendor's examples disagree or are unclear, and only the
real camera can answer them. This script reports the evidence:
  * RANGE: one vendor example sets 2000/4000, another sets 2/4. The read-back value printed here shows
    which one this camera accepts. Depth values are millimetres.
  * CONFIDENCE: the vendor example shows a confidence control only for the VGA model; yours is 240x180.
    The percentiles printed here show whether the confidence image carries information.
  * FLAT-WALL TEST: aim the camera straight at a flat wall. If the five "region" distances come out
    about equal, depth is distance ALONG THE OPTICAL AXIS (Z). If the corners are larger than the center,
    depth is distance along each pixel's ray and must be corrected before computing X/Y/Z.
  * MIRRORING: wave your hand from the camera's left to its right while looking at depth_color.png taken
    before/after; or put a small object left of center and check which side of the picture it appears on.
    (Hint: left of the camera should be left of the image. If not, tell the assistant; HFLIP can fix it.)

Needs (inside the venv):  pip install "numpy<2" ArducamDepthCamera "opencv-python-headless<4.12"
"""
import argparse
import os
import sys
import time

import numpy as np

HINTS = {
    "ArducamNoPermission": "Permission denied. For a USB camera this usually needs a udev rule or running as a user in the "
                           "right group (try plugging it in again, then check `lsusb` and `dmesg | tail`). Tell the assistant "
                           "the exact message.",
    "ArducamUnkownDevice": "No supported device found on that connection. Check the cable/ribbon and the connection type.",
    "ArducamSystemError": "System error opening the device. For CSI this usually means the boot overlay for the camera is "
                          "not loaded (see docs/PI_SETUP.md, 'CSI only'), or the ribbon cable is in the wrong port/orientation.",
    "ArducamUnkownError": "Unknown error. For a USB camera check `lsusb` shows the Arducam device and that the 5 V 2 A "
                          "power supply for the camera is connected.",
}


def code_name(code) -> str:
    return getattr(code, "name", str(code))


def ok(code) -> bool:
    return code_name(code) == "ArducamSuccess" or code == 0


def open_camera(ac, which: str):
    """Returns (cam, connection_name) or (None, None). 'auto' tries CSI first, then USB."""
    order = {"csi": ["CSI"], "usb": ["USB"], "auto": ["CSI", "USB"]}[which]
    last = None
    for name in order:
        cam = ac.ArducamCamera()
        ret = cam.open(getattr(ac.Connection, name), 0)
        if ok(ret):
            return cam, name
        last = ret
        print(f"  open({name}) failed: {code_name(ret)}")
        hint = HINTS.get(code_name(ret))
        if hint:
            print(f"     -> {hint}")
    return None, None


def safe_control(cam, ctrl, scale=1.0):
    try:
        return cam.getControl(ctrl) / scale
    except Exception as ex:  # a control may not exist on every model
        return f"unavailable ({ex})"


def patch_median(depth, conf, cy, cx, half, conf_min):
    h, w = depth.shape
    ys, ye = max(0, cy - half), min(h, cy + half + 1)
    xs, xe = max(0, cx - half), min(w, cx + half + 1)
    d, c = depth[ys:ye, xs:xe], conf[ys:ye, xs:xe]
    good = np.isfinite(d) & (d > 0) & (c >= conf_min)
    return (float(np.median(d[good])) if good.any() else None), float(good.mean())


def main(argv=None, ac=None) -> int:
    ap = argparse.ArgumentParser(description="Headless Arducam ToF check")
    ap.add_argument("--connection", choices=["auto", "csi", "usb"], default="auto")
    ap.add_argument("--range", type=int, default=None,
                    help="value passed as-is to the RANGE control (try 4000, then 4)")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--confidence-min", type=float, default=30.0, help="pixels below this confidence are ignored")
    ap.add_argument("--hflip", type=int, choices=[0, 1], default=None)
    ap.add_argument("--vflip", type=int, choices=[0, 1], default=None)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "tof_check_out"))
    a = ap.parse_args(argv)

    if ac is None:
        try:
            import ArducamDepthCamera as ac  # noqa: N813
        except ImportError:
            print('ERROR: ArducamDepthCamera is not installed in this Python.\n'
                  '       Activate the venv and run:  pip install "numpy<2" ArducamDepthCamera')
            return 2
    import cv2

    print("Arducam ToF check.  SDK version:", getattr(ac, "__version__", "?"))
    cam, conn = open_camera(ac, a.connection)
    if cam is None:
        print("\nCould not open the camera. Nothing was changed on the system.")
        return 1
    print(f"Opened via {conn}.")

    try:
        ret = cam.start(ac.FrameType.DEPTH)
        if not ok(ret):
            print("start(DEPTH) failed:", code_name(ret))
            return 1
        if a.range is not None:
            print("setControl(RANGE,", a.range, ") ->", code_name(cam.setControl(ac.Control.RANGE, a.range)))
        if a.hflip is not None:
            cam.setControl(ac.Control.HFLIP, a.hflip)
        if a.vflip is not None:
            cam.setControl(ac.Control.VFLIP, a.vflip)

        info = cam.getCameraInfo()
        print(f"Device type: {getattr(info.device_type, 'name', info.device_type)}   resolution: {info.width}x{info.height}")
        print("RANGE read-back      :", safe_control(cam, ac.Control.RANGE))
        for nm in ("FX", "FY", "CX", "CY"):
            print(f"INTRINSIC_{nm} (/100)  :", safe_control(cam, getattr(ac.Control, "INTRINSIC_" + nm), 100.0))
        print("HFLIP / VFLIP        :", safe_control(cam, ac.Control.HFLIP), "/", safe_control(cam, ac.Control.VFLIP))

        os.makedirs(a.out, exist_ok=True)
        depths, confs, amps, stamps = [], [], [], []
        t_end = time.time() + 20.0
        while len(depths) < a.frames and time.time() < t_end:
            frame = cam.requestFrame(2000)
            if frame is None or not isinstance(frame, ac.DepthData):
                continue
            depths.append(np.array(frame.depth_data, dtype=np.float32))
            confs.append(np.array(frame.confidence_data, dtype=np.float32))
            amp = getattr(frame, "amplitude_data", None)
            amps.append(None if amp is None else np.array(amp, dtype=np.float32))
            stamps.append(time.time())
            cam.releaseFrame(frame)
        if len(depths) < 2:
            print("ERROR: got no frames within 20 s (camera powered? 5 V 2 A supply connected?).")
            return 1
        fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(f"\nGot {len(depths)} frames, about {fps:.1f} frames per second.")

        d, c = depths[-1], confs[-1]
        h, w = d.shape
        print("\nConfidence percentiles (0/10/50/90/100):", np.round(np.percentile(c, [0, 10, 50, 90, 100]), 1))
        print("Depth (mm) percentiles of pixels with depth>0 (0/10/50/90/100):",
              np.round(np.percentile(d[np.isfinite(d) & (d > 0)], [0, 10, 50, 90, 100]), 0) if (d > 0).any() else "none")
        valid = np.isfinite(d) & (d > 0) & (c >= a.confidence_min)
        print(f"Valid pixels (depth>0 and confidence>={a.confidence_min:g}): {100 * valid.mean():.0f}%")

        med, frac = patch_median(d, c, h // 2, w // 2, 4, a.confidence_min)
        if med is None:
            print("\nCENTER PIXEL: no valid depth in the 9x9 patch (too close/far, dark/shiny surface, or confidence too strict).")
        else:
            print(f"\nCENTER PIXEL distance: {med:.0f} mm  ({med / 1000:.3f} m)   [9x9 median, {100 * frac:.0f}% of patch valid]")

        r = 8
        regions = {"center": (h // 2, w // 2), "top-left": (r + 4, r + 4), "top-right": (r + 4, w - r - 5),
                   "bottom-left": (h - r - 5, r + 4), "bottom-right": (h - r - 5, w - r - 5)}
        print("Flat-wall test (all values should be ~equal if depth is along the optical axis):")
        for name, (cy, cx) in regions.items():
            m, _ = patch_median(d, c, cy, cx, r // 2, a.confidence_min)
            print(f"   {name:12s}: {'n/a' if m is None else f'{m:.0f} mm'}")

        # --- pictures ---
        rng = safe_control(cam, ac.Control.RANGE)
        rng = float(rng) if isinstance(rng, (int, float)) and rng > 0 else 4000.0
        if rng < 100:      # read-back in metres (2 or 4) -> convert to mm for scaling
            rng *= 1000.0
        col = cv2.applyColorMap(np.clip(np.nan_to_num(d) * (255.0 / rng), 0, 255).astype(np.uint8), cv2.COLORMAP_RAINBOW)
        col[~valid] = 0
        cv2.imwrite(os.path.join(a.out, "depth_color.png"), col)
        cv2.imwrite(os.path.join(a.out, "confidence.png"), cv2.normalize(c, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8))
        if amps[-1] is not None:
            cv2.imwrite(os.path.join(a.out, "amplitude.png"),
                        cv2.normalize(amps[-1], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8))
        np.save(os.path.join(a.out, "depth_mm.npy"), d)
        print(f"\nSaved pictures to {os.path.abspath(a.out)}/  (copy them with scp and look at them)")
        return 0
    finally:
        try:
            cam.stop()
            cam.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

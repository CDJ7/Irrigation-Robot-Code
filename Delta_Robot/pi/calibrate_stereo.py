"""
calibrate_stereo.py -- stereo camera calibration, designed for SSH (no GUI windows).
==============================================================================
Why calibrate? Depth from two cameras needs to know (a) each lens's focal length and
distortion, (b) how far apart the cameras are and exactly how they are rotated relative
to each other. Guessing these (as the old code did) gives wrong millimeters.

STEP 0  Print the board:      python make_checkerboard.py      (see that file)
        Glue it FLAT on a rigid board (foam board, plywood). Matte paper, no glare.
        Measure one square with a ruler -> that number is --square-mm.

STEP 1  Stop the vision server (it owns the camera):  sudo systemctl stop delta-vision
        (or Ctrl+C if you started it by hand).

STEP 2  Capture pairs:
            python calibrate_stereo.py capture --square-mm 25
        Hold the board in view of BOTH eyes and move it slowly. The script saves a pair
        whenever the board is found in both eyes AND has moved enough since the last
        save, and prints where you still need coverage. Aim for 25+ pairs:
          * fill the image: board in the corners and edges, not only the middle
          * vary distance: some near (board fills ~2/3 of the picture) and some far
          * vary tilt: rotate the board +-30 degrees about both axes (not just flat)
          * hold still while it saves (motion blur ruins corners)
          * cover the distance range where strawberries will actually be
        Over SSH you cannot see the picture, so the script writes calib_preview.jpg with
        the detected corners drawn on it; copy it to your PC to look:
            scp deltarobot@deltapi.local:Delta_Robot/pi/calib_preview.jpg .

STEP 3  Solve:
            python calibrate_stereo.py solve --square-mm 25
        Prints the quality numbers and writes calibration/stereo_calib.npz, which
        pi_camera_server.py loads automatically. Good result: stereo RMS < ~0.5 px and
        mean row error after rectification < ~0.3 px.

        Rectification check image:  calibration/rectify_check.jpg  (horizontal lines should
        pass through the same features in both eyes).

Pattern size is INNER corners (default 9x6). Units: millimeters everywhere.
"""

import argparse
import glob
import os
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pi_camera_server import V4L2Camera, load_config, split_eyes   # noqa: E402

SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)


def parse_pattern(s: str) -> Tuple[int, int]:
    c, r = s.lower().split("x")
    return int(c), int(r)


def find_corners(gray: np.ndarray, pattern: Tuple[int, int], fast: bool = False) -> Optional[np.ndarray]:
    """Returns refined corners as a float32 array of shape (N,1,2), or None.
    `fast` is for the live capture loop. (Shape is normalized because different OpenCV
    versions return slightly different array shapes.)"""
    if not fast and hasattr(cv2, "findChessboardCornersSB"):
        # The "SB" detector is more robust on small, blurry images -- try it first.
        ok, c = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return np.asarray(c, np.float32).reshape(-1, 1, 2)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast:
        flags |= cv2.CALIB_CB_FAST_CHECK
    ok, c = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    c = np.asarray(c, np.float32).reshape(-1, 1, 2)
    return np.asarray(cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), SUBPIX), np.float32).reshape(-1, 1, 2)


def object_points(pattern: Tuple[int, int], square_mm: float) -> np.ndarray:
    """3D corner positions in the board's own frame (Z=0 plane), in mm."""
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_mm
    return objp


def pair_is_consistent(cl: np.ndarray, cr: np.ndarray) -> bool:
    """Both eyes must label corners in the same order. For a camera pair with the left eye on
    the left, a point appears further RIGHT in the left image than in the right image, so
    the mean (uL - uR) must be positive. A flipped board would give the wrong sign."""
    return float(np.mean(cl[:, 0, 0] - cr[:, 0, 0])) > 0.0


# =========================================================
#   CAPTURE
# =========================================================
def capture(args, camera=None) -> int:
    pattern = parse_pattern(args.board)
    os.makedirs(args.dir, exist_ok=True)
    cfg = load_config(args.config)
    own_camera = camera is None
    if own_camera:
        camera = V4L2Camera(cfg["camera"])
        if not camera.open():
            print("ERROR: cannot open the camera. Is pi_camera_server.py (or main_harvester.py) "
                  "still running? Stop it first:  fuser -v /dev/video0")
            return 1
    existing = len(glob.glob(os.path.join(args.dir, "pair_*_L.png")))
    n = existing
    print(f"Board {pattern[0]}x{pattern[1]} inner corners. Saving pairs to {args.dir}/ "
          f"(already there: {existing}). Target: {args.count}. Ctrl+C to finish early.")
    last_saved_corners = None
    last_save_t = 0.0
    last_preview_t = 0.0
    last_msg_t = 0.0
    coverage = np.zeros((3, 3), int)
    sizes = []
    fails = 0
    try:
        while n < args.count:
            frame = camera.read()
            if frame is None:
                fails += 1
                if fails > 100:
                    print("ERROR: camera gives no frames.")
                    return 1
                time.sleep(0.05)
                continue
            fails = 0
            left, right = split_eyes(frame)
            h, w = left.shape
            cl = find_corners(left, pattern, fast=True)
            cr = find_corners(right, pattern, fast=True) if cl is not None else None
            now = time.time()
            both = cl is not None and cr is not None and pair_is_consistent(cl, cr)

            if now - last_preview_t > 1.0:      # preview for scp
                last_preview_t = now
                vis = np.hstack([cv2.cvtColor(left, cv2.COLOR_GRAY2BGR), cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)])
                if cl is not None:
                    cv2.drawChessboardCorners(vis[:, :w], pattern, cl, True)
                if cr is not None:
                    cv2.drawChessboardCorners(vis[:, w:], pattern, cr, True)
                cv2.imwrite(os.path.join(HERE, "calib_preview.jpg"), vis)

            if not both:
                if now - last_msg_t > 3.0:
                    last_msg_t = now
                    print(f"  searching... left={'found' if cl is not None else 'no'} "
                          f"right={'found' if cr is not None else 'no'}  (board fully visible in BOTH eyes? "
                          f"evenly lit? flat?)")
                continue

            moved = True
            if last_saved_corners is not None:
                moved = float(np.mean(np.linalg.norm(cl - last_saved_corners, axis=2))) > args.min_move
            if not moved or now - last_save_t < args.interval:
                continue

            cv2.imwrite(os.path.join(args.dir, f"pair_{n:03d}_L.png"), left)
            cv2.imwrite(os.path.join(args.dir, f"pair_{n:03d}_R.png"), right)
            n += 1
            last_saved_corners, last_save_t = cl.copy(), now
            cx, cy = cl[:, 0, 0].mean(), cl[:, 0, 1].mean()
            coverage[min(2, int(cy / h * 3)), min(2, int(cx / w * 3))] += 1
            sizes.append(float(np.ptp(cl[:, 0, 0])))
            empty = [(["top", "middle", "bottom"][r], ["left", "center", "right"][c])
                     for r in range(3) for c in range(3) if coverage[r, c] == 0]
            hint = ("still empty: " + ", ".join(f"{a}-{b}" for a, b in empty[:4])) if empty else "all regions covered"
            print(f"  saved pair {n}/{args.count}  (board width {sizes[-1]:.0f}px; {hint})")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if own_camera:
            camera.close()
    print(f"Done: {n} pairs in {args.dir}/. Next:  python calibrate_stereo.py solve --square-mm {args.square_mm:g}")
    return 0


# =========================================================
#   SOLVE
# =========================================================
def load_pairs(directory: str, pattern, quiet=False):
    names = sorted(glob.glob(os.path.join(directory, "pair_*_L.png")))
    if not names:
        raise SystemExit(f"No pair_*_L.png images in {directory}/ -- run the capture step first.")
    pairs = []
    size = None
    for lp in names:
        rp = lp.replace("_L.png", "_R.png")
        gl, gr = cv2.imread(lp, cv2.IMREAD_GRAYSCALE), cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
        if gl is None or gr is None:
            continue
        if size is None:
            size = (gl.shape[1], gl.shape[0])
        if (gl.shape[1], gl.shape[0]) != size or gr.shape != gl.shape:
            print(f"  skip {os.path.basename(lp)}: size differs from the others")
            continue
        cl, cr = find_corners(gl, pattern), find_corners(gr, pattern)
        if cl is None or cr is None:
            print(f"  skip {os.path.basename(lp)}: board not found in {'left' if cl is None else 'right'}")
            continue
        if not pair_is_consistent(cl, cr):
            print(f"  skip {os.path.basename(lp)}: corner order differs between eyes")
            continue
        pairs.append((os.path.basename(lp), gl, gr, cl, cr))
    return pairs, size


def calibrate_once(objp, ptsL, ptsR, size, alpha):
    """Intrinsics per camera first, then the relative pose (R, T) with intrinsics held fixed.
    Fixing them is the robust choice with a small, low-resolution data set."""
    obj = [objp] * len(ptsL)
    flags = cv2.CALIB_FIX_K3                      # 3rd radial term overfits small images
    rmsL, K1, D1, *_ = cv2.calibrateCamera(obj, ptsL, size, None, None, flags=flags)
    rmsR, K2, D2, *_ = cv2.calibrateCamera(obj, ptsR, size, None, None, flags=flags)
    crit = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 200, 1e-7)
    rms, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj, ptsL, ptsR, K1, D1, K2, D2, size, flags=cv2.CALIB_FIX_INTRINSIC, criteria=crit)[:9]
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha)
    return dict(rmsL=rmsL, rmsR=rmsR, rms=rms, K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T,
                R1=R1, R2=R2, P1=P1, P2=P2, Q=Q)


def row_errors(cal, ptsL, ptsR):
    """For each image: mean |y_left - y_right| (pixels) after rectification. If the geometry
    is right, matching points sit on the same row, so this should be tiny."""
    errs = []
    for cl, cr in zip(ptsL, ptsR):
        ul = cv2.undistortPoints(cl, cal["K1"], cal["D1"], R=cal["R1"], P=cal["P1"]).reshape(-1, 2)
        ur = cv2.undistortPoints(cr, cal["K2"], cal["D2"], R=cal["R2"], P=cal["P2"]).reshape(-1, 2)
        errs.append(float(np.mean(np.abs(ul[:, 1] - ur[:, 1]))))
    return np.array(errs)


def solve(args) -> int:
    pattern = parse_pattern(args.board)
    pairs, size = load_pairs(args.dir, pattern)
    print(f"{len(pairs)} usable pairs, image size {size}")
    if len(pairs) < 10:
        print("ERROR: need at least 10 usable pairs (20+ recommended). Capture more.")
        return 1
    objp = object_points(pattern, args.square_mm)

    # Fit, drop clearly bad pairs (blurry / mis-detected), refit. Up to 3 rounds.
    for rnd in range(3):
        ptsL, ptsR = [p[3] for p in pairs], [p[4] for p in pairs]
        cal = calibrate_once(objp, ptsL, ptsR, size, args.alpha)
        errs = row_errors(cal, ptsL, ptsR)
        thr = max(2.5 * float(np.median(errs)), 0.4)
        bad = [i for i, e in enumerate(errs) if e > thr]
        if not bad or len(pairs) - len(bad) < 10:
            break
        for i in sorted(bad, reverse=True):
            print(f"  dropping {pairs[i][0]} (row error {errs[i]:.2f}px > {thr:.2f})")
            pairs.pop(i)

    P1, P2 = cal["P1"], cal["P2"]
    f = float(P1[0, 0])
    baseline = float(-P2[0, 3] / P2[0, 0])
    cfg = load_config(args.config)
    nd, md = int(cfg["stereo"]["num_disparities"]), int(cfg["stereo"]["min_disparity"])

    lines = [
        f"Stereo calibration  ({len(pairs)} pairs, {size[0]}x{size[1]} per eye, "
        f"board {pattern[0]}x{pattern[1]} @ {args.square_mm:g} mm)",
        f"  left  camera RMS reprojection error : {cal['rmsL']:.3f} px",
        f"  right camera RMS reprojection error : {cal['rmsR']:.3f} px",
        f"  stereo RMS reprojection error       : {cal['rms']:.3f} px   (good: < ~0.5)",
        f"  mean row error after rectification : {float(np.mean(row_errors(cal, [p[3] for p in pairs], [p[4] for p in pairs]))):.3f} px   (good: < ~0.3)",
        f"  focal length (rectified)            : {f:.1f} px",
        f"  principal point (rectified, left)   : ({P1[0, 2]:.1f}, {P1[1, 2]:.1f})",
        f"  BASELINE                            : {baseline:.2f} mm   <- compare with a ruler!",
        f"  rotation between eyes               : {np.degrees(np.linalg.norm(cv2.Rodrigues(cal['R'])[0])):.2f} deg",
        "",
        f"With num_disparities={nd}, min_disparity={md} (config.yaml):",
        f"  nearest measurable distance : {f * baseline / (nd + md):.0f} mm",
        "  depth resolution (change in Z for 1/16 px of disparity, the matcher's step):",
    ]
    for z in (200, 300, 400, 600, 800, 1000):
        lines.append(f"    at Z={z:4d} mm : {z * z / (f * baseline) / 16.0:.2f} mm")
    if cal["rms"] > 1.0:
        lines.append("\nWARNING: RMS is high. Re-capture with sharper images / flatter board / more "
                     "varied poses, and double-check --square-mm.")
    report = "\n".join(lines)
    print("\n" + report)

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez(out, image_size=np.array(size), K1=cal["K1"], D1=cal["D1"], K2=cal["K2"], D2=cal["D2"],
             R=cal["R"], T=cal["T"], R1=cal["R1"], R2=cal["R2"], P1=P1, P2=P2, Q=cal["Q"],
             rms=cal["rms"], square_mm=args.square_mm, n_pairs=len(pairs),
             board=np.array(pattern), created=time.time())
    with open(os.path.join(os.path.dirname(out), "stereo_calib_report.txt"), "w") as fh:
        fh.write(report + "\n")

    # Visual check: rectified pair with horizontal lines.
    name, gl, gr, *_ = pairs[len(pairs) // 2]
    m1 = cv2.initUndistortRectifyMap(cal["K1"], cal["D1"], cal["R1"], P1, size, cv2.CV_16SC2)
    m2 = cv2.initUndistortRectifyMap(cal["K2"], cal["D2"], cal["R2"], P2, size, cv2.CV_16SC2)
    rl, rr = cv2.remap(gl, *m1, cv2.INTER_LINEAR), cv2.remap(gr, *m2, cv2.INTER_LINEAR)
    vis = cv2.cvtColor(np.hstack([rl, rr]), cv2.COLOR_GRAY2BGR)
    for y in range(0, vis.shape[0], 20):
        cv2.line(vis, (0, y), (vis.shape[1], y), (0, 255, 0), 1)
    cv2.imwrite(os.path.join(os.path.dirname(out), "rectify_check.jpg"), vis)
    print(f"\nSaved {out}\n      + stereo_calib_report.txt, rectify_check.jpg in the same folder.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Stereo calibration (SSH friendly)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("capture", "solve"):
        p = sub.add_parser(name)
        p.add_argument("--board", default="9x6", help="inner corners, e.g. 9x6")
        p.add_argument("--square-mm", type=float, required=True, help="MEASURED square size in mm")
        p.add_argument("--dir", default=os.path.join(HERE, "calibration", "images"))
        p.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    c = sub.choices["capture"]
    c.add_argument("--count", type=int, default=30)
    c.add_argument("--interval", type=float, default=1.5, help="min seconds between saves")
    c.add_argument("--min-move", type=float, default=12.0, help="min mean corner motion (px) between saves")
    s = sub.choices["solve"]
    s.add_argument("--out", default=os.path.join(HERE, "calibration", "stereo_calib.npz"))
    s.add_argument("--alpha", type=float, default=0.0,
                   help="0 = crop to valid pixels only, 1 = keep everything (black borders)")
    a = ap.parse_args()
    return capture(a) if a.cmd == "capture" else solve(a)


if __name__ == "__main__":
    sys.exit(main())

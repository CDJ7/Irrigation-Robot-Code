"""
Tests the calibration logic (delta_gui/core/calibration.py) and checks the home calibration against the real controller + simulated ESP32.
Run:  python tests/test_core_calibration.py      (needs numpy)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from delta_gui.core.calibration import (CameraCalibrationSession, ProbeCollector, home_calibration, invert_from_answer,   # noqa: E402
                                        nudge_steps, suggest_points)
from delta_gui.core.kinematics import DeltaKinematics, Workspace                                                         # noqa: E402


def test_home():
    k = DeltaKinematics()
    true_angles = (-22.0, -22.0, -22.0)                                  # arms raised 22 deg at the limit switches
    res = home_calibration(true_angles, k, -30, 80, Workspace())
    assert res.offsets_deg == [22.0, 22.0, 22.0]
    assert k.inverse(*res.home_xyz) is not None and all(abs(u - v) < 1e-6 for u, v in zip(k.inverse(*res.home_xyz), true_angles)), \
        "HOME XYZ must map back to the measured angles"
    assert abs(res.home_xyz[0]) < 1e-6 and abs(res.home_xyz[1]) < 1e-6 and res.home_xyz[2] > -200
    assert any("outside the workspace box" in w for w in res.warnings)
    # slightly different arms -> a warning, still a result
    r2 = home_calibration((-22.0, -20.5, -24.0), k, -30, 80)
    assert any("differ" in w for w in r2.warnings)
    for bad, word in (((-22, -22), "all three"), ((-50, -22, -22), "allowed range"), ((float("nan"), 0, 0), "all three")):
        try:
            home_calibration(bad, k, -30, 80); raise AssertionError(bad)
        except ValueError as ex:
            assert word in str(ex), (bad, ex)
    try:                                                                  # three angles that cannot meet at one point
        home_calibration((-30, -30, -30), DeltaKinematics(bicep=150, forearm=120), -30, 80); raise AssertionError("must refuse")
    except ValueError as ex:
        assert "cannot belong to one pose" in str(ex)
    assert nudge_steps(3.0, 417.7778) == 1253 and invert_from_answer(True) is False and invert_from_answer(False) is True
    print("home calibration OK (offsets, FK home XYZ, round trip, warnings, refusals, direction answer)")


def test_home_with_controller():
    """After the wizard values are applied, the controller must consider step 0 consistent with HOME and a move to HOME must be a no-op."""
    from test_core_robot import Rig
    r = Rig(); r.connect_enable_home()
    assert r.robot.home_consistency() is not None, "defaults are inconsistent (that is why the wizard exists)"
    res = home_calibration((-21.0, -21.0, -21.0), r.robot.kin, -30, 80, r.robot.workspace)
    r.settings.data["axes"]["offsets_deg"] = res.offsets_deg
    r.settings.data["home"].update(xyz_mm=list(res.home_xyz), calibrated=True)
    r.robot.apply_settings()
    assert r.robot.home_consistency() is None
    assert r.robot._steps_for(r.robot.kin.inverse(*r.robot.home_xyz)) == [0, 0, 0] or max(abs(v) for v in r.robot._steps_for(r.robot.kin.inverse(*r.robot.home_xyz))) <= 1
    r.logs.clear()                                   # (the FIRST homing, before calibration, did warn)
    assert r.robot.home() and r.wait(lambda: r.robot.xyz_valid and not r.robot.busy)
    assert not r.said("WARNING"), "no home-mismatch warning after calibration"
    print("home calibration + controller OK (consistent, home XYZ == step 0, no warning at homing)")


def test_camera_session():
    rng = np.random.default_rng(5)
    th = np.radians(28)
    R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]]) @ np.diag([1, -1, -1])
    t = np.array([15.0, -25.0, 760.0])
    ws = Workspace()
    ok_everywhere = lambda x, y, z: None
    pts = suggest_points(ok_everywhere, ws)
    assert len(pts) == 8 and all(ws.contains(*p) for p in pts)
    spread = np.ptp(np.array(pts), axis=0)
    assert spread[0] > 100 and spread[1] > 100 and spread[2] > 60, spread
    # ordered for short travel: total path shorter than the worst-case ordering
    path = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    assert path < 8 * math.dist(pts[0], pts[-1]) * 1.5
    # the suggestion respects 'allowed' rules (here: only a sub-box is allowed)
    tight = suggest_points(lambda x, y, z: None if abs(x) < 40 and abs(y) < 40 else "no", ws)
    assert tight and all(abs(p[0]) < 40 and abs(p[1]) < 40 for p in tight)

    s = CameraCalibrationSession()
    for p in pts:
        cam = R.T @ (np.array(p) - t) + rng.normal(0, 0.6, 3)             # robot = R cam + t  ->  cam = R^T (robot - t)
        s.add(p, cam, 8, 0.5)
    rep = s.solve()
    assert rep.rms_mm < 1.5 and np.allclose(rep.transform.R, R, atol=0.01) and not rep.warnings, (rep.rms_mm, rep.warnings)
    # a bad point is pointed out
    s.points[3].cam_xyz = (s.points[3].cam_xyz[0] + 40, s.points[3].cam_xyz[1], s.points[3].cam_xyz[2])
    rep = s.solve()
    assert any("point 4" in w for w in rep.warnings), rep.warnings
    s.remove(3)
    assert not any("point" in w for w in s.solve().warnings) and len(s.points) == 7
    # too few points / coplanar-and-clustered points
    s2 = CameraCalibrationSession()
    for p in pts[:3]:
        s2.add(p, R.T @ (np.array(p) - t))
    try:
        s2.solve(); raise AssertionError("3 points must be refused")
    except ValueError as ex:
        assert "at least 4" in str(ex)
    s3 = CameraCalibrationSession()
    for i in range(5):                                                    # a small flat ring: clustered, coplanar, NOT collinear
        q = (15 * math.cos(2 * math.pi * i / 5), 15 * math.sin(2 * math.pi * i / 5), -280.0)
        s3.add(q, R.T @ (np.array(q) - t))
    w3 = " ".join(s3.solve().warnings)
    assert "spread" in w3 and "same height" in w3, w3
    # a MIRRORED camera frame is diagnosed in plain words
    s4 = CameraCalibrationSession()
    for p in pts:
        c = R.T @ (np.array(p) - t); s4.add(p, (-c[0], c[1], c[2]))
    w4 = s4.solve().warnings
    assert any("MIRRORED" in w for w in w4), w4
    print("camera calibration session OK (suggested points, solve, outlier named, refusals, mirrored frame diagnosed)")


def test_probe_collector():
    pc = ProbeCollector(n=5, max_spread_mm=4.0)
    for v in (700.0, 701.0, 699.5, 700.4, 700.1):
        pc.feed((10.0, 20.0, v))
    pc.feed(None); pc.feed((float("nan"), 0, 0))
    assert pc.done and pc.acceptable()[0]
    med, spread = pc.result(); assert abs(med[2] - 700.1) < 1e-9 and spread < 2
    noisy = ProbeCollector(n=3, max_spread_mm=4.0)
    for v in (700.0, 720.0, 680.0):
        noisy.feed((0, 0, v))
    ok, why = noisy.acceptable(); assert not ok and "jump around" in why
    try:
        ProbeCollector().result(); raise AssertionError
    except ValueError as ex:
        assert "no valid camera readings" in str(ex)
    print("probe collector OK (median, spread rejection, invalid readings ignored)")


if __name__ == "__main__":
    test_home(); test_home_with_controller(); test_camera_session(); test_probe_collector()
    print("\nALL TESTS PASSED")

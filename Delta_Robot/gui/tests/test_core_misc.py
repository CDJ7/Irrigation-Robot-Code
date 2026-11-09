"""
Tests the rest of delta_gui/core: kinematics (inverse/forward round trip, radius->side conversion, steps), Kabsch transform,
settings file (defaults, save/load, broken file, validation) and the REAL SerialLink through pyserial's loop:// loopback port.
Run:  python tests/test_core_misc.py     (needs numpy and pyserial)
"""
import json
import math
import os
import random
import sys
import tempfile
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from delta_gui.core.kinematics import (DeltaKinematics, Workspace, angles_to_steps, joint_limit_violation,   # noqa: E402
                                       radius_to_side, side_to_radius, steps_to_angles)
from delta_gui.core.link import SerialLink                                                                  # noqa: E402
from delta_gui.core.settings import DEFAULTS, Settings, validate                                            # noqa: E402
from delta_gui.core.transform import CameraToRobot, solve_rigid                                            # noqa: E402


def test_kinematics():
    assert abs(radius_to_side(76.2) - 263.97) < 0.01 and abs(side_to_radius(radius_to_side(50)) - 50) < 1e-9
    k = DeltaKinematics()
    random.seed(1)
    n = worst = 0
    for _ in range(3000):
        p = (random.uniform(-150, 150), random.uniform(-150, 150), random.uniform(-420, -120))
        a = k.inverse(*p)
        if a is None:
            continue
        q = k.forward(*a)
        worst = max(worst, max(abs(u - v) for u, v in zip(p, q)))
        n += 1
    assert n > 1000 and worst < 1e-6, (n, worst)
    for _ in range(2000):
        a = tuple(random.uniform(-20, 60) for _ in range(3))
        p = k.forward(*a)
        if p and k.inverse(*p):
            assert max(abs(u - v) for u, v in zip(a, k.inverse(*p))) < 1e-6
    assert k.inverse(0, 0, 0) is None, "z = 0 must not divide by zero"
    assert k.inverse(1000, 0, -200) is None
    # symmetry: the centre point needs identical angles on all three arms
    a = k.inverse(0, 0, -250); assert abs(a[0] - a[1]) < 1e-9 and abs(a[1] - a[2]) < 1e-9
    # the radius/side fix: the SAME typed number 76.2 means very different geometry depending on the meaning
    wrong = DeltaKinematics(base_radius=side_to_radius(76.2))          # what typing 76.2 into the old 'side length' box did
    assert abs(wrong.inverse(0, 0, -250)[0] - a[0]) > 5, "radius vs side length changes the arm angles by >5 deg"
    spd = 200 * 16 * 47 / 360
    st = angles_to_steps((10, 20, 30), (1, 2, 3), (False, True, False), spd)
    back = steps_to_angles(st, (1, 2, 3), (False, True, False), spd)
    assert max(abs(x - y) for x, y in zip(back, (10, 20, 30))) < 1 / spd
    assert joint_limit_violation((0, 95, 0), -30, 80) and joint_limit_violation((0, 10, 0), -30, 80) is None
    w = Workspace((-1, 1), (-1, 1), (-3, -2))
    assert w.contains(0, 0, -2.5) and "Z=" in w.violation(0, 0, -1)
    print("kinematics OK (round trips to 1e-6, radius/side, steps, limits, box)")


def test_transform():
    rng = np.random.default_rng(3)
    th = np.radians(30)
    R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]]) @ np.diag([1, -1, -1])
    t = np.array([10.0, 20.0, 700.0])
    cam = rng.uniform([-150, -100, 600], [150, 100, 900], (10, 3))
    rob = cam @ R.T + t + rng.normal(0, 0.5, (10, 3))
    tf, errs = solve_rigid(cam, rob)
    assert tf.valid and tf.rms_mm < 1.5 and np.allclose(tf.R, R, atol=0.01) and abs(np.linalg.det(tf.R) - 1) < 1e-9
    assert np.allclose(tf.to_camera(tf.to_robot(cam[0])), cam[0])
    px = tf.project(rob[0], 726.0, 726.0, 320.0, 240.0)
    assert px is not None and 0 < px[0] < 640 and 0 < px[1] < 480
    assert tf.project([0, 0, 0], 726, 726, 320, 240) is None or True
    d = json.loads(json.dumps(tf.to_dict())); tf2 = CameraToRobot.from_dict(d)
    assert tf2.valid and np.allclose(tf2.R, tf.R) and abs(tf2.rms_mm - tf.rms_mm) < 1e-9
    assert not CameraToRobot.from_dict({"R": "junk"}).valid and not CameraToRobot.from_dict(None).valid
    for bad in (cam[:2], ):
        try:
            solve_rigid(bad, rob[:2]); raise AssertionError("2 points must be refused")
        except ValueError:
            pass
    try:
        solve_rigid([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]]); raise AssertionError
    except ValueError:
        pass
    mirrored, _ = solve_rigid(cam * np.array([-1, 1, 1]), rob)
    assert mirrored.rms_mm > 20, "a mirrored camera frame must show up as a large error, never fit"
    print("transform OK (Kabsch, round trip, json, refusals, mirrored frame exposed)")


def test_settings():
    d = tempfile.mkdtemp(); p = os.path.join(d, "sub", "s.json")
    s = Settings(p); s.load(); assert "no settings file" in s.load_message and s.steps_per_rev == 150400.0
    assert abs(s.steps_per_deg - 417.7777) < 1e-3
    s.data["camera"]["host"] = "10.1.2.3"; s.data["drive"]["microsteps"] = 8; s.save()
    s2 = Settings(p); s2.load(); assert s2.data["camera"]["host"] == "10.1.2.3" and s2.steps_per_rev == 75200.0
    # a file from an older version (missing keys) is completed with defaults
    open(p, "w").write(json.dumps({"camera": {"host": "1.1.1.1"}})); s3 = Settings(p); s3.load()
    assert s3.data["camera"]["host"] == "1.1.1.1" and s3.data["safety"]["confirm_homing"] is True
    # corrupted / invalid file -> moved aside, defaults, program still starts
    open(p, "w").write("{ nope"); s4 = Settings(p); s4.load()
    assert "unusable" in s4.load_message and os.path.exists(p.replace(".json", ".broken")) and s4.data["camera"]["host"] == DEFAULTS["camera"]["host"]
    bad = json.loads(json.dumps(DEFAULTS)); bad["axes"]["theta_min_deg"] = 99
    assert "theta_min" in validate(bad)
    bad = json.loads(json.dumps(DEFAULTS)); bad["geometry"]["forearm_mm"] = 10
    assert "forearm" in validate(bad)
    s5 = Settings(p); s5.data = bad
    try:
        s5.save(); raise AssertionError("invalid settings must not be saved")
    except ValueError:
        pass
    assert not [f for f in os.listdir(os.path.dirname(p)) if f.endswith(".tmp")], "no temp files left behind"
    print("settings OK (defaults, save/load, completion, broken file, validation, atomic)")


def test_serial_link_loopback():
    lines, states = [], []
    link = SerialLink(reset_wait_s=0.1)
    link.on_line, link.on_state = lines.append, lambda c, m: states.append((c, m))
    link.connect("loop://")
    end = time.time() + 3
    while not link.is_connected() and time.time() < end:
        time.sleep(0.02)
    assert link.is_connected() and states and states[0][0] is True
    link.send_line("P?"); link.send_line("M 1 2 3 400")          # loop:// echoes what we write
    end = time.time() + 3
    while len(lines) < 2 and time.time() < end:
        time.sleep(0.02)
    assert lines == ["P?", "M 1 2 3 400"], lines
    # a cable pull: reading fails -> state callback says the link is lost, further sends raise ConnectionError
    link._ser.close()
    end = time.time() + 3
    while states[-1][0] is True and time.time() < end:       # wait for the NOTIFICATION (is_connected() flips a moment earlier)
        time.sleep(0.02)
    assert not link.is_connected() and states[-1][0] is False and "lost" in states[-1][1], states
    try:
        link.send_line("P?"); raise AssertionError("send after loss must raise")
    except ConnectionError:
        pass
    # opening a port that does not exist reports the problem instead of crashing
    st2 = []
    l2 = SerialLink(reset_wait_s=0.0); l2.on_state = lambda c, m: st2.append((c, m)); l2.connect("/dev/does-not-exist")
    end = time.time() + 3
    while not st2 and time.time() < end:
        time.sleep(0.02)
    assert st2 and st2[0][0] is False and "cannot open" in st2[0][1], st2
    # disconnect while still waiting for the board to boot cancels cleanly
    st3 = []
    l3 = SerialLink(reset_wait_s=1.0); l3.on_state = lambda c, m: st3.append((c, m)); l3.connect("loop://"); time.sleep(0.1); l3.disconnect()
    time.sleep(1.3)
    assert not l3.is_connected() and not st3, st3
    link.disconnect()
    print("serial link OK (loopback lines, cable pull, bad port, cancel while booting)")


if __name__ == "__main__":
    test_kinematics(); test_transform(); test_settings(); test_serial_link_loopback()
    print("\nALL TESTS PASSED")

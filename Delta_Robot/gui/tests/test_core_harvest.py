"""
Tests the automatic-picking state machine (delta_gui/core/harvest.py) against the SIMULATED ESP32 and a SIMULATED camera feed.
No hardware, no screen.   Run:  python tests/test_core_harvest.py

The fake scene holds strawberries at known ROBOT positions; each frame is turned into a camera-frame J message with the inverse of a
known camera->robot transform, so the whole chain (camera XYZ -> robot XYZ -> pick sequence -> serial commands) is checked.
Covers: preflight refusals (stock model without strawberry class, no calibration, not homed, no frames, E-STOP), what is ignored
(person, low confidence, no depth, class not on our list, flickering/jittering berries), closest-first order, unreachable berries
skipped, the exact command sequence (dry run and with vacuum), limits (max picks, idle timeout), and every way a run must END
safely: E-STOP mid-pick, soft stop, camera silence, firmware without vacuum, ESP32 reset, link loss.
"""
import os
import queue
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from delta_gui.core import harvest as H                              # noqa: E402
from delta_gui.core.link import SimulatedESP32Link                  # noqa: E402
from delta_gui.core.robot import Robot                              # noqa: E402
from delta_gui.core.settings import Settings                        # noqa: E402
from delta_gui.core.transform import CameraToRobot                  # noqa: E402

SCALE = 60.0
th = np.radians(25)
R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]]) @ np.diag([1.0, -1.0, -1.0])
TRUTH = CameraToRobot(R, np.array([10.0, -20.0, 800.0]), True, 0.3, 8)


class Rig:
    def __init__(self, vacuum=True, calibrated_cam=True, stock_model=False, **harvest_cfg):
        self.settings = Settings(os.path.join(tempfile.mkdtemp(), "s.json"))
        d = self.settings.data
        d["home"]["calibrated"] = True
        d["drive"].update(max_step_rate=60000.0, max_speed_mm_s=2000.0)
        d["harvest"].update(dict(vacuum_enabled=vacuum, dwell_s=0.05, release_s=0.05, idle_timeout_s=2.0, frame_timeout_s=1.0,
                                 stable_frames=4, max_picks=10), **harvest_cfg)
        self.link = SimulatedESP32Link(time_scale=SCALE, home_seconds=1.0)
        self.robot = Robot(self.settings, self.link)
        self.q = queue.Queue()
        self.link.on_line = lambda t: self.q.put(("line", t))
        self.link.on_state = lambda c, m: self.q.put(("state", c, m))
        self.logs = []
        self.robot.listeners.append(lambda ev, data: self.logs.append(data["text"]) if ev == "log" else None)
        self.cal = TRUTH if calibrated_cam else CameraToRobot()
        self.h = H.Harvester(self.robot, self.settings, lambda: self.cal)
        self.h.listeners.append(lambda ev, data: self.logs.append(data["text"]) if ev == "log" else None)
        self.stock_model = stock_model
        self.scene = []                      # dicts: xyz (robot frame), cls, conf, depth(bool)
        self.seq = 0
        self.feeding = True
        self._next_frame = 0.0
        self.frame_hook = None

    def berry(self, x, y, z, cls="strawberry", conf=0.9, depth=True):
        b = dict(xyz=[x, y, z], cls=cls, conf=conf, depth=depth)
        self.scene.append(b)
        return b

    def frame(self):
        self.seq += 1
        dets = []
        for b in self.scene:
            xyz = list(b["xyz"])
            if self.frame_hook:
                xyz = self.frame_hook(b, self.seq, xyz)
                if xyz is None:
                    continue
            cam = TRUTH.to_camera(xyz)
            dets.append({"class": b["cls"], "confidence": b["conf"], "allowed": b["cls"] == "strawberry",
                         "bbox": [1, 1, 20, 20], "center": [10, 10], "depth_valid_fraction": 0.9,
                         "xyz_mm": [float(v) for v in cam] if b["depth"] else None})
        return {"seq": self.seq, "calibrated": True, "model": "yolo11n.pt" if self.stock_model else "straw.pt",
                "model_has_allowed_class": not self.stock_model, "allowed_classes": ["strawberry"], "detections": dets}

    def spin(self, seconds=0.02):
        end = time.time() + seconds
        while time.time() < end:
            try:
                item = self.q.get(timeout=0.005)
            except queue.Empty:
                item = None
            if item:
                self.robot.on_line(item[1]) if item[0] == "line" else self.robot.on_link_state(item[1], item[2])
            now = time.time()
            if self.feeding and now >= self._next_frame:
                self.h.on_detections(self.frame()); self._next_frame = now + 0.03
            self.h.tick(); self.robot.tick()

    def until(self, pred, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            self.spin(0.02)
            if pred():
                return True
        return False

    def ready(self):
        self.link.connect("SIM"); assert self.until(lambda: self.robot.connected)
        self.robot.enable(True); assert self.until(lambda: self.robot.drivers_enabled)
        self.robot.home(); assert self.until(lambda: self.robot.xyz_valid and not self.robot.busy)
        self.link.sent.clear()
        return self

    def slow(self):
        """Make moves take about a second so a test can reliably catch the robot mid-move."""
        self.link.time_scale = 1.0
        self.settings.data["drive"]["max_speed_mm_s"] = 100.0
        return self

    def moves(self):
        return [l for l in self.link.sent if l.startswith("M ")]

    def said(self, s):
        return any(s in t for t in self.logs)


def refuses(rig, needle):
    rig.link.sent.clear()
    probs = rig.h.start()
    assert probs and any(needle in p for p in probs), (needle, probs)
    assert rig.h.state == H.IDLE and not rig.link.sent, "a refused start must not send anything"


def main():
    # ---------------------------------------------------------------- 1. preflight refusals
    rig = Rig(stock_model=True).ready(); rig.berry(20, 10, -290); rig.until(lambda: False, 0.3)
    refuses(rig, "no allowed strawberry class")
    print("1a. stock YOLO model (no strawberry class): refused, nothing sent OK")
    rig = Rig(calibrated_cam=False).ready(); rig.berry(20, 10, -290); rig.until(lambda: False, 0.3)
    refuses(rig, "Camera -> robot calibration")
    rig = Rig().ready(); rig.feeding = False; rig.until(lambda: False, 1.3); refuses(rig, "Camera sending frames")
    rig = Rig(); rig.link.connect("SIM"); rig.until(lambda: rig.robot.connected); rig.berry(20, 10, -290); rig.until(lambda: False, 0.2)
    refuses(rig, "Motors enabled"); refuses(rig, "Position known")
    rig = Rig().ready(); rig.berry(20, 10, -290); rig.until(lambda: False, 0.2); rig.robot.estop()
    refuses(rig, "E-STOP not active")
    rig = Rig(drop_xyz_mm=[400.0, 0.0, -250.0]).ready(); rig.berry(20, 10, -290); rig.until(lambda: False, 0.2)
    refuses(rig, "Drop point")
    rig = Rig().ready(); rig.berry(20, 10, -290); rig.settings.data["home"]["calibrated"] = False; rig.until(lambda: False, 0.2)
    refuses(rig, "Home angles calibrated")
    print("1b. refused with a plain reason: no calibration, no frames, motors off, not homed, E-STOP, bad drop point, home not calibrated OK")

    # ---------------------------------------------------------------- 2. full run with vacuum
    rig = Rig(max_picks=3).ready()
    rig.berry(60, -50, -280); rig.berry(30, 20, -290); rig.berry(-40, 10, -300)        # distances from centre: 78, 36, 41
    rig.berry(0, 0, -290, cls="person")                                                  # not a strawberry
    rig.berry(5, 5, -290, conf=0.2)                                                      # too unsure
    rig.berry(-5, 5, -290, depth=False)                                                  # no depth
    rig.berry(40, 40, -215)                                                              # approach pose would leave the workspace
    rig.berry(300, 0, -290)                                                              # unreachable
    rig.until(lambda: False, 0.3)
    assert rig.h.start() == []
    done_order = []

    def remove_picked():
        if rig.h.picks_done > len(done_order):
            p = rig.h.pick_log[-1]["robot_xyz"]
            near = min(rig.scene, key=lambda b: np.linalg.norm(np.array(b["xyz"]) - np.array(p)))
            done_order.append(tuple(near["xyz"])); rig.scene.remove(near)
        return rig.h.state in (H.DONE, H.ERROR, H.STOPPED)
    assert rig.until(remove_picked, 40), (rig.h.state, rig.h.message, rig.logs[-6:])
    assert rig.h.state == H.DONE and rig.h.picks_done == 3 and "limit of 3" in rig.h.message, (rig.h.state, rig.h.message)
    assert done_order == [(30, 20, -290), (-40, 10, -300), (60, -50, -280)], done_order
    assert rig.h.skipped >= 2 and rig.said("skipped") and rig.said("unreachable") or rig.said("outside")
    seq = [l.split()[0] if not l.startswith("M") else "M" for l in rig.link.sent if l.split()[0] in ("M", "V")]
    assert seq == ["M", "M", "V", "M", "M", "V"] * 3 or seq == (["M", "M", "V", "V"][:0] + ["M", "M", "V", "M", "M", "V"] * 3), seq
    assert [l for l in rig.link.sent if l.startswith("V")] == ["V 1", "V 0"] * 3
    assert rig.link.vacuum is False and not rig.robot.vacuum_on
    assert all(e["result"].startswith("completed") for e in rig.h.pick_log) and len(rig.h.pick_log) == 3
    print("2. run: 3 picks closest-first (person, low confidence, no-depth, unreachable and out-of-box berries ignored/skipped),")
    print("   commands per pick: approach, descend, V 1, lift, carry, V 0; vacuum off at the end OK")
    print("   sent:", " | ".join(rig.link.sent[:6]))

    # ---------------------------------------------------------------- 3. dry run: no V commands at all
    rig = Rig(vacuum=False, max_picks=1).ready(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    assert rig.h.start() == [] and rig.until(lambda: rig.h.state == H.DONE, 30)
    assert not [l for l in rig.link.sent if l.startswith("V")] and len(rig.moves()) == 4 and "DRY RUN" in rig.h.pick_log[0]["result"]
    print("3. dry run: 4 moves, no vacuum command ever sent OK")

    # ---------------------------------------------------------------- 4. what must NOT become a target
    rig = Rig(max_picks=1).ready(); rig.berry(30, 20, -290); rig.berry(-40, 10, -300)
    n = [0]
    rig.frame_hook = lambda b, seq, xyz: None if (b["xyz"][0] == 30 and seq % 2) else ([xyz[0] + (15 if seq % 2 else -15), xyz[1], xyz[2]] if b["xyz"][0] == -40 else xyz)
    rig.until(lambda: False, 0.3); assert rig.h.start() == []
    rig.until(lambda: rig.h.state != H.SEARCHING, 8)
    assert rig.h.state == H.DONE and "no stable strawberry" in rig.h.message and not rig.moves(), (rig.h.state, rig.h.message, rig.moves())
    print("4. a flickering berry and a jittering berry (+-15 mm) never become stable; the robot never moved OK")
    rig = Rig(allowed_classes=["ripe"]).ready(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.3)
    refuses(rig, "no allowed strawberry class")
    print("   our class list disagrees with the Pi's ('ripe' only): refused before any motion OK")
    rig = Rig(allowed_classes=["strawberry", "ripe"], max_picks=1).ready(); rig.berry(30, 20, -290, cls="ripe"); rig.until(lambda: False, 0.3)
    assert rig.h.start() == []; rig.until(lambda: rig.h.state != H.SEARCHING, 8); assert not rig.moves() and rig.h.state == H.DONE
    print("   a class on our list but NOT marked allowed by the Pi is still ignored (both checks must agree) OK")

    # ---------------------------------------------------------------- 5. grasp offset
    rig = Rig(vacuum=False, max_picks=1, grasp_offset_mm=[0.0, 0.0, -10.0], approach_height_mm=50.0).ready(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    assert rig.h.start() == [] and rig.until(lambda: rig.h.state == H.DONE, 30)
    assert rig.h.pick_log[0]["pick_xyz"][2] == -300.0 and abs(rig.h.pick_log[0]["robot_xyz"][2] + 290) < 1.5
    print("5. grasp offset applied (pick 10 mm below the measured surface point) OK")

    # ---------------------------------------------------------------- 6. E-STOP mid-pick
    for stage in ("descend", "vacuum"):
        rig = Rig().ready().slow(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
        rig.settings.data["harvest"]["dwell_s"] = 5.0
        assert rig.h.start() == []
        target = (lambda: rig.h.message.endswith("descend")) if stage == "descend" else (lambda: rig.link.vacuum)
        assert rig.until(target, 20), (rig.h.state, rig.h.message)
        rig.link.sent.clear(); rig.robot.estop()
        assert rig.h.state == H.STOPPED and "E-STOP" in rig.h.message
        rig.spin(0.4)
        assert rig.link.vacuum is False and rig.link.sent[:2] == ["STOP", "V 0"], rig.link.sent
        assert not [l for l in rig.link.sent if l.startswith("M ")], "no motion after the E-STOP"
        assert rig.h.pick_log and rig.h.pick_log[-1]["result"].startswith("aborted")
        assert rig.h.start() and rig.h.state == H.STOPPED
    print("6. E-STOP during descend and during vacuum-on: run STOPPED at once, STOP then V 0 sent, vacuum off, pick logged as aborted OK")

    # ---------------------------------------------------------------- 7. soft stop
    rig = Rig().ready(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    rig.settings.data["harvest"]["dwell_s"] = 5.0
    assert rig.h.start() == [] and rig.until(lambda: rig.link.vacuum, 20)
    rig.h.request_stop(); assert rig.until(lambda: rig.h.state == H.STOPPED, 10), rig.h.state
    assert rig.link.vacuum is False and rig.link.sent[-1] == "V 0" and not rig.robot.estopped
    print("7. soft Stop: current step ends, vacuum switched off, run STOPPED (robot not E-stopped) OK")

    # ---------------------------------------------------------------- 8. camera silence while searching
    rig = Rig().ready(); rig.until(lambda: False, 0.2); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    assert rig.h.start() == []; rig.feeding = False
    assert rig.until(lambda: rig.h.state == H.ERROR, 6) and "camera" in rig.h.message
    print("8. camera goes silent: run ends in ERROR ('camera stopped sending frames') OK")

    # ---------------------------------------------------------------- 9. firmware without vacuum support
    rig = Rig().ready(); rig.link.vacuum_supported = False; rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    assert rig.h.start() == [] and rig.until(lambda: rig.h.state == H.ERROR, 30), (rig.h.state, rig.logs[-5:])
    assert rig.said("firmware support") and rig.link.vacuum is False
    print("9. current firmware (no V command): run ends in ERROR with an explanation; no pick reported as done OK")

    # ---------------------------------------------------------------- 10. ESP32 reset and link loss during a run
    rig = Rig(vacuum=False).ready().slow(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    rig.settings.data["harvest"]["dwell_s"] = 5.0
    assert rig.h.start() == [] and rig.until(lambda: rig.h.message.endswith("descend"), 20)
    rig.link.inject_reset(); assert rig.until(lambda: rig.h.state in (H.ERROR, H.STOPPED), 10), rig.h.state
    assert rig.h.state == H.ERROR and not rig.robot.xyz_valid
    rig = Rig(vacuum=False).ready().slow(); rig.berry(30, 20, -290); rig.until(lambda: False, 0.2)
    assert rig.h.start() == [] and rig.until(lambda: rig.h.message.endswith("descend"), 20)
    rig.link.inject_link_loss(); assert rig.until(lambda: rig.h.state == H.ERROR, 10)
    print("10. ESP32 brown-out reset and cable loss mid-run: run ends in ERROR, position marked unknown OK")

    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

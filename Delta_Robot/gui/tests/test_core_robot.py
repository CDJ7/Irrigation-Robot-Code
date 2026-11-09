"""
Tests the robot controller (delta_gui/core) against the SIMULATED ESP32 -- no hardware, no screen.
Run:  python tests/test_core_robot.py        (needs numpy and PyYAML only)

Covers every safety rule: refusals (not connected / drivers off / position unknown / busy / outside box / unreachable / joint
limit), reply matching (the OK ambiguity), E-STOP latching, stop during a move and during homing, lost reply, brown-out reset,
link loss, step-rate limit, home/offset consistency and position recovery.
"""
import copy
import os
import queue
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from delta_gui.core.kinematics import DeltaKinematics               # noqa: E402
from delta_gui.core.link import SimulatedESP32Link                  # noqa: E402
from delta_gui.core.robot import Robot                              # noqa: E402
from delta_gui.core.settings import Settings                        # noqa: E402

SCALE = 40.0          # simulated firmware runs 40x faster than real time


class Rig:
    """A robot + simulated ESP32, with a queue standing in for the GUI thread (link callbacks arrive on other threads)."""
    def __init__(self, settings=None, home_seconds=2.0):
        d = tempfile.mkdtemp()
        self.settings = settings or Settings(os.path.join(d, "s.json"))
        self.clock_now = [1000.0]
        self.link = SimulatedESP32Link(time_scale=SCALE, home_seconds=home_seconds)
        self.robot = Robot(self.settings, self.link, clock=lambda: self.clock_now[0])
        self.q = queue.Queue()
        self.link.on_line = lambda t: self.q.put(("line", t))
        self.link.on_state = lambda c, m: self.q.put(("state", c, m))
        self.logs = []
        self.robot.listeners.append(lambda ev, data: self.logs.append((data["level"], data["text"])) if ev == "log" else None)

    def pump(self, seconds=0.05):
        end = time.time() + seconds
        while time.time() < end:
            try:
                item = self.q.get(timeout=0.01)
            except queue.Empty:
                continue
            if item[0] == "line":
                self.robot.on_line(item[1])
            else:
                self.robot.on_link_state(item[1], item[2])

    def wait(self, pred, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.02)
            if pred():
                return True
        return False

    def connect_enable_home(self):
        self.link.connect("SIM")
        assert self.wait(lambda: self.robot.connected)
        assert self.robot.enable(True)
        assert self.wait(lambda: self.robot.drivers_enabled)
        assert self.robot.home()
        assert self.wait(lambda: self.robot.xyz_valid and not self.robot.busy), self.logs[-3:]

    def said(self, needle):
        return any(needle in t for _, t in self.logs)


def main():
    # ----------------------------------------------------------------- 1. connect, refusals before enable/home
    r = Rig()
    assert r.robot.state_name == "DISCONNECTED"
    assert not r.robot.move_xyz(0, 0, -250), "no move without a connection"
    r.link.connect("SIM")
    assert r.wait(lambda: r.robot.connected)
    assert r.robot.state_name == "POSITION UNKNOWN" and r.robot.drivers_enabled is False
    res = r.robot.move_xyz(0, 0, -250)
    assert not res and "enable the motors" in res.message, res
    assert r.robot.enable(True) and r.wait(lambda: r.robot.drivers_enabled is True)
    res = r.robot.move_xyz(0, 0, -250)
    assert not res and "home" in res.message.lower(), res
    print("1. refusals: not connected / drivers off / position unknown OK")

    # ----------------------------------------------------------------- 2. home, then a normal move
    assert r.robot.home() and r.robot.state_name == "HOMING"
    assert not r.robot.move_xyz(0, 0, -250), "busy while homing"
    assert r.wait(lambda: r.robot.xyz_valid and not r.robot.busy)
    assert r.robot.current_steps == [0, 0, 0] and r.robot.current_xyz == r.robot.home_xyz and r.robot.state_name == "IDLE"
    assert r.said("WARNING"), "default HOME and offsets disagree -> must warn at homing"
    assert r.robot.move_xyz(20, -30, -280)
    assert r.robot.state_name == "MOVING"
    assert r.wait(lambda: not r.robot.busy)
    k = r.robot.kin
    exp = r.robot._steps_for(k.inverse(20, -30, -280))
    assert r.robot.current_steps == exp == r.link.steps, (r.robot.current_steps, exp, r.link.steps)
    assert r.robot.current_xyz == [20, -30, -280] and r.robot.xyz_valid
    print("2. home + XYZ move: GUI steps == firmware steps ==", exp)

    # ----------------------------------------------------------------- 3. refusals of bad targets
    for tgt, word in [((500, 0, -280), "outside the workspace"), ((0, 0, -100), "outside the workspace"),
                      ((float("nan"), 0, -280), "non-numeric")]:
        res = r.robot.move_xyz(*tgt)
        assert not res and word in res.message, (tgt, res)
    # unreachable / joint limit: shrink the arms so a point inside the box cannot be reached, then tighten the angle limit
    s2 = copy.deepcopy(r.settings.data); s2["geometry"]["forearm_mm"] = 120.0; s2["geometry"]["bicep_mm"] = 100.0
    r.settings.data = s2; r.robot.apply_settings()
    res = r.robot.move_xyz(0, 0, -300)
    assert not res and "unreachable" in res.message, res
    s2["geometry"].update(bicep_mm=150.0, forearm_mm=280.0); s2["axes"]["theta_max_deg"] = 30.0
    r.settings.data = s2; r.robot.apply_settings()
    res = r.robot.move_xyz(0, 0, -340)
    assert not res and "allowed range" in res.message, res
    s2["axes"]["theta_max_deg"] = 80.0; r.settings.data = s2; r.robot.apply_settings()
    print("3. workspace box / non-numeric / unreachable / joint-limit refusals OK")

    # ----------------------------------------------------------------- 4. the OK ambiguity: 'EN' during a move
    assert r.robot.move_xyz(-20, 30, -300)
    res = r.robot.enable(False)
    assert not res and "busy" in res.message, "EN must not be sent while a move is in flight"
    assert not r.robot.set_zero() and not r.robot.home()
    assert r.wait(lambda: not r.robot.busy)
    assert r.robot.xyz_valid and r.robot.current_xyz == [-20, 30, -300]
    assert r.robot.enable(False) and r.wait(lambda: r.robot.drivers_enabled is False)
    assert not r.robot.xyz_valid and "motors were disabled" in r.robot.invalid_reason
    print("4. EN refused while moving; disabling the drivers invalidates the position OK")

    # ----------------------------------------------------------------- 5. raw move invalidates XYZ
    assert r.robot.enable(True) and r.wait(lambda: r.robot.drivers_enabled)
    assert r.robot.home() and r.wait(lambda: r.robot.xyz_valid and not r.robot.busy)
    assert r.robot.raw_move([300, -300, 300], 200)
    assert not r.robot.xyz_valid and r.robot.state_name == "MOVING"
    assert r.wait(lambda: not r.robot.busy) and r.robot.current_steps == [300, -300, 300] == r.link.steps
    assert not r.robot.move_xyz(0, 0, -250), "XYZ moves refused after a raw move until homed"
    assert r.robot.raw_move_size_deg([4177, 0, 0]) > 9
    assert r.robot.home() and r.wait(lambda: r.robot.xyz_valid and not r.robot.busy)
    print("5. raw move -> xyz_valid False -> re-home restores OK")

    # ----------------------------------------------------------------- 6. E-STOP mid-move: latched, position unknown
    assert r.robot.move_xyz(80, 80, -340)
    time.sleep(0.003); r.pump(0.01)
    r.robot.estop()
    assert r.robot.estopped and r.robot.state_name == "E-STOP" and not r.robot.xyz_valid and r.robot.drivers_enabled is False
    r.pump(0.3)
    assert r.link.sent[-2:] == ["STOP", "EN 0"], r.link.sent[-3:]
    for fn in (lambda: r.robot.move_xyz(0, 0, -250), r.robot.home, lambda: r.robot.enable(True), lambda: r.robot.jog(1, 0, 0)):
        res = fn(); assert not res and "E-STOP" in res.message, res
    assert r.link.steps != [0, 0, 0], "simulated arm stopped part-way"
    r.robot.release_estop()
    assert r.robot.state_name == "POSITION UNKNOWN"
    assert not r.robot.move_xyz(0, 0, -250)
    assert r.robot.enable(True) and r.wait(lambda: r.robot.drivers_enabled)
    assert r.robot.home() and r.wait(lambda: r.robot.xyz_valid and not r.robot.busy)
    print("6. E-STOP mid-move: STOP + EN 0 sent, everything refused until released, re-home recovers OK")

    # ----------------------------------------------------------------- 7. E-STOP during homing; setting 'drivers stay on'
    r2 = Rig(home_seconds=20.0)
    r2.settings.data["safety"]["estop_disables_drivers"] = False
    r2.link.connect("SIM"); assert r2.wait(lambda: r2.robot.connected)
    assert r2.robot.enable(True) and r2.wait(lambda: r2.robot.drivers_enabled)
    assert r2.robot.home(); r2.pump(0.05)
    r2.robot.estop(); r2.pump(0.4)
    assert r2.said("HOMING_ABORTED") or r2.said("ESP32 reported"), r2.logs[-4:]
    assert r2.robot.drivers_enabled is True and "EN 0" not in r2.link.sent
    assert not r2.robot.xyz_valid
    print("7. E-STOP during homing aborts it; 'keep drivers on' setting honoured OK")

    # ----------------------------------------------------------------- 8. lost reply -> timeout -> unknown
    r3 = Rig(); r3.connect_enable_home()
    r3.link.drop_replies = 1
    assert r3.robot.move_xyz(10, 10, -280); r3.pump(0.3)
    assert r3.robot.busy
    r3.clock_now[0] += 60; r3.robot.tick()
    assert not r3.robot.busy and not r3.robot.xyz_valid and "no reply" in r3.robot.invalid_reason
    print("8. lost reply: watchdog gives up and marks position unknown OK")

    # ----------------------------------------------------------------- 9. brown-out reset and link loss
    r3.connect_enable_home()
    assert r3.robot.xyz_valid
    r3.link.inject_reset(); r3.pump(0.2)
    assert not r3.robot.xyz_valid and r3.robot.drivers_enabled is False and r3.said("restarted unexpectedly")
    r3.connect_enable_home()
    r3.link.inject_link_loss(); r3.pump(0.2)
    assert not r3.robot.connected and not r3.robot.xyz_valid and r3.robot.state_name == "DISCONNECTED"
    print("9. unexpected ESP32 reset and link loss both invalidate the position OK")

    # ----------------------------------------------------------------- 10. step-rate limit lengthens fast moves
    r4 = Rig(); r4.connect_enable_home()
    r4.settings.data["drive"]["max_speed_mm_s"] = 1000.0     # absurdly fast tip speed...
    r4.settings.data["drive"]["max_step_rate"] = 2000.0      # ...but the step rate must still limit it
    r4.robot.apply_settings()
    assert r4.robot.move_xyz(60, -60, -330)
    dur = int(r4.link.sent[-1].split()[-1])
    big = max(abs(a) for a in r4.robot._steps_for(r4.robot.kin.inverse(60, -60, -330)))
    assert dur >= big / 2000.0 * 1000 - 1, (dur, big)
    print(f"10. step-rate limit: {big} steps need >= {big/2000*1000:.0f} ms, duration sent {dur} ms OK")

    # ----------------------------------------------------------------- 11. home/offset consistency + position recovery
    r5 = Rig(); r5.connect_enable_home()
    assert r5.robot.home_consistency() is not None
    a = r5.robot.kin.inverse(*r5.robot.home_xyz)
    r5.settings.data["axes"]["offsets_deg"] = [-v for v in a]      # step 0 == the HOME pose
    r5.robot.apply_settings()
    assert r5.robot.home_consistency() is None, r5.robot.home_consistency()
    assert not r5.robot.recover_position_from_steps().ok          # not allowed until home is declared calibrated
    r5.settings.data["home"]["calibrated"] = True; r5.robot.apply_settings()
    assert r5.robot.home() and r5.wait(lambda: r5.robot.xyz_valid and not r5.robot.busy)
    assert r5.robot.move_xyz(40, -20, -300) and r5.wait(lambda: not r5.robot.busy)
    r5.robot.estop(); r5.pump(0.2); r5.robot.release_estop()
    assert not r5.robot.xyz_valid
    assert r5.robot.recover_position_from_steps().ok
    assert all(abs(u - v) < 0.2 for u, v in zip(r5.robot.current_xyz, (40, -20, -300))), r5.robot.current_xyz
    print("11. home consistency check and position recovery from step counts OK ->", [round(v, 2) for v in r5.robot.current_xyz])

    # ----------------------------------------------------------------- 12. jog uses the same checks
    r6 = Rig(); r6.connect_enable_home()
    r6.settings.data["axes"]["offsets_deg"] = [-v for v in r6.robot.kin.inverse(*r6.robot.home_xyz)]; r6.robot.apply_settings()
    assert r6.robot.move_xyz(0, 0, -250) and r6.wait(lambda: not r6.robot.busy)
    assert r6.robot.jog(5, 0, 0) and r6.wait(lambda: not r6.robot.busy) and abs(r6.robot.current_xyz[0] - 5) < 1e-9
    r6.robot.move_xyz(95, 0, -250); r6.wait(lambda: not r6.robot.busy)
    assert not r6.robot.jog(10, 0, 0), "jog beyond the box edge must be refused"
    print("12. jog follows the same rules OK")
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

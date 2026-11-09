"""Runs one complete simulated harvest and prints the log (what the GUI's log dock would show). Run: python tests/demo_harvest_sim.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_core_harvest as T                                           # noqa: E402

rig = T.Rig(max_picks=3, approach_height_mm=60.0).ready()
rig.berry(60, -50, -280); rig.berry(30, 20, -290); rig.berry(-40, 10, -300)
rig.berry(0, 0, -290, cls="person"); rig.berry(5, 5, -290, conf=0.2); rig.berry(40, 40, -215); rig.berry(300, 0, -290)
rig.logs.clear(); rig.until(lambda: False, 0.3)
assert rig.h.start() == []
picked = []
def step():
    if rig.h.picks_done > len(picked):
        near = min(rig.scene, key=lambda b: sum((a - c) ** 2 for a, c in zip(b["xyz"], rig.h.pick_log[-1]["robot_xyz"])))
        picked.append(near); rig.scene.remove(near)
    return rig.h.state in (T.H.DONE, T.H.ERROR, T.H.STOPPED)
rig.until(step, 60)
print("SIMULATED ESP32 + SIMULATED CAMERA (nothing real moved)\n")
for t in rig.logs:
    print(t)
print("\nfinal state:", rig.h.state, "-", rig.h.message)

"""
Tests delta_gui/core/sac_policy.py: the observation the SAC policy sees (layout, nearest-first order, clipping), action -> target
clipping to the workspace box, the berry memory (keeps occluded berries a few frames, then forgets), and the numpy policy runner
(file checks + a hand-made network). No PyTorch needed.
Run:  python tests/test_core_sac_policy.py     (needs numpy)
"""
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from delta_gui.core.sac_policy import (BERRY_FEATURES, MAX_BERRIES, MAX_STEP_MM, NEAR_BERRY_MM, OBS_SIZE,   # noqa: E402
                                       OBS_VERSION, SLOW_STEP_MM, BerryMemory, NumpyPolicy, SacStepper, action_to_target, build_observation)
from delta_gui.core.settings import DEFAULTS                                                            # noqa: E402

WS = DEFAULTS["workspace_mm"]


def berry(x, y, z, conf=0.9, dvf=0.8):
    return {"xyz_mm": [x, y, z], "confidence": conf, "depth_valid_fraction": dvf}


def test_observation():
    tip = [0.0, 0.0, -280.0]
    obs = build_observation(tip, [berry(50, 0, -300), berry(10, 0, -290), {"xyz_mm": None}, berry(0, 400, -300),
                                  berry(-60, 0, -280)], [0.5, -2.0, 0.0], WS)
    assert obs.shape == (OBS_SIZE,) and obs.dtype == np.float32
    assert abs(obs[0]) < 1e-6 and abs(obs[1]) < 1e-6 and abs(obs[2] - 0.0) < 1e-6        # centre of the box
    first = obs[3:3 + BERRY_FEATURES]
    assert np.allclose(first, [0.1, 0.0, -0.1, 0.9, 0.8, 1.0], atol=1e-6), first          # nearest berry first
    assert np.allclose(obs[3 + BERRY_FEATURES:3 + BERRY_FEATURES + 3], [0.5, 0.0, -0.2], atol=1e-6)
    assert np.allclose(obs[3 + 2 * BERRY_FEATURES:3 + 2 * BERRY_FEATURES + 3], [-0.6, 0.0, 0.0], atol=1e-6)
    assert np.allclose(obs[-3:], [0.5, -1.0, 0.0])                                        # previous action clipped
    # the far berry (4th by distance) is dropped; with only one berry the other slots are zero, present=0
    one = build_observation(tip, [berry(0, 400, -300)], [0, 0, 0], WS)
    assert one[3 + 1] == 3.0                                                              # 400 mm clipped to +3
    assert not one[3 + BERRY_FEATURES:3 + MAX_BERRIES * BERRY_FEATURES].any()
    print("observation OK (layout, nearest first, clipping, empty slots)")


def test_action_to_target():
    t = action_to_target([0, 0, -280], [1, -1, 0.5], WS)
    assert np.allclose(t, [MAX_STEP_MM, -MAX_STEP_MM, -280 + 0.5 * MAX_STEP_MM])
    t = action_to_target([95, -95, -212], [5, -5, 1], WS)                                 # clipped to the box
    assert t == (100.0, -100.0, -210.0), t
    # near a berry: shrunk to SLOW_STEP_MM, direction kept; far from it: full step
    near = [berry(0, 0, -280 - NEAR_BERRY_MM + 1)]
    t = action_to_target([0, 0, -280], [0.5, 0, -1], WS, berries=near)
    assert np.allclose(t, [0.5 * SLOW_STEP_MM, 0, -280 - SLOW_STEP_MM]), t
    t = action_to_target([0, 0, -280], [0.5, 0, -1], WS, berries=[berry(0, 0, -280 - NEAR_BERRY_MM - 1)])
    assert np.allclose(t, [0.5 * MAX_STEP_MM, 0, -280 - MAX_STEP_MM]), t
    t = action_to_target([0, 0, -280], [0.1, 0, -0.2], WS, berries=near)              # already small: unchanged
    assert np.allclose(t, [0.1 * MAX_STEP_MM, 0, -280 - 0.2 * MAX_STEP_MM]), t
    print("action -> target OK (scaled, slowed near a berry, clipped to the workspace box)")


def test_memory():
    m = BerryMemory(match_radius_mm=20, max_age=3, decay=0.5)
    assert len(m.update([berry(0, 0, -300)])) == 1
    k = m.update([])                                                                      # hidden: remembered, half confidence
    assert len(k) == 1 and k[0]["age"] == 1 and abs(k[0]["confidence"] - 0.45) < 1e-9
    k = m.update([berry(5, 0, -300, conf=0.8)])                                           # seen again nearby: replaced, not doubled
    assert len(k) == 1 and k[0]["age"] == 0 and k[0]["xyz_mm"][0] == 5
    for _ in range(3):
        k = m.update([])
    assert len(k) == 1 and k[0]["age"] == 3
    assert m.update([]) == []                                                             # forgotten after max_age
    k = m.update([berry(0, 0, -300), berry(100, 0, -300)])
    assert len(k) == 2
    print("berry memory OK (keeps hidden berries, re-matches, forgets)")


def _write_policy(path, version=OBS_VERSION, size=OBS_SIZE):
    """A hand-made 'policy': one hidden ReLU layer copying the nearest berry's relative position -> moves towards it."""
    W0 = np.zeros((6, size), np.float32)
    for i in range(3):
        W0[2 * i, 3 + i], W0[2 * i + 1, 3 + i] = 1.0, -1.0                                   # +r and -r through ReLU
    W_mu = np.zeros((3, 6), np.float32)
    for i in range(3):
        W_mu[i, 2 * i], W_mu[i, 2 * i + 1] = 10.0, -10.0
    np.savez(path, obs_version=np.int64(version), obs_size=np.int64(size), n_hidden=np.int64(1), max_step_mm=np.float64(15),
             W0=W0, b0=np.zeros(6, np.float32), W_mu=W_mu, b_mu=np.zeros(3, np.float32))


def test_numpy_policy_and_stepper():
    d = tempfile.mkdtemp()
    good, bad = os.path.join(d, "p.npz"), os.path.join(d, "old.npz")
    _write_policy(good)
    _write_policy(bad, version=OBS_VERSION + 1)
    try:
        NumpyPolicy(bad)
        raise AssertionError("a policy for another observation version must be refused")
    except ValueError as e:
        assert "version" in str(e)
    st = SacStepper(NumpyPolicy(good), WS)
    assert st.step([0, 0, -250], []) is None                                              # nothing seen, nothing remembered
    tip = [0.0, 0.0, -250.0]
    for _ in range(25):
        tgt = st.step(tip, [berry(40, -30, -300)])
        assert all(WS[a][0] <= v <= WS[a][1] for a, v in zip("xyz", tgt))
        step = max(abs(u - v) for u, v in zip(tgt, tip))
        assert step <= MAX_STEP_MM + 1e-9
        if np.linalg.norm(np.subtract(tip, [40, -30, -300])) <= NEAR_BERRY_MM:
            assert step <= SLOW_STEP_MM + 1e-9
        tip = list(tgt)
    assert np.linalg.norm(np.subtract(tip, [40, -30, -300])) < 3.0, tip
    assert st.step(tip, []) is not None                                                   # berry hidden: memory keeps it
    st.reset()
    assert st.step(tip, []) is None and not st.prev_action.any()
    print("numpy policy + stepper OK (version check, bounded steps, reaches a berry, memory, reset)")


if __name__ == "__main__":
    test_observation(); test_action_to_target(); test_memory(); test_numpy_policy_and_stepper()
    print("\nALL TESTS PASSED")

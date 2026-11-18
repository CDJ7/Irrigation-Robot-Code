"""
evaluate.py -- measure a policy in the simulator (same seeds for every policy, so results are comparable).

    .venv/bin/python sac/evaluate.py runs/<run>/sac_policy.npz [--episodes 500] [--noise 1.5]
"""
import argparse
import json
import os
import sys
from typing import Callable

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "gui"))
from delta_pick_env import DeltaPickEnv                       # noqa: E402
from delta_gui.core.sac_policy import BERRY_FEATURES, REL_SCALE_MM   # noqa: E402


def scripted_policy(obs: np.ndarray) -> np.ndarray:
    """Hand-written baseline: hover 40 mm above the nearest seen berry, then descend onto it."""
    rel = obs[3:6] * REL_SCALE_MM
    if obs[3 + BERRY_FEATURES - 1] == 0:          # no berry seen or remembered
        return np.zeros(3, dtype=np.float32)
    tgt = rel.copy()
    if np.hypot(rel[0], rel[1]) > 8.0:
        tgt[2] = rel[2] + 40.0
    return np.clip(tgt / 15.0, -1, 1).astype(np.float32)


def evaluate(act: Callable[[np.ndarray], np.ndarray], episodes: int = 500, noise: float = 1.0, seed0: int = 10_000) -> dict:
    env = DeltaPickEnv(noise_scale=noise)
    succ, steps, final, knocked, blocked = 0, [], [], 0, 0
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed0 + ep)
        done, ep_knock = False, False
        while not done:
            obs, _, term, trunc, info = env.step(act(obs))
            done = term or trunc
            ep_knock |= info["knocked"]
            blocked += info["blocked"]
        succ += info["success"]
        knocked += ep_knock
        final.append(info["dist_mm"])
        if info["success"]:
            steps.append(env.steps)
    return {"episodes": episodes, "noise_scale": noise, "success_rate": round(succ / episodes, 3),
            "mean_steps_when_successful": round(float(np.mean(steps)), 1) if steps else None,
            "median_final_dist_mm": round(float(np.median(final)), 1),
            "episodes_knocking_a_berry": knocked, "refused_moves": int(blocked)}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("policy", nargs="?", help=".npz policy; omit to evaluate the scripted baseline")
    ap.add_argument("--episodes", type=int, default=500)
    ap.add_argument("--noise", type=float, default=1.0, help="multiply all simulated noise (robustness check)")
    a = ap.parse_args()
    if a.policy:
        from delta_gui.core.sac_policy import NumpyPolicy
        fn = NumpyPolicy(a.policy).act
    else:
        fn = scripted_policy
    print(json.dumps(evaluate(fn, a.episodes, a.noise), indent=2))

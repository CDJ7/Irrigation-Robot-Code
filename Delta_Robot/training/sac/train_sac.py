"""
train_sac.py -- train the SAC picking policy in the simulator, then export it for the GUI (numpy .npz, no PyTorch needed).

    cd Delta_Robot/training
    .venv/bin/python sac/train_sac.py --steps 300000            # ~20-40 min on a laptop GPU/CPU
    .venv/bin/python sac/train_sac.py --steps 2000 --out /tmp/x # smoke test

Outputs (in --out, default training/sac/runs/<date-time>/):
    sac_model.zip      full SB3 model (to continue training)
    sac_policy.npz     the actor only, for delta_gui.core.sac_policy.NumpyPolicy  <- what the GUI loads
    eval.json          success rate etc. of the trained policy and of the scripted baseline, on the same seeds
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from delta_pick_env import DeltaPickEnv            # noqa: E402
from export_policy import export_actor             # noqa: E402
from evaluate import evaluate, scripted_policy     # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--out", default=os.path.join(HERE, "runs", time.strftime("%Y%m%d-%H%M%S")))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--eval-episodes", type=int, default=500)
    args = ap.parse_args()

    from stable_baselines3 import SAC
    from stable_baselines3.common.env_util import make_vec_env

    os.makedirs(args.out, exist_ok=True)
    env = make_vec_env(DeltaPickEnv, n_envs=args.n_envs, seed=args.seed)
    model = SAC("MlpPolicy", env, seed=args.seed, device=args.device, verbose=1,
                learning_rate=3e-4, buffer_size=500_000, batch_size=256, gamma=0.98, tau=0.005,
                train_freq=1, gradient_steps=-1,   # -1 = one update per collected sample (8 per round)
                learning_starts=5_000,
                policy_kwargs={"net_arch": [256, 256]}, tensorboard_log=os.path.join(args.out, "tb"))
    t0 = time.time()
    model.learn(total_timesteps=args.steps, log_interval=50, progress_bar=False)
    model.save(os.path.join(args.out, "sac_model.zip"))
    npz = os.path.join(args.out, "sac_policy.npz")
    export_actor(model, npz)

    sys.path.insert(0, os.path.join(HERE, "..", "..", "gui"))
    from delta_gui.core.sac_policy import NumpyPolicy
    pol = NumpyPolicy(npz)
    report = {"train_steps": args.steps, "train_minutes": round((time.time() - t0) / 60, 1),
              "sac": evaluate(pol.act, args.eval_episodes), "scripted_baseline": evaluate(scripted_policy, args.eval_episodes)}
    with open(os.path.join(args.out, "eval.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    print("policy for the GUI:", npz)


if __name__ == "__main__":
    main()

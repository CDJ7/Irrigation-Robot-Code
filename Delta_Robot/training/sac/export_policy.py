"""
export_policy.py -- write a trained SB3 SAC actor as a small .npz that delta_gui.core.sac_policy.NumpyPolicy runs with numpy.

    .venv/bin/python sac/export_policy.py runs/<run>/sac_model.zip runs/<run>/sac_policy.npz
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gui"))
from delta_gui.core.sac_policy import MAX_STEP_MM, OBS_SIZE, OBS_VERSION   # noqa: E402


def export_actor(model, path: str) -> None:
    import torch.nn as nn
    actor = model.policy.actor
    linears = [m for m in actor.latent_pi if isinstance(m, nn.Linear)]
    acts = [m for m in actor.latent_pi if not isinstance(m, nn.Linear)]
    if not all(isinstance(a, nn.ReLU) for a in acts):
        raise ValueError("export expects ReLU hidden layers (SB3's default for SAC)")
    arrays = {"obs_version": np.int64(OBS_VERSION), "obs_size": np.int64(OBS_SIZE), "n_hidden": np.int64(len(linears)),
              "max_step_mm": np.float64(MAX_STEP_MM)}
    for i, lin in enumerate(linears):
        arrays[f"W{i}"] = lin.weight.detach().cpu().numpy()
        arrays[f"b{i}"] = lin.bias.detach().cpu().numpy()
    arrays["W_mu"] = actor.mu.weight.detach().cpu().numpy()
    arrays["b_mu"] = actor.mu.bias.detach().cpu().numpy()
    np.savez(path, **arrays)


if __name__ == "__main__":
    from stable_baselines3 import SAC
    export_actor(SAC.load(sys.argv[1], device="cpu"), sys.argv[2])
    print("wrote", sys.argv[2])

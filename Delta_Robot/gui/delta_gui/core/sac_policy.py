"""
sac_policy.py -- the SAC (Soft Actor-Critic) picking policy's interface: what it SEES, what it DOES, and a numpy-only runner.

Design (see training/sac/README.md):
    The policy does NOT look at camera pixels. YOLO + stereo depth on the Pi turn every frame into a short list of berries
    (X, Y, Z, confidence, depth quality). The GUI converts them to robot coordinates with the camera->robot calibration, and
    this module packs them, together with the tip position, into a fixed-size vector. That vector is easy to simulate
    faithfully, so a policy trained in simulation (training/sac/) sees the same kind of numbers on the real robot.

    Every camera frame the policy outputs a small step (dx, dy, dz) for the tip ("closed-loop moves"). The step is turned into
    an absolute target here, clipped to the workspace box -- and the GUI must STILL pass that target through Robot.move_xyz /
    check_target, so the policy can never bypass the workspace box, reachability, joint limits or E-STOP.

This file is shared by the simulator and the GUI: change the observation here and you MUST retrain the policy (the policy file
stores OBS_VERSION and refuses to load on a mismatch).

Pure numpy, no Qt, no torch.
"""
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

OBS_VERSION = 2                 # 2 = near-berry speed limit added (same inputs, but the moves behave differently -> retrain)
MAX_BERRIES = 3                 # the K nearest berries are shown to the policy; the rest are ignored
BERRY_FEATURES = 6              # rel_x, rel_y, rel_z, confidence, depth_valid_fraction, present
REL_SCALE_MM = 100.0            # relative positions are divided by this (so +-1 ~ +-10 cm)
REL_CLIP = 3.0                  # ... and clipped to +-3 (30 cm) so a far false detection cannot produce huge inputs
MAX_STEP_MM = 15.0              # largest move per decision along each axis
NEAR_BERRY_MM = 30.0            # within this distance of a (seen or remembered) berry ...
SLOW_STEP_MM = 5.0              # ... a move is shrunk so no axis moves more than this: the cup touches down gently, and depth
                                # noise (~5-8 mm) cannot turn one big last step into pushing through the berry
OBS_SIZE = 3 + MAX_BERRIES * BERRY_FEATURES + 3   # tip position + berries + previous action  = 24

Triple = Tuple[float, float, float]


def _norm_in_box(v: float, lo: float, hi: float) -> float:
    """Map [lo, hi] -> [-1, 1]."""
    return 2.0 * (v - lo) / (hi - lo) - 1.0 if hi > lo else 0.0


def build_observation(tip_mm: Sequence[float], berries: Iterable[Dict], prev_action: Sequence[float],
                      workspace: Dict[str, Sequence[float]]) -> np.ndarray:
    """
    tip_mm       current tip position in ROBOT coordinates (mm)
    berries      dicts with "xyz_mm" (ROBOT coordinates), "confidence", "depth_valid_fraction"; only usable berries
                 (allowed class, valid depth) should be passed -- filtering is the caller's job, as in the Harvester
    prev_action  the policy's previous action, each in [-1, 1] (zeros at the start of an episode)
    workspace    {"x": [lo, hi], "y": [lo, hi], "z": [lo, hi]}  (settings["workspace_mm"])
    """
    tip = [float(v) for v in tip_mm]
    obs = np.zeros(OBS_SIZE, dtype=np.float32)
    obs[0] = _norm_in_box(tip[0], *workspace["x"])
    obs[1] = _norm_in_box(tip[1], *workspace["y"])
    obs[2] = _norm_in_box(tip[2], *workspace["z"])
    rows: List[Tuple[float, List[float]]] = []
    for b in berries:
        p = b.get("xyz_mm")
        if p is None:
            continue
        rel = [float(p[i]) - tip[i] for i in range(3)]
        rows.append((math.sqrt(sum(r * r for r in rel)),
                     [max(-REL_CLIP, min(REL_CLIP, r / REL_SCALE_MM)) for r in rel]
                     + [float(b.get("confidence", 0.0)), float(b.get("depth_valid_fraction", 1.0)), 1.0]))
    rows.sort(key=lambda r: r[0])                                   # nearest first: a stable, order-independent layout
    for k, (_, feat) in enumerate(rows[:MAX_BERRIES]):
        obs[3 + k * BERRY_FEATURES: 3 + (k + 1) * BERRY_FEATURES] = feat
    obs[3 + MAX_BERRIES * BERRY_FEATURES:] = np.clip(np.asarray(prev_action, dtype=np.float32), -1.0, 1.0)
    return obs


def step_limit_mm(tip_mm: Sequence[float], berries: Iterable[Dict], max_step_mm: float = MAX_STEP_MM) -> float:
    """Largest allowed move along any axis: SLOW_STEP_MM near a berry, max_step_mm elsewhere. A rule, not learned."""
    for b in berries:
        p = b.get("xyz_mm")
        if p is not None and math.dist([float(v) for v in p], [float(v) for v in tip_mm]) <= NEAR_BERRY_MM:
            return min(SLOW_STEP_MM, max_step_mm)
    return max_step_mm


def action_to_target(tip_mm: Sequence[float], action: Sequence[float], workspace: Dict[str, Sequence[float]],
                     max_step_mm: float = MAX_STEP_MM, berries: Optional[Iterable[Dict]] = None) -> Triple:
    """Policy action (each in [-1, 1]) -> absolute XYZ target: scaled to max_step_mm, shrunk (direction kept) to SLOW_STEP_MM
    when a berry is within NEAR_BERRY_MM, then clipped to the workspace box.
    The result still has to go through Robot.move_xyz (reachability, joint limits, E-STOP...)."""
    a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    delta = a * max_step_mm
    limit = step_limit_mm(tip_mm, berries or [], max_step_mm)
    biggest = float(np.max(np.abs(delta)))
    if biggest > limit:
        delta *= limit / biggest
    out = []
    for i, axis in enumerate("xyz"):
        lo, hi = workspace[axis]
        out.append(float(min(hi, max(lo, float(tip_mm[i]) + delta[i]))))
    return out[0], out[1], out[2]


class NumpyPolicy:
    """Runs a trained SAC actor with numpy only (exported by training/sac/export_policy.py), so the GUI needs no PyTorch.
    File layout (.npz): W0,b0,W1,b1,... hidden layers (ReLU), then W_mu,b_mu; action = tanh(mu)  (SAC's deterministic action)."""

    def __init__(self, path: str):
        d = np.load(path, allow_pickle=False)
        version = int(d["obs_version"])
        if version != OBS_VERSION:
            raise ValueError(f"policy was trained for observation version {version}, this program builds version {OBS_VERSION} "
                             "-- retrain or use a matching policy")
        if int(d["obs_size"]) != OBS_SIZE:
            raise ValueError(f"policy expects {int(d['obs_size'])} inputs, this program builds {OBS_SIZE}")
        n = int(d["n_hidden"])
        self.hidden = [(d[f"W{i}"].astype(np.float32), d[f"b{i}"].astype(np.float32)) for i in range(n)]
        self.mu = (d["W_mu"].astype(np.float32), d["b_mu"].astype(np.float32))
        self.max_step_mm = float(d["max_step_mm"]) if "max_step_mm" in d else MAX_STEP_MM
        self.path = path

    def act(self, obs: np.ndarray) -> np.ndarray:
        h = np.asarray(obs, dtype=np.float32)
        for W, b in self.hidden:
            h = np.maximum(h @ W.T + b, 0.0)
        W, b = self.mu
        return np.tanh(h @ W.T + b)


class BerryMemory:
    """Short memory of recently seen berries (ROBOT coordinates).
    As the tip closes in, the effector hides the berry from the camera. A berry missing from a frame is kept at its last
    position for up to `max_age` frames with its confidence halved per missed frame, so the policy can finish the approach.
    Used IDENTICALLY by the simulator and the GUI -- that is what keeps training and the real robot consistent."""

    def __init__(self, match_radius_mm: float = 20.0, max_age: int = 8, decay: float = 0.5):
        self.match_radius_mm, self.max_age, self.decay = match_radius_mm, max_age, decay
        self.items: List[Dict] = []          # {"xyz_mm", "confidence", "depth_valid_fraction", "age"}

    def reset(self) -> None:
        self.items = []

    def update(self, seen: Iterable[Dict]) -> List[Dict]:
        fresh: List[Dict] = []
        used = set()
        for b in seen:
            p = b.get("xyz_mm")
            if p is None:
                continue
            p = [float(v) for v in p]
            best, best_d = None, self.match_radius_mm
            for i, m in enumerate(self.items):
                if i in used:
                    continue
                d = math.dist(m["xyz_mm"], p)
                if d <= best_d:
                    best, best_d = i, d
            if best is not None:
                used.add(best)
            fresh.append({"xyz_mm": p, "confidence": float(b.get("confidence", 0.0)),
                          "depth_valid_fraction": float(b.get("depth_valid_fraction", 1.0)), "age": 0})
        for i, m in enumerate(self.items):
            if i in used or m["age"] + 1 > self.max_age:
                continue
            if any(math.dist(m["xyz_mm"], f["xyz_mm"]) <= self.match_radius_mm for f in fresh):
                continue
            fresh.append({**m, "confidence": m["confidence"] * self.decay, "age": m["age"] + 1})
        self.items = fresh
        return [dict(m) for m in fresh]


class SacStepper:
    """Keeps the per-episode state (previous action, berry memory) and turns one frame's berries into one move target.
    Start a new episode (reset) whenever the run starts or a pick completes."""

    def __init__(self, policy: NumpyPolicy, workspace: Dict[str, Sequence[float]]):
        self.policy, self.workspace = policy, workspace
        self.prev_action = np.zeros(3, dtype=np.float32)
        self.memory = BerryMemory()

    def reset(self) -> None:
        self.prev_action[:] = 0.0
        self.memory.reset()

    def step(self, tip_mm: Sequence[float], berries: Iterable[Dict]) -> Optional[Triple]:
        """berries: this frame's usable detections in ROBOT coordinates. Returns the next target, or None when no berry
        is seen or remembered (the caller then waits or ends the attempt)."""
        known = self.memory.update(berries)
        if not known:
            return None
        obs = build_observation(tip_mm, known, self.prev_action, self.workspace)
        a = self.policy.act(obs)
        self.prev_action = a.astype(np.float32)
        return action_to_target(tip_mm, a, self.workspace, self.policy.max_step_mm, known)

"""
delta_pick_env.py -- gymnasium simulator for training the SAC picking policy.

What is simulated (and why):
  * The robot: the real DeltaKinematics, workspace box and arm-angle limits from the GUI's settings defaults. A step the real GUI
    would refuse (unreachable / joint limit) is refused here too: the tip stays put and the policy is penalised.
  * The camera: the stereo camera sits ~70-93 cm from the berries (user's measurement). Stereo depth error grows with distance
    squared: sigma_Z = Z^2 / (focal * baseline) * sigma_disparity, using the first real calibration (focal 726 px, baseline 62.2 mm).
    Sideways error is much smaller. The camera's pose is randomised per episode.
  * Calibration error: a constant camera->robot offset per episode (a few mm) that the policy cannot see -- it learns to tolerate it.
  * Detection: berries are missed sometimes, MUCH more often when the effector hides them (tip above and close),
    plus occasional false detections. Confidences and depth-quality numbers are randomised.
  * Plants: berries sway slightly; pushing down THROUGH a berry knocks it away and is penalised (a vacuum cup must come from above).

The observation is built by delta_gui.core.sac_policy.build_observation -- the SAME function the GUI uses on the real robot --
after the same BerryMemory. Success = tip within `success_radius_mm` of a TRUE berry position (hand over to the vacuum sequence).
All noise levels are starting guesses; replace them with measured values once the real camera is running (see README).
"""
import math
import os
import sys
from typing import Dict, List, Optional

import gymnasium as gym
import numpy as np
from gymnasium import spaces

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gui"))
from delta_gui.core.kinematics import DeltaKinematics, joint_limit_violation          # noqa: E402
from delta_gui.core.sac_policy import (MAX_STEP_MM, OBS_SIZE, BerryMemory, action_to_target,  # noqa: E402
                                       build_observation)
from delta_gui.core.settings import DEFAULTS                                         # noqa: E402

FOCAL_PX, BASELINE_MM = 726.3, 62.24


class DeltaPickEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, success_radius_mm: float = 10.0, max_steps: int = 80, noise_scale: float = 1.0,
                 settings: Optional[Dict] = None, knock_ends_episode: bool = True):
        super().__init__()
        s = settings or DEFAULTS
        g = s["geometry"]
        self.kin = DeltaKinematics(g["base_radius_mm"], g["effector_radius_mm"], g["bicep_mm"], g["forearm_mm"])
        self.ws = {k: [float(v) for v in s["workspace_mm"][k]] for k in "xyz"}
        self.theta = (float(s["axes"]["theta_min_deg"]), float(s["axes"]["theta_max_deg"]))
        self.success_radius_mm, self.max_steps, self.noise_scale = success_radius_mm, max_steps, noise_scale
        # v2: pushing through a berry is a FAILED attempt (episode over, -10), not just a small penalty. v1 (-2, carry on)
        # learned to cut corners diagonally and knocked berries in 2 % of runs (13 % at double noise).
        self.knock_ends_episode = knock_ends_episode
        self.observation_space = spaces.Box(-3.0, 3.0, (OBS_SIZE,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.memory = BerryMemory()

    # ------------------------------------------------------------------------------------------------ helpers
    def _reachable(self, p) -> bool:
        ang = self.kin.inverse(*p)
        return ang is not None and joint_limit_violation(ang, *self.theta) is None

    def _rand_in_ws(self, z_range=None) -> np.ndarray:
        r = self.np_random
        x = r.uniform(self.ws["x"][0] + 10, self.ws["x"][1] - 10)
        y = r.uniform(self.ws["y"][0] + 10, self.ws["y"][1] - 10)
        z = r.uniform(*(z_range or (self.ws["z"][0] + 5, self.ws["z"][1] - 5)))
        return np.array([x, y, z])

    def _min_dist(self) -> float:
        return float(min(np.linalg.norm(b - self.tip) for b in self.berries))

    def _observe(self) -> List[Dict]:
        """One camera frame: noisy berry positions in ROBOT coordinates (what the GUI computes from the Pi's J message)."""
        r, k = self.np_random, self.noise_scale
        out = []
        for b in self.berries:
            horiz = math.hypot(*(self.tip[:2] - b[:2]))
            hidden = horiz < 35.0 and self.tip[2] > b[2]               # effector between camera and berry
            p_det = 0.35 if hidden else 0.92
            if r.random() > p_det:
                continue
            ray = b - self.cam_pos
            dist = float(np.linalg.norm(ray))
            ray /= dist
            sigma_depth = dist * dist / (FOCAL_PX * BASELINE_MM) * 0.25 * k   # 0.25 px disparity noise
            sigma_side = dist * 0.5 / FOCAL_PX * k                             # 0.5 px detection-centre noise
            side = r.normal(0, sigma_side, 3)
            side -= ray * float(np.dot(side, ray))                             # sideways = perpendicular to the view ray
            noise = ray * r.normal(0, sigma_depth) + side
            # the camera sees the berry's FRONT surface, a little closer than its centre
            surface = -ray * r.uniform(5.0, 12.0)
            p = b + noise + surface + self.calib_bias
            out.append({"xyz_mm": p.tolist(), "confidence": float(r.uniform(0.5, 0.97)),
                        "depth_valid_fraction": float(r.uniform(0.4, 1.0))})
        if r.random() < 0.03 * k:                                       # a false detection (leaf, shadow...)
            out.append({"xyz_mm": self._rand_in_ws().tolist(), "confidence": float(r.uniform(0.5, 0.7)),
                        "depth_valid_fraction": float(r.uniform(0.2, 0.7))})
        return out

    def _obs(self) -> np.ndarray:
        self.known = self.memory.update(self._observe())
        return build_observation(self.tip, self.known, self.prev_action, self.ws)

    # ------------------------------------------------------------------------------------------------ gym API
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        r = self.np_random
        self.memory.reset()
        self.steps, self.prev_action = 0, np.zeros(3, dtype=np.float32)
        # camera in front of and above the workspace, ~70-93 cm from the berries, pose varied per episode
        self.cam_pos = np.array([r.uniform(-150, 150), r.uniform(-560, -440), r.uniform(250, 420)])
        self.calib_bias = r.normal(0, 3.0 * self.noise_scale, 3)
        n = int(r.integers(1, 5))
        self.berries, self.home = [], []
        while len(self.berries) < n:
            b = self._rand_in_ws(z_range=(self.ws["z"][0] + 15, self.ws["z"][0] + 80))   # berries hang low in the box
            if self._reachable(b) and all(np.linalg.norm(b - o) > 30 for o in self.berries):
                self.berries.append(b)
        self.home = [b.copy() for b in self.berries]
        while True:                                                     # start: usually high (ready pose), sometimes anywhere
            self.tip = self._rand_in_ws(z_range=(self.ws["z"][1] - 25, self.ws["z"][1] - 5)) if r.random() < 0.7 \
                else self._rand_in_ws()
            if self._reachable(self.tip) and self._min_dist() > 40:
                break
        self.prev_dist = self._min_dist()
        return self._obs(), {}

    def step(self, action):
        r = self.np_random
        a = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        self.steps += 1
        reward = -0.05 - 0.02 * float(np.dot(a, a))                    # time cost + a little for big jerky steps
        target = np.array(action_to_target(self.tip, a, self.ws, MAX_STEP_MM, self.known))   # same speed limit as the GUI
        blocked = not self._reachable(target)
        if blocked:
            reward -= 1.0                                              # the GUI would refuse this move
        else:
            self.tip = target
        info = {"blocked": blocked, "knocked": False}
        for i, b in enumerate(self.berries):                           # pushed down through a berry -> knocked away
            horiz = math.hypot(*(self.tip[:2] - b[:2]))
            if horiz < 12.0 and self.tip[2] < b[2] - 6.0:
                push = np.append((b[:2] - self.tip[:2]) / max(horiz, 1e-3), 0.0)
                self.berries[i] = b + push * 15.0
                reward -= 10.0 if self.knock_ends_episode else 2.0
                info["knocked"] = True
            else:                                                      # gentle sway around its stem
                self.berries[i] = self.home[i] + np.clip(b - self.home[i] + r.normal(0, 0.5, 3), -15, 15)
        d = self._min_dist()
        reward += (self.prev_dist - d) / 10.0                          # progress towards the nearest berry
        self.prev_dist = d
        success = d <= self.success_radius_mm and not (info["knocked"] and self.knock_ends_episode)
        if success:
            reward += 10.0
        info.update(success=success, dist_mm=d)
        self.prev_action = a
        terminated = success or (info["knocked"] and self.knock_ends_episode)
        return self._obs(), float(reward), bool(terminated), self.steps >= self.max_steps, info

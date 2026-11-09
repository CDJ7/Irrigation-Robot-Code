"""
harvest.py -- the automatic picking state machine.  Qt-free, so it is tested without a screen and without hardware.

What it does, in plain words:
  1. SEARCHING: watch the detections the Pi sends. A detection only counts if ALL of these hold:
        - its class is on OUR allowed list (default: "strawberry") AND the Pi also marked it allowed,
        - confidence and depth quality are high enough, and it has a real 3D position,
        - the camera->robot calibration is valid (so the position can be turned into a robot position).
     A strawberry must then be seen in several frames in a row at (nearly) the same robot position before it is trusted ("stable").
  2. Pick the stable strawberry closest to the robot's centre line and check that every pose we would use is allowed
     (workspace box, reachable, arm-angle limits). If not, skip it and say why.
  3. Pick sequence:  approach (above the berry) -> descend -> vacuum ON + dwell -> lift -> carry to the drop point ->
     vacuum OFF + wait.  Then look for the next berry. Every robot move goes through Robot.move_xyz, so ALL the normal safety checks
     (workspace box, IK, busy flag, position-valid) still apply -- this file adds NO way around them.
  4. Stops: max picks reached, nothing found for a while, camera silent, any robot move refused or not completed, the user's Stop
     (finishes the current move, then switches the vacuum off), or the E-STOP (immediate; Robot.estop does the stopping).

Honest limits: there is no pick verification (no vacuum-pressure sensor), so "picked" means "the sequence ran". The camera sees the
FRONT surface of the berry, not its centre: use grasp_offset_mm to correct. Vacuum commands need the Phase-7 firmware; with
harvest.vacuum_enabled = False the run is a DRY RUN (all moves, no vacuum).
"""
import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .robot import Robot
from .settings import Settings
from .transform import CameraToRobot

# states
IDLE, SEARCHING, RUNNING, STOPPING = "IDLE", "SEARCHING", "PICKING", "STOPPING"
DONE, STOPPED, ERROR = "DONE", "STOPPED", "ERROR"
ACTIVE = (SEARCHING, RUNNING, STOPPING)


@dataclass
class Track:
    """One strawberry followed from frame to frame (positions are ROBOT frame, mm)."""
    id: int
    positions: Deque[Tuple[float, float, float]]
    hits: int = 1                 # consecutive frames it was seen in
    last_seq: int = -1
    last_time: float = 0.0
    cls: str = ""
    conf: float = 0.0
    cam_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def mean(self) -> Tuple[float, float, float]:
        n = len(self.positions)
        return tuple(sum(p[i] for p in self.positions) / n for i in range(3))   # type: ignore[return-value]

    @property
    def jitter(self) -> float:
        m = self.mean
        return max(math.dist(p, m) for p in self.positions)


class Harvester:
    def __init__(self, robot: Robot, settings: Settings, get_calibration: Callable[[], CameraToRobot],
                 clock: Callable[[], float] = time.monotonic, log_path: Optional[str] = None):
        self.robot, self.settings, self.get_calibration, self.clock = robot, settings, get_calibration, clock
        self.log_path = log_path
        self.listeners: List[Callable[[str, dict], None]] = []
        self.state = IDLE
        self.message = "idle"
        self.picks_done = 0
        self.skipped = 0
        self.pick_log: List[dict] = []
        self.tracks: Dict[int, Track] = {}
        self._next_id = 1
        self._last_meta: Optional[dict] = None
        self._last_meta_time = -1e9
        self._last_seq = -1
        self._blocked_ids: set = set()
        self._recent_picks: List[Tuple[Tuple[float, float, float], float]] = []      # (robot xyz, time) exclusion zones
        self._search_since = 0.0
        self._steps: List[tuple] = []
        self._step_i = 0
        self._issued = False
        self._wait_until = 0.0
        self._current: Optional[dict] = None
        self._final: Optional[Tuple[str, str]] = None
        self._stop_requested = False
        self._stop_vac_tries = 0
        self.blocked_reason = ""                 # why detections are being ignored right now (shown in the GUI)
        robot.listeners.append(self._on_robot_event)

    # ------------------------------------------------------------------------------------------------ helpers
    def cfg(self) -> dict:
        return self.settings.data["harvest"]

    def _emit(self, event: str, **data) -> None:
        for fn in list(self.listeners):
            try:
                fn(event, data)
            except Exception:                     # noqa: BLE001  (a broken listener must never break the state machine)
                pass

    def log(self, text: str, level: str = "info") -> None:
        self._emit("log", text=text, level=level)

    def _set(self, state: str, message: str) -> None:
        self.state, self.message = state, message
        self._emit("state", state=state, message=message)

    @property
    def active(self) -> bool:
        return self.state in ACTIVE

    def _allowed_names(self) -> List[str]:
        return [c.strip().lower() for c in self.cfg()["allowed_classes"] if c.strip()]

    # ------------------------------------------------------------------------------------------------ preflight
    def preflight(self) -> List[Tuple[str, bool, str]]:
        """The checklist shown before a run: (what, ok, detail). start() refuses unless every line is ok."""
        r, c = self.robot, self.cfg()
        out: List[Tuple[str, bool, str]] = []

        def add(what: str, ok: bool, detail: str = "") -> None:
            out.append((what, bool(ok), detail))

        add("ESP32 connected", r.connected, "connect to the ESP32 (top bar)")
        add("E-STOP not active", not r.estopped, "release the E-STOP, enable the motors, home")
        add("Motors enabled", bool(r.drivers_enabled), "press Enable motors")
        add("Position known (homed)", r.xyz_valid, r.invalid_reason or "home the robot")
        add("Robot idle", not r.busy, "wait for the current move to finish")
        add("Home angles calibrated", bool(self.settings.data["home"]["calibrated"]), "Calibration page -> Home && axes")
        cal = self.get_calibration()
        add("Camera -> robot calibration valid", cal.valid, "Calibration page -> Camera -> robot")
        names = self._allowed_names()
        add("Allowed classes set", bool(names), "Harvest settings: allowed classes")
        meta = self._last_meta
        fresh = meta is not None and (self.clock() - self._last_meta_time) <= float(c["frame_timeout_s"])
        add("Camera sending frames", fresh, "connect the camera (Camera page)")
        if fresh:
            add("Stereo depth calibrated", bool(meta.get("calibrated")), "the Pi says depth is NOT calibrated")
            has = bool(meta.get("model_has_allowed_class"))
            pi_classes = [str(x).lower() for x in meta.get("allowed_classes", [])]
            overlap = (not pi_classes) or bool(set(pi_classes) & set(names))
            add("Detection model has a strawberry class", has and overlap,
                f"model '{meta.get('model', '?')}' has no allowed strawberry class -- picking refused (stock YOLO finds people, not berries)")
        else:
            add("Stereo depth calibrated", False, "no camera data")
            add("Detection model has a strawberry class", False, "no camera data")
        drop = c["drop_xyz_mm"]
        why = r.check_target(*[float(v) for v in drop])
        add("Drop point reachable and inside the workspace", why is None, why or "")
        return out

    def problems(self) -> List[str]:
        return [f"{w}: {d}" if d else w for w, ok, d in self.preflight() if not ok]

    # ------------------------------------------------------------------------------------------------ start / stop
    def start(self) -> List[str]:
        """Begin a run. Returns the list of problems; empty = started."""
        if self.active:
            return ["already running"]
        probs = self.problems()
        if probs:
            for p in probs:
                self.log("Harvest refused: " + p, "warn")
            return probs
        self.picks_done = self.skipped = 0
        self.tracks.clear(); self._blocked_ids.clear(); self._recent_picks.clear()
        self._final = None; self._stop_requested = False; self._steps = []; self._current = None
        self._search_since = self.clock()
        mode = "LIVE vacuum" if self.cfg()["vacuum_enabled"] else "DRY RUN (no vacuum)"
        self.log(f"Harvest started: {mode}, up to {self.cfg()['max_picks']} picks, classes {self._allowed_names()}.", "info")
        self._set(SEARCHING, "searching for a stable strawberry")
        return []

    def request_stop(self) -> None:
        """Soft stop: finish the move in progress, switch the vacuum off, end. (For an emergency use Robot.estop.)"""
        if self.active and not self._stop_requested:
            self._stop_requested = True
            self.log("Stop requested: finishing the current move, then vacuum off.", "warn")

    def _begin_stop(self, state: str, message: str, level: str = "warn") -> None:
        if self.state == STOPPING or not self.active:
            return
        self._final = (state, message)
        self.log(f"Harvest {state.lower()}: {message}", level)
        self._stop_vac_tries = 0
        self._set(STOPPING, message)

    def _on_robot_event(self, event: str, data: dict) -> None:
        # Runs immediately, not on the next tick: the E-STOP must end the run at once. Robot.estop() has already sent STOP (+ V 0).
        if event == "estop" and data.get("active") and self.active:
            self._steps = []
            self.log("Harvest halted by E-STOP.", "error")
            self._finish(STOPPED, "E-STOP pressed")
        elif event == "connection" and not data.get("connected") and self.active:
            self._finish(ERROR, "lost the connection to the ESP32")

    def _finish(self, state: str, message: str) -> None:
        if self.state in ACTIVE:
            if self._current is not None and self._current.get("result") is None:
                self._current["result"] = f"aborted: {message}"
                self._record(self._current); self._current = None
            self._set(state, message)
            self.log(f"Harvest finished: {state} ({message}); {self.picks_done} pick(s), {self.skipped} skipped.",
                     "info" if state == DONE else "warn")

    # ------------------------------------------------------------------------------------------------ camera input
    def on_detections(self, meta: dict) -> None:
        """Feed every J message here (from the GUI thread)."""
        self._last_meta, self._last_meta_time = meta, self.clock()
        if self.state != SEARCHING:
            return
        seq = meta.get("seq", -1)
        if isinstance(seq, int) and seq <= self._last_seq:
            return
        self._last_seq = seq if isinstance(seq, int) else self._last_seq
        self._update_tracks(meta)

    def _good(self, d: dict) -> bool:
        c = self.cfg()
        return (d.get("allowed") is True and str(d.get("class", "")).strip().lower() in self._allowed_names()
                and float(d.get("confidence", 0.0)) >= float(c["min_confidence"]) and d.get("xyz_mm") is not None
                and float(d.get("depth_valid_fraction", 1.0)) >= float(c["min_depth_valid_fraction"]))

    def _update_tracks(self, meta: dict) -> None:
        c, now = self.cfg(), self.clock()
        cal = self.get_calibration()
        self.blocked_reason = ""
        if not cal.valid:
            self.blocked_reason = "camera->robot calibration is not valid"
        elif not meta.get("calibrated"):
            self.blocked_reason = "stereo depth is not calibrated"
        elif not meta.get("model_has_allowed_class"):
            self.blocked_reason = "the model has no allowed strawberry class"
        if self.blocked_reason:
            self.tracks.clear()
            return
        seen: set = set()
        radius = float(c["match_radius_mm"])
        n = int(c["stable_frames"])
        for d in meta.get("detections", []):
            if not self._good(d):
                continue
            cam = tuple(float(v) for v in d["xyz_mm"])
            p = tuple(float(v) for v in cal.to_robot(cam))
            best, best_d = None, radius
            for t in self.tracks.values():
                if t.id in seen:
                    continue
                dist = math.dist(t.mean, p)
                if dist <= best_d:
                    best, best_d = t, dist
            if best is None:
                best = Track(self._next_id, deque(maxlen=n))
                self._next_id += 1
                self.tracks[best.id] = best
                best.hits = 0
            best.positions.append(p)
            best.hits += 1
            best.last_seq, best.last_time = meta.get("seq", -1), now
            best.cls, best.conf, best.cam_xyz = str(d.get("class")), float(d.get("confidence", 0.0)), cam
            seen.add(best.id)
        for tid in list(self.tracks):                      # missed this frame: its consecutive run is broken
            if tid not in seen:
                self.tracks[tid].hits = 0
                if now - self.tracks[tid].last_time > 1.5:
                    del self.tracks[tid]

    def stable_tracks(self) -> List[Track]:
        c = self.cfg()
        n, jit = int(c["stable_frames"]), float(c["max_jitter_mm"])
        return [t for t in self.tracks.values()
                if t.hits >= n and len(t.positions) >= n and t.jitter <= jit and t.id not in self._blocked_ids]

    # ------------------------------------------------------------------------------------------------ the poses of a pick
    def _poses(self, mean: Tuple[float, float, float]) -> Dict[str, List[float]]:
        c = self.cfg()
        off = [float(v) for v in c["grasp_offset_mm"]]
        pick = [mean[i] + off[i] for i in range(3)]
        above = [pick[0], pick[1], pick[2] + float(c["approach_height_mm"])]
        return {"approach": above, "pick": pick, "lift": list(above), "drop": [float(v) for v in c["drop_xyz_mm"]]}

    def _recently_tried(self, p: Tuple[float, float, float]) -> bool:
        now, radius = self.clock(), float(self.cfg()["match_radius_mm"])
        self._recent_picks = [(q, t) for q, t in self._recent_picks if now - t < 6.0]
        return any(math.dist(p, q) <= radius for q, _ in self._recent_picks)

    # ------------------------------------------------------------------------------------------------ the tick
    def tick(self) -> None:
        """Call about 10 times a second from the GUI thread."""
        if not self.active:
            return
        r, now = self.robot, self.clock()
        if r.estopped:
            self._finish(STOPPED, "E-STOP is active")
            return
        if not r.connected:
            self._finish(ERROR, "lost the connection to the ESP32")
            return
        if self.state == STOPPING:
            self._tick_stopping()
            return
        if self._stop_requested:
            if r.busy:
                return                                   # let the move in progress finish
            self._begin_stop(STOPPED, "stopped by the user")
            return
        if self.state == SEARCHING:
            self._tick_searching(now)
        elif self.state == RUNNING:
            self._tick_running(now)

    def _tick_searching(self, now: float) -> None:
        c, r = self.cfg(), self.robot
        if self.picks_done >= int(c["max_picks"]):
            self._finish(DONE, f"reached the limit of {c['max_picks']} picks")
            return
        if now - self._last_meta_time > float(c["frame_timeout_s"]):
            self._begin_stop(ERROR, "the camera stopped sending frames", "error")
            return
        if not r.xyz_valid or not r.drivers_enabled:
            self._begin_stop(ERROR, f"robot position/drivers not valid ({r.invalid_reason})", "error")
            return
        if r.busy:
            return
        cand = sorted(self.stable_tracks(), key=lambda t: math.hypot(t.mean[0], t.mean[1]))
        for t in cand:
            mean = t.mean
            if self._recently_tried(mean):
                continue
            poses = self._poses(mean)
            why = next((f"{k} pose {[round(v, 1) for v in p]}: {w}" for k, p in poses.items()
                        if (w := r.check_target(*p)) is not None), None)
            if why:
                self._blocked_ids.add(t.id)
                self.skipped += 1
                self.log(f"Strawberry #{t.id} skipped: {why}", "warn")
                continue
            self._start_pick(t, poses)
            return
        if now - self._search_since > float(c["idle_timeout_s"]):
            self._finish(DONE, f"no stable strawberry for {c['idle_timeout_s']:g} s")
        else:
            self.message = f"searching ({len(self.tracks)} candidate(s), {len(self.stable_tracks())} stable)"

    def _start_pick(self, t: Track, poses: Dict[str, List[float]]) -> None:
        n = self.picks_done + 1
        self._current = {"n": n, "track": t.id, "class": t.cls, "confidence": round(t.conf, 3),
                         "cam_xyz": [round(v, 1) for v in t.cam_xyz], "robot_xyz": [round(v, 1) for v in t.mean],
                         "pick_xyz": [round(v, 1) for v in poses["pick"]], "started": time.time(), "result": None}
        self._recent_picks.append((t.mean, self.clock()))
        self.log(f"Pick {n}: {t.cls} #{t.id} conf {t.conf:.2f} at robot XYZ "
                 f"({t.mean[0]:.1f}, {t.mean[1]:.1f}, {t.mean[2]:.1f}) mm, jitter {t.jitter:.1f} mm.", "info")
        self._steps = [("move", "approach", poses["approach"]), ("move", "descend", poses["pick"]),
                       ("vacuum", True), ("wait", self.cfg()["dwell_s"]),
                       ("move", "lift", poses["lift"]), ("move", "carry", poses["drop"]),
                       ("vacuum", False), ("wait", self.cfg()["release_s"])]
        self._step_i, self._issued = 0, False
        self.tracks.clear()
        self._set(RUNNING, f"pick {n}: starting")

    def _tick_running(self, now: float) -> None:
        r, c = self.robot, self.cfg()
        if r.busy:
            return
        if self._step_i >= len(self._steps):
            self._current["result"] = "completed (sequence ran; not verified)" if c["vacuum_enabled"] else "completed (DRY RUN, no vacuum)"
            self.picks_done += 1
            self._record(self._current); self._current = None
            self.tracks.clear()
            self._search_since = self.clock()
            self._set(SEARCHING, f"{self.picks_done} done; searching")
            return
        step = self._steps[self._step_i]
        kind = step[0]
        if kind == "move":
            label, target = step[1], step[2]
            if not self._issued:
                res = r.move_xyz(*target, source="harvest")
                if not res.ok:
                    self._fail(f"move '{label}' refused: {res.message}")
                    return
                self._issued = True
                self._set(RUNNING, f"pick {self._current['n']}: {label}")
                return
            if not r.xyz_valid or math.dist(r.current_xyz, target) > 0.5:
                self._fail(f"move '{label}' did not complete (position {r.invalid_reason or 'mismatch'})")
                return
        elif kind == "vacuum":
            want = step[1]
            if not c["vacuum_enabled"]:
                self.log(f"DRY RUN: vacuum {'ON' if want else 'OFF'} skipped.", "info")
            elif not self._issued:
                res = r.vacuum(want)
                if not res.ok:
                    self._fail(f"vacuum {'on' if want else 'off'} refused: {res.message}")
                    return
                self._issued = True
                self._set(RUNNING, f"pick {self._current['n']}: vacuum {'ON' if want else 'OFF'}")
                return
            elif r.vacuum_on != want:
                self._fail("the ESP32 did not confirm the vacuum command")
                return
        elif kind == "wait":
            if not self._issued:
                self._wait_until = now + float(step[1])
                self._issued = True
                return
            if now < self._wait_until:
                return
        self._step_i += 1
        self._issued = False

    def _fail(self, why: str) -> None:
        if self._current is not None:
            self._current["result"] = f"failed: {why}"
            self._record(self._current); self._current = None
        self._steps = []
        self._begin_stop(ERROR, why, "error")

    def _tick_stopping(self) -> None:
        """Vacuum OFF before ending, whatever the reason (the robot must be idle to take the command)."""
        r = self.robot
        if r.busy:
            return
        if self.cfg()["vacuum_enabled"] and (r.vacuum_on or self._stop_vac_tries == 0) and self._stop_vac_tries < 5:
            self._stop_vac_tries += 1
            res = r.vacuum(False)
            if res.ok:
                return
            self.log(f"Could not send vacuum OFF: {res.message}", "error")
            return
        if self.cfg()["vacuum_enabled"] and r.vacuum_on:
            self.log("WARNING: vacuum may still be ON. Use the E-STOP / switch it off by hand.", "error")
        state, msg = self._final or (STOPPED, "stopped")
        self._final = None
        if self._current is not None and self._current.get("result") is None:
            self._current["result"] = f"aborted: {msg}"; self._record(self._current); self._current = None
        self._set(state, msg)
        self.log(f"Harvest finished: {state} ({msg}); {self.picks_done} pick(s), {self.skipped} skipped.", "info" if state == DONE else "warn")

    # ------------------------------------------------------------------------------------------------ pick log
    def _record(self, entry: dict) -> None:
        entry["ended"] = time.time()
        self.pick_log.append(entry)
        self.log(f"Pick {entry['n']} {entry['result']}", "info" if str(entry["result"]).startswith("completed") else "warn")
        if self.log_path:
            try:
                with open(self.log_path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(entry) + "\n")
            except OSError:
                pass

"""
robot.py -- the robot controller: ALL motion safety lives here (not in the GUI), so it can be tested without a screen.

It keeps the original program's safety ideas and fixes the gaps found in the code review:

  kept      workspace box | IK "unreachable" check | busy flag | xyz_valid after raw moves | EMERGENCY STOP
  added     * ONE command in flight at a time, and every reply is matched to the command that asked for it. The firmware answers
              'OK' to both a finished move and an 'EN' command; matching replies to commands removes that confusion.
            * the position counts as UNKNOWN (xyz_valid False) until a home / zero, and again after: a stop, any error, an
              unexpected ESP32 reset, a lost link, a reply that never came (timeout), or the drivers being switched off.
            * moves are refused unless the drivers are enabled (the firmware would happily 'move' with them off)
            * arm-angle limits are checked as well as the workspace box
            * a move's duration also respects a maximum step rate (the firmware has no acceleration yet)
            * E-STOP is LATCHED: after it, nothing moves until the user releases it, and the position is unknown

Everything here runs on ONE thread (the GUI thread). The link's callbacks arrive on other threads; the GUI hands them over
with Qt signals before calling on_line() / on_link_state().
"""
import math
import re
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from .kinematics import (DeltaKinematics, Workspace, angles_to_steps, joint_limit_violation, steps_to_angles)
from .link import LinkBase
from .settings import Settings


@dataclass
class Result:
    ok: bool
    message: str = ""

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class Command:
    kind: str                      # "M" (kinematic move), "RAW", "H", "Z", "EN1", "EN0", "P"
    line: str
    deadline: float
    pending_steps: Optional[List[int]] = None
    pending_xyz: Optional[List[float]] = None


class Robot:
    def __init__(self, settings: Settings, link: Optional[LinkBase] = None, clock: Callable[[], float] = time.monotonic):
        self.settings = settings
        self.clock = clock
        self.listeners: List[Callable[[str, dict], None]] = []
        self.link: Optional[LinkBase] = None
        # --- state ---
        self.connected = False
        self.drivers_enabled: Optional[bool] = None      # None = not known yet
        self.xyz_valid = False
        self.invalid_reason = "not homed since connecting"
        self.current_steps: List[int] = [0, 0, 0]
        self.current_xyz: List[float] = [0.0, 0.0, 0.0]
        self.estopped = False
        self.inflight: Optional[Command] = None
        self.apply_settings()
        if link is not None:
            self.attach(link)

    # ---------------------------------------------------------------------------------------------- setup
    def attach(self, link: LinkBase) -> None:
        self.link = link

    def apply_settings(self) -> None:
        """(Re)build kinematics, workspace and conversion factors from the settings. Moving while this changes is refused."""
        s = self.settings.data
        g = s["geometry"]
        self.kin = DeltaKinematics(g["base_radius_mm"], g["effector_radius_mm"], g["bicep_mm"], g["forearm_mm"])
        w = s["workspace_mm"]
        self.workspace = Workspace(tuple(w["x"]), tuple(w["y"]), tuple(w["z"]))
        self.steps_per_deg = self.settings.steps_per_deg
        self.home_xyz = [float(v) for v in s["home"]["xyz_mm"]]
        if not self.xyz_valid and not self.connected:
            self.current_xyz = list(self.home_xyz)
        self._emit("state")

    def home_consistency(self) -> Optional[str]:
        """Does 'step 0' (where homing leaves the arms) agree with the configured HOME position? If not, a move 'to home'
        would actually move the robot. Returns a warning text, or None if consistent."""
        a = self.kin.inverse(*self.home_xyz)
        if a is None:
            return "the HOME position itself is unreachable with this geometry"
        s = self._steps_for(a)
        worst = max(abs(v) for v in s)
        if worst > 50:
            return (f"HOME and the axis offsets disagree: a move to the home XYZ would command up to {worst} steps "
                    f"({worst / self.steps_per_deg:.1f} deg) from step 0. Run the home calibration.")
        return None

    # ---------------------------------------------------------------------------------------------- events
    def _emit(self, event: str, **data) -> None:
        for fn in list(self.listeners):
            try:
                fn(event, data)
            except Exception:                     # noqa: BLE001  (a broken listener must never break safety code)
                pass

    def log(self, text: str, level: str = "info") -> None:
        self._emit("log", text=text, level=level)

    def snapshot(self) -> dict:
        return {"state": self.state_name, "connected": self.connected, "drivers_enabled": self.drivers_enabled,
                "xyz_valid": self.xyz_valid, "invalid_reason": self.invalid_reason, "steps": list(self.current_steps),
                "xyz": list(self.current_xyz), "estopped": self.estopped, "busy": self.busy,
                "busy_kind": self.inflight.kind if self.inflight else None}

    @property
    def busy(self) -> bool:
        return self.inflight is not None

    @property
    def state_name(self) -> str:
        if not self.connected:
            return "DISCONNECTED"
        if self.estopped:
            return "E-STOP"
        if self.inflight is not None:
            return "HOMING" if self.inflight.kind == "H" else "MOVING"
        if not self.xyz_valid:
            return "POSITION UNKNOWN"
        return "IDLE"

    def _invalidate(self, why: str) -> None:
        if self.xyz_valid:
            self.log(f"Position is no longer trusted: {why}. Home the robot before using XYZ moves.", "warn")
        self.xyz_valid, self.invalid_reason = False, why
        self._emit("state")

    # ---------------------------------------------------------------------------------------------- link events
    def on_link_state(self, connected: bool, message: str) -> None:
        self.log(message, "info" if connected else "warn")
        self.connected = connected
        self.inflight = None
        if connected:
            self.drivers_enabled = False        # the firmware boots with the drivers off
            self.estopped = False
            self.xyz_valid, self.invalid_reason = False, "not homed since connecting"
        else:
            self.drivers_enabled = None
            self._invalidate("the link to the ESP32 was lost")
        self._emit("connection", connected=connected)
        self._emit("state")

    def _send(self, line: str) -> bool:
        if self.link is None or not self.connected:
            self.log("Not connected to the ESP32.", "error")
            return False
        try:
            self.link.send_line(line)
        except ConnectionError as ex:
            self.log(f"Send failed: {ex}", "error")
            return False
        self._emit("log", text=line, level="tx")
        return True

    def _begin(self, cmd: Command) -> Result:
        if not self._send(cmd.line):
            return Result(False, "could not send (link problem)")
        self.inflight = cmd
        self._emit("state")
        return Result(True)

    # ---------------------------------------------------------------------------------------------- guards
    def _guard(self, need_enabled: bool = False, need_valid: bool = False) -> Optional[Result]:
        # Returns None when the action is ALLOWED, or a failed Result describing why it is refused.
        # CAREFUL: a failed Result is falsy, so always test the return value with `is not None`, never `if r:`.
        # (An earlier version of this file made exactly that mistake and silently ignored every refusal; the tests caught it.)
        if not self.connected:
            return Result(False, "not connected to the ESP32")
        if self.estopped:
            return Result(False, "E-STOP is active: release it, enable the motors and home first")
        if self.inflight is not None:
            return Result(False, f"robot is busy ({self.inflight.kind})")
        if need_enabled and not self.drivers_enabled:
            return Result(False, "enable the motors first")
        if need_valid and not self.xyz_valid:
            return Result(False, f"position unknown ({self.invalid_reason}): home the robot first")
        return None

    def _steps_for(self, angles: Sequence[float]) -> List[int]:
        a = self.settings.data["axes"]
        return list(angles_to_steps(angles, a["offsets_deg"], a["invert"], self.steps_per_deg))

    def check_target(self, x: float, y: float, z: float) -> Optional[str]:
        """Is this XYZ allowed? (workspace box, reachable, within arm-angle limits.) Returns the reason if not, else None."""
        if not all(math.isfinite(v) for v in (x, y, z)):
            return "target contains a non-numeric value"
        why = self.workspace.violation(x, y, z)
        if why:
            return why
        angles = self.kin.inverse(x, y, z)
        if angles is None:
            return f"({x:.1f}, {y:.1f}, {z:.1f}) is unreachable with this geometry"
        a = self.settings.data["axes"]
        why = joint_limit_violation(angles, a["theta_min_deg"], a["theta_max_deg"])
        return why

    # ---------------------------------------------------------------------------------------------- commands
    def move_xyz(self, x: float, y: float, z: float, source: str = "user") -> Result:
        r = self._guard(need_enabled=True, need_valid=True)
        if r is not None:
            self.log(f"Move refused: {r.message}", "warn")
            return r
        why = self.check_target(x, y, z)
        if why:
            self.log(f"Move to ({x:.1f}, {y:.1f}, {z:.1f}) refused: {why}", "warn")
            return Result(False, why)
        steps = self._steps_for(self.kin.inverse(x, y, z))
        d = self.settings.data["drive"]
        dist = math.dist(self.current_xyz, (x, y, z))
        dur_speed = dist / max(float(d["max_speed_mm_s"]), 1.0) * 1000.0
        dur_rate = max(abs(a - b) for a, b in zip(steps, self.current_steps)) / max(float(d["max_step_rate"]), 1.0) * 1000.0
        dur_ms = max(int(max(dur_speed, dur_rate)), 20)
        deadline = self.clock() + dur_ms / 1000.0 + float(self.settings.data["safety"]["reply_margin_s"])
        return self._begin(Command("M", f"M {steps[0]} {steps[1]} {steps[2]} {dur_ms}", deadline, steps, [x, y, z]))

    def jog(self, dx: float, dy: float, dz: float) -> Result:
        return self.move_xyz(self.current_xyz[0] + dx, self.current_xyz[1] + dy, self.current_xyz[2] + dz, "jog")

    def raw_move(self, steps: Sequence[int], duration_ms: int) -> Result:
        """Move the motors by step count, bypassing kinematics. The XYZ position is declared UNKNOWN first (as in the original)."""
        r = self._guard(need_enabled=True)
        if r is not None:
            self.log(f"Raw move refused: {r.message}", "warn")
            return r
        duration_ms = max(int(duration_ms), 1)
        s = [int(v) for v in steps]
        self._invalidate("a raw (manual) move was made")
        deadline = self.clock() + duration_ms / 1000.0 + float(self.settings.data["safety"]["reply_margin_s"])
        return self._begin(Command("RAW", f"M {s[0]} {s[1]} {s[2]} {duration_ms}", deadline, s))

    def raw_move_size_deg(self, steps: Sequence[int]) -> float:
        """Largest single-axis change (degrees) a raw move would make -- the GUI asks for confirmation above a threshold."""
        return max(abs(a - b) for a, b in zip(steps, self.current_steps)) / self.steps_per_deg

    def home(self) -> Result:
        r = self._guard(need_enabled=True)
        if r is not None:
            self.log(f"Home refused: {r.message}", "warn")
            return r
        timeout = float(self.settings.data["safety"]["home_timeout_s"])
        return self._begin(Command("H", "H", self.clock() + timeout))

    def set_zero(self) -> Result:
        """'Z': declare the CURRENT physical pose to be home (use only after checking it by eye)."""
        r = self._guard()
        if r is not None:
            return r
        return self._begin(Command("Z", "Z", self.clock() + 3.0))

    def enable(self, on: bool) -> Result:
        r = self._guard()
        if r is not None:
            self.log(f"Enable/disable refused: {r.message}", "warn")
            return r
        return self._begin(Command("EN1" if on else "EN0", "EN 1" if on else "EN 0", self.clock() + 3.0))

    def query_position(self) -> Result:
        r = self._guard()
        if r is not None:
            return r
        return self._begin(Command("P", "P?", self.clock() + 3.0))

    def estop(self) -> None:
        """EMERGENCY STOP. Always acts, whatever the state. STOP is sent at once (it is the one command the firmware
        also honours while homing), then -- if the setting says so -- the drivers are switched off. The position is unknown
        afterwards and the stop is LATCHED until release_estop()."""
        self.estopped = True
        self.inflight = None                        # whatever was pending no longer matters; late replies are just logged
        self.log("EMERGENCY STOP", "error")
        if self.connected:
            self._send("STOP")
            if self.settings.data["safety"]["estop_disables_drivers"]:
                self._send("EN 0")
                self.drivers_enabled = False
        self.xyz_valid, self.invalid_reason = False, "emergency stop"
        self._emit("estop", active=True)
        self._emit("state")

    def release_estop(self) -> None:
        if self.estopped:
            self.estopped = False
            self.log("E-STOP released. Enable the motors and home before moving.", "warn")
            self._emit("estop", active=False)
            self._emit("state")

    def recover_position_from_steps(self) -> Result:
        """After a stop, compute XYZ from the motor step counts (forward kinematics) instead of re-homing. Only meaningful if
        the home calibration is done (the axis offsets must be right) -- hence the explicit settings flag."""
        if not self.settings.data["home"]["calibrated"]:
            return Result(False, "home is not calibrated yet, so the step counts cannot be converted to a position")
        r = self._guard()
        if r is not None:
            return r
        a = self.settings.data["axes"]
        angles = steps_to_angles(self.current_steps, a["offsets_deg"], a["invert"], self.steps_per_deg)
        p = self.kin.forward(*angles)
        if p is None:
            return Result(False, "the step counts do not correspond to any reachable position")
        self.current_xyz, self.xyz_valid, self.invalid_reason = list(p), True, ""
        self.log(f"Position recovered from step counts: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})", "info")
        self._emit("state")
        return Result(True)

    # ---------------------------------------------------------------------------------------------- replies
    _POS = re.compile(r"POS\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)")

    def _apply_pos(self, text: str) -> bool:
        m = self._POS.fullmatch(text.strip())
        if not m:
            return False
        self.current_steps = [int(m.group(i)) for i in (1, 2, 3)]
        self._emit("state")
        return True

    def _apply_home(self) -> None:
        self.current_steps = [0, 0, 0]
        self.current_xyz = list(self.home_xyz)
        self.xyz_valid, self.invalid_reason = True, ""
        warn = self.home_consistency()
        self.log("Robot homed. XYZ tracking is valid." + (f"  WARNING: {warn}" if warn else ""),
                 "warn" if warn else "info")

    def on_line(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        self._emit("log", text=text, level="rx")
        if text.startswith("READY"):
            # The firmware prints READY when it boots. Right after connecting that is normal; at any other time it means the
            # ESP32 reset by itself (brown-out, watchdog) and has forgotten where the arms are.
            unexpected = self.inflight is not None or self.xyz_valid or self.drivers_enabled
            self.drivers_enabled = False
            self.inflight = None
            if unexpected:
                self.log("The ESP32 restarted unexpectedly (brown-out or reset). Position lost.", "error")
                self._invalidate("the ESP32 restarted")
            else:
                self._emit("state")
            return
        cmd = self.inflight
        if cmd is None:
            if not self._apply_pos(text) and text.startswith("ERR"):
                self.log(f"ESP32 reported: {text}", "warn")
            return
        if cmd.kind in ("M", "RAW"):
            if text == "OK":
                self.current_steps = list(cmd.pending_steps)
                if cmd.kind == "M":
                    self.current_xyz = list(cmd.pending_xyz)
                self._done()
            elif text.startswith("ERR"):
                self._failed(cmd, text)
            else:
                self._apply_pos(text)
        elif cmd.kind in ("H", "Z"):
            if text == "HOMED":
                self._apply_home()
                self._done()
            elif text.startswith("ERR"):
                self._failed(cmd, text)
            else:
                self._apply_pos(text)
        elif cmd.kind in ("EN1", "EN0"):
            if text == "OK":
                self.drivers_enabled = cmd.kind == "EN1"
                if cmd.kind == "EN0":
                    self._invalidate("the motors were disabled (the arms are no longer held)")
                self._done()
            elif text.startswith("ERR"):
                self._failed(cmd, text)
        elif cmd.kind == "P":
            if self._apply_pos(text):
                self._done()
            elif text.startswith("ERR"):
                self._failed(cmd, text)

    def _done(self) -> None:
        self.inflight = None
        self._emit("state")

    def _failed(self, cmd: Command, text: str) -> None:
        self.inflight = None
        if "BAD_M_ARGS" in text:
            self.log(f"ESP32 rejected the command ({text}); nothing moved.", "warn")
        elif "STOPPED" in text:
            self.log("Move aborted by STOP.", "warn")
            self._invalidate("the move was stopped part-way")
        elif "BUSY" in text:
            self.log("ESP32 says BUSY although the GUI thought it was idle: out of sync.", "error")
            self._invalidate("out of sync with the ESP32 (it reported BUSY)")
        else:
            self.log(f"ESP32 error: {text}", "error")
            self._invalidate(f"the ESP32 reported {text}")
        self._emit("state")

    def tick(self) -> None:
        """Call a few times per second: gives up on a command whose reply never came."""
        cmd = self.inflight
        if cmd is not None and self.clock() > cmd.deadline:
            self.log(f"No reply to '{cmd.line}' in time. Treating the position as unknown.", "error")
            self.inflight = None
            self._invalidate(f"no reply to '{cmd.line}'")
            self._emit("state")

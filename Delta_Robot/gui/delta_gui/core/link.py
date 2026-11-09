"""
link.py -- the connection to the ESP32.

  SerialLink          the real thing (USB serial, via pyserial)
  SimulatedESP32Link  a fake ESP32 that behaves like the firmware, INCLUDING its quirks, for testing without hardware

Both have the same interface, so the rest of the program does not know which one it talks to:
    link.on_line  = f(text)               called (from a background thread!) for every line the ESP32 sends
    link.on_state = f(connected, message) called when the link goes up or down
    link.connect(target, baud)            returns immediately; the result arrives through on_state
    link.send_line("M 1 2 3 500")
    link.disconnect()
No Qt in here: the GUI hands the callbacks to Qt signals so everything reaches the screen on the GUI thread.
"""
import re
import threading
import time
from typing import Callable, List, Optional


class LinkBase:
    def __init__(self) -> None:
        self.on_line: Optional[Callable[[str], None]] = None
        self.on_state: Optional[Callable[[bool, str], None]] = None

    def connect(self, target: str, baud: int = 115200) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError

    def is_connected(self) -> bool:
        raise NotImplementedError

    def send_line(self, text: str) -> None:
        raise NotImplementedError

    def _state(self, connected: bool, message: str) -> None:
        if self.on_state:
            self.on_state(connected, message)

    def _line(self, text: str) -> None:
        if self.on_line:
            self.on_line(text)


# =====================================================================================================================
class SerialLink(LinkBase):
    """USB serial to the real ESP32. `target` is a port name such as COM5 (or a pyserial URL such as loop://)."""

    RESET_WAIT_S = 2.0      # opening the port resets most ESP32 boards; give the firmware time to boot

    def __init__(self, reset_wait_s: Optional[float] = None) -> None:
        super().__init__()
        self._ser = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._wlock = threading.Lock()
        self._gen = 0                       # connect attempts are numbered so a stale one can never "win"
        self.reset_wait_s = self.RESET_WAIT_S if reset_wait_s is None else reset_wait_s

    def connect(self, target: str, baud: int = 115200) -> None:
        self.disconnect()
        self._gen += 1
        threading.Thread(target=self._open, args=(target, baud, self._gen), daemon=True, name="serial-open").start()

    def _open(self, target: str, baud: int, gen: int) -> None:
        import serial   # imported here so the rest of the program (and the tests) work without pyserial installed
        try:
            ser = serial.serial_for_url(target, baudrate=baud, timeout=0.1, write_timeout=1.0)
        except Exception as ex:             # noqa: BLE001  (any open failure is reported to the user)
            if gen == self._gen:
                self._state(False, f"cannot open {target}: {ex}")
            return
        end = time.monotonic() + self.reset_wait_s
        while time.monotonic() < end:       # wait for the board to boot, but give up promptly if cancelled
            if gen != self._gen:
                ser.close()
                return
            time.sleep(0.02)
        if gen != self._gen:
            ser.close()
            return
        self._ser = ser
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, args=(ser,), daemon=True, name="serial-read")
        self._thread.start()
        self._state(True, f"connected to {target} at {baud} baud")

    def disconnect(self) -> None:
        self._gen += 1                       # cancels an open that is still waiting
        self._stop.set()
        ser, self._ser = self._ser, None
        t, self._thread = self._thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.0)
        if ser is not None:
            try:
                ser.close()
            except Exception:                # noqa: BLE001
                pass

    def is_connected(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def send_line(self, text: str) -> None:
        ser = self._ser
        if ser is None or not getattr(ser, "is_open", False):
            raise ConnectionError("not connected to the ESP32")
        try:
            with self._wlock:
                ser.write((text.strip() + "\n").encode("utf-8"))
        except Exception as ex:              # noqa: BLE001
            self._lost(f"write failed: {ex}")
            raise ConnectionError(str(ex))

    def _lost(self, why: str) -> None:
        if self._ser is not None:
            self._ser = None
            self._stop.set()
            self._state(False, f"serial link lost ({why})")

    def _read_loop(self, ser) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except Exception as ex:          # noqa: BLE001  (unplugged cable, driver error, ...)
                self._lost(str(ex))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", errors="replace").strip()
                if text:
                    self._line(text)


# =====================================================================================================================
class SimulatedESP32Link(LinkBase):
    """A fake ESP32 that answers like Delta_Robot_ESP32.ino -- including its quirks, so the GUI is tested against reality:
       * 'OK' answers BOTH a finished move and an 'EN' command
       * 'Z' is accepted while moving, and answers HOMED
       * homing blocks: while it runs, every command except STOP is silently thrown away
       * a move is accepted even when the drivers are disabled (it just 'moves' the counters)
    time_scale > 1 makes everything run faster (for tests)."""

    def __init__(self, time_scale: float = 1.0, home_seconds: float = 2.0, boot_delay: float = 0.05) -> None:
        super().__init__()
        self.time_scale, self.home_seconds, self.boot_delay = time_scale, home_seconds, boot_delay
        self._lock = threading.RLock()
        self._connected = False
        self.steps: List[int] = [0, 0, 0]
        self.enabled = False
        self._timers: List[threading.Timer] = []
        self._move = None                    # (t_start, duration_s, from_steps, to_steps)
        self._homing = False
        self.sent: List[str] = []            # every line received (for tests)
        self.drop_replies = 0                # test hook: swallow this many replies

    # ---------------------------------------------------------------- link interface
    def connect(self, target: str = "SIM", baud: int = 115200) -> None:
        def up() -> None:
            with self._lock:
                self._connected = True
            self._state(True, "simulated ESP32 (no hardware)")
            self._reply("READY DeltaRobotSlave")
        self._after(self.boot_delay, up)

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False
            self._cancel_timers()
            self._move, self._homing = None, False

    def is_connected(self) -> bool:
        return self._connected

    def send_line(self, text: str) -> None:
        if not self._connected:
            raise ConnectionError("not connected to the ESP32")
        line = text.strip()
        with self._lock:
            self.sent.append(line)
            self._after(0.0, lambda: self._handle(line))   # processed 'on the other side', like a real serial port

    # ---------------------------------------------------------------- test hooks
    def inject_reset(self) -> None:
        """Pretend the ESP32 browned out and rebooted: counters lost, drivers off, READY printed."""
        with self._lock:
            self._cancel_timers()
            self._move, self._homing = None, False
            self.steps, self.enabled = [0, 0, 0], False
        self._reply("READY DeltaRobotSlave")

    def inject_link_loss(self) -> None:
        with self._lock:
            self._connected = False
        self._state(False, "serial link lost (simulated cable pull)")

    # ---------------------------------------------------------------- internals
    def _after(self, seconds: float, fn: Callable[[], None]) -> None:
        t = threading.Timer(max(0.0, seconds / self.time_scale), fn)
        t.daemon = True
        with self._lock:
            self._timers = [x for x in self._timers if x.is_alive()] + [t]
        t.start()

    def _cancel_timers(self) -> None:
        for t in self._timers:
            t.cancel()
        self._timers = []

    def _reply(self, text: str) -> None:
        with self._lock:
            if self.drop_replies > 0:
                self.drop_replies -= 1
                return
        self._line(text)

    def _pos(self) -> None:
        self._reply("POS %d %d %d" % tuple(self.steps))

    def _handle(self, line: str) -> None:
        with self._lock:
            if not self._connected:
                return
            if self._homing and line != "STOP":
                return                        # blocking homing loop: everything but STOP is discarded (firmware quirk)
            if line.startswith("M "):
                m = re.fullmatch(r"M\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(\d+)", line)
                if not m:
                    return self._reply("ERR BAD_M_ARGS")
                if self._move:
                    return self._reply("ERR BUSY")
                target = [int(m.group(i)) for i in (1, 2, 3)]
                dur = max(int(m.group(4)), 1) / 1000.0
                if target == self.steps:
                    return self._reply("OK")
                self._move = (time.monotonic(), dur, list(self.steps), target)
                self._after(dur, self._finish_move)
            elif line == "H":
                if self._move:
                    return self._reply("ERR BUSY")
                self._homing = True
                self._after(self.home_seconds, self._finish_home)
            elif line == "Z":
                self.steps = [0, 0, 0]
                self._reply("HOMED")
            elif line.startswith("EN"):
                self.enabled = "1" in line
                self._reply("OK")
            elif line == "STOP":
                if self._homing:
                    self._cancel_timers()
                    self._homing = False
                    return self._reply("ERR HOMING_ABORTED")
                if self._move:
                    t0, dur, a, b = self._move
                    frac = min(1.0, max(0.0, (time.monotonic() - t0) * self.time_scale / dur))
                    self.steps = [round(x + (y - x) * frac) for x, y in zip(a, b)]
                    self._move = None
                    self._cancel_timers()
                    self._reply("ERR STOPPED")
                self._pos()
            elif line == "P?":
                self._pos()
            elif line:
                self._reply("ERR UNKNOWN_CMD " + line)

    def _finish_move(self) -> None:
        with self._lock:
            if not self._move:
                return
            self.steps = list(self._move[3])
            self._move = None
        self._reply("OK")

    def _finish_home(self) -> None:
        with self._lock:
            if not self._homing:
                return
            self._homing = False
            self.steps = [0, 0, 0]
        self._reply("HOMED")
        self._pos()

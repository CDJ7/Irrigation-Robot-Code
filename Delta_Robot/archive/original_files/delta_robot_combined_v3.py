"""
Delta Robot - Combined Master Controller & Debug GUI (v3)
-----------------------------------------------------------
Tabs: Main Operations (XYZ + live random camera sim), Debug & Calibration
(raw per-axis steps), Depth Map (simulated depth map + auto POI tour),
and Live Camera (NEW -- real video/depth stream from a Raspberry Pi).

WHAT CHANGED IN v3:
  1. Removed AXIS_STEP_LIMITS / the Axis-2 dead zone. Those were specific
     to the faulty gearboxes -- now fixed, so hard-coded step ceilings
     from that era would just be wrong/obsolete. Reachability is instead
     bounded by a WORKSPACE BOX defined in real mm from your measured
     working envelope (24in W x 27in L x 24in H), checked against the
     XYZ target BEFORE kinematics ever runs. Raw/debug moves are no
     longer step-limited at all -- test the (now-fixed) gearboxes fresh
     and add new limits here if you determine you need them.
  2. DeltaKinematics defaults updated to your real geometry: rf (bicep,
     motor-to-elbow) = 10in = 254mm, re (forearm, elbow-to-effector) =
     13.5in = 342.9mm, e (effector radius) ~= 3in = 76.2mm.
     f (base radius, center to motor shaft) is NOT derivable from what
     you gave me -- it's still the old placeholder value. MEASURE IT
     and update the "Base f (mm)" field before trusting XYZ moves.
  3. New "Live Camera" tab: connects to a small TCP server you run on
     the Raspberry Pi (see pi_camera_server.py) and displays the live
     video feed + live depth map, with a crosshair overlay showing
     where the end effector is currently commanded to be, projected
     onto the video using the workspace box as the pixel<->mm mapping.
     This tab needs `opencv-python` and `pillow` installed on the
     COMPUTER running this GUI (not required for the other tabs).
"""

import math
import time
import threading
import queue
import random
import tkinter as tk
from tkinter import ttk, messagebox
from typing import List, Optional, Tuple, Dict

import serial
import serial.tools.list_ports

# Optional dependencies for the Live Camera tab only. The rest of the
# app works fine without these -- the tab just disables itself with a
# clear message if they're missing.
try:
    from PIL import Image, ImageTk, ImageDraw
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False

try:
    from camera_client import CameraClient
    _CAMERA_CLIENT_AVAILABLE = True
except ImportError:
    _CAMERA_CLIENT_AVAILABLE = False

# Kinematics constants
SQRT3 = math.sqrt(3.0)
SIN120 = SQRT3 / 2.0
COS120 = -0.5
TAN60 = SQRT3
TAN30 = 1.0 / SQRT3

INCH_TO_MM = 25.4

# =========================================================
#   WORKSPACE BOX -- real measured working envelope
# =========================================================
# From your measurements: 24in wide (X), 27in long (Y), 24in tall (Z).
# X/Y are assumed centered on the robot's origin. Z's upper edge
# (closest to the base, least negative) defaults to -50mm -- ADJUST this
# once you know where your effector actually sits at the top of its
# useful range; the box's bottom is simply top - height.
# All of this is also editable live from the Geometry panel in the GUI.
DEFAULT_WORKSPACE_WIDTH_IN = 24.0   # X
DEFAULT_WORKSPACE_LENGTH_IN = 27.0  # Y
DEFAULT_WORKSPACE_HEIGHT_IN = 24.0  # Z
DEFAULT_WORKSPACE_Z_TOP_MM = -50.0


class DeltaKinematics:
    def __init__(self, f: float = 100.0, e: float = 76.2, rf: float = 254.0, re: float = 342.9):
        # Defaults: rf=10in bicep, re=13.5in forearm, e~=3in effector radius
        # (all converted to mm). f (base radius) is UNKNOWN -- still the
        # old placeholder. Measure it and set it via the Geometry panel.
        self.f = f
        self.e = e
        self.rf = rf
        self.re = re

    def _calc_angle_yz(self, x0: float, y0: float, z0: float) -> Optional[float]:
        f, e, rf, re = self.f, self.e, self.rf, self.re
        y1 = -0.5 * TAN30 * f
        y0 = y0 - 0.5 * TAN30 * e

        a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
        b = (y1 - y0) / z0

        d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
        if d < 0:
            return None

        yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
        zj = a + b * yj
        theta = math.degrees(math.atan2(-zj, y1 - yj))
        return theta

    def inverse(self, x0: float, y0: float, z0: float) -> Optional[Tuple[float, float, float]]:
        t1 = self._calc_angle_yz(x0, y0, z0)
        if t1 is None:
            return None
        t2 = self._calc_angle_yz(x0 * COS120 + y0 * SIN120, y0 * COS120 - x0 * SIN120, z0)
        if t2 is None:
            return None
        t3 = self._calc_angle_yz(x0 * COS120 - y0 * SIN120, y0 * COS120 + x0 * SIN120, z0)
        if t3 is None:
            return None
        return t1, t2, t3


class SerialLink:
    def __init__(self, rx_queue: queue.Queue):
        self.ser: Optional[serial.Serial] = None
        self.rx_queue = rx_queue
        self._stop_flag = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def connect(self, port: str, baud: int = 115200) -> None:
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(2.0)
        self._stop_flag.clear()
        thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread = thread
        thread.start()

    def disconnect(self) -> None:
        self._stop_flag.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send_line(self, text: str) -> None:
        ser = self.ser
        if ser is None or not ser.is_open:
            raise ConnectionError("Not connected to ESP32")
        line = text.strip() + "\n"
        ser.write(line.encode("utf-8"))
        self.rx_queue.put(("TX", line.strip()))

    def _read_loop(self) -> None:
        buf = b""
        while not self._stop_flag.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", errors="replace").strip()
                        if text:
                            self.rx_queue.put(("RX", text))
                else:
                    time.sleep(0.02)
            except (serial.SerialException, OSError):
                self.rx_queue.put(("RX", "[ERROR] Serial link lost"))
                break


class CameraVisionThread(threading.Thread):
    """Existing live 'random point' simulator, unchanged -- separate from
    the new Depth Map / points-of-interest feature below."""

    def __init__(self, target_queue: queue.Queue):
        super().__init__(daemon=True)
        self.target_queue = target_queue
        self.running = False

    def start_tracking(self):
        self.running = True

    def stop_tracking(self):
        self.running = False

    def run(self):
        while True:
            if self.running:
                simulated_x = random.uniform(-40.0, 40.0)
                simulated_y = random.uniform(-40.0, 40.0)
                simulated_z = -200.0
                self.target_queue.put((simulated_x, simulated_y, simulated_z))
                time.sleep(1.0)
            else:
                time.sleep(0.1)


# =========================================================
#   DEPTH MAP: simulation + point-of-interest detection
# =========================================================
def generate_simulated_depth_map(width: int = 64, height: int = 64, num_blobs: int = 4,
                                  background: float = 0.0, blob_value: float = 1.0,
                                  blob_radius_range: Tuple[int, int] = (3, 8)) -> List[List[float]]:
    """Stand-in for a real depth camera frame: a flat background with a
    few circular 'objects' of elevated value, each with a soft falloff
    from center to edge (roughly like a real depth blob)."""
    depth = [[background for _ in range(width)] for _ in range(height)]
    lo_r, hi_r = blob_radius_range
    for _ in range(num_blobs):
        r = random.randint(lo_r, hi_r)
        cx = random.randint(r, max(r, width - r - 1))
        cy = random.randint(r, max(r, height - r - 1))
        for yy in range(max(0, cy - r), min(height, cy + r + 1)):
            for xx in range(max(0, cx - r), min(width, cx + r + 1)):
                dist = math.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
                if dist <= r:
                    val = blob_value * (1.0 - dist / (r + 1))
                    if val > depth[yy][xx]:
                        depth[yy][xx] = val
    return depth


def find_points_of_interest(depth_map: List[List[float]], threshold: float,
                             min_region_size: int) -> List[Dict]:
    """Flood-fill connected-component search for regions of the depth map
    at or above `threshold`. Returns one dict per region with its pixel
    centroid (row, col) and average depth, for regions with at least
    `min_region_size` pixels (filters out single-pixel noise)."""
    height = len(depth_map)
    width = len(depth_map[0]) if height else 0
    visited = [[False] * width for _ in range(height)]
    results: List[Dict] = []

    for y in range(height):
        for x in range(width):
            if visited[y][x] or depth_map[y][x] < threshold:
                continue
            stack = [(x, y)]
            visited[y][x] = True
            region = []
            while stack:
                cx, cy = stack.pop()
                region.append((cx, cy, depth_map[cy][cx]))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if (0 <= nx < width and 0 <= ny < height
                            and not visited[ny][nx] and depth_map[ny][nx] >= threshold):
                        visited[ny][nx] = True
                        stack.append((nx, ny))
            if len(region) >= min_region_size:
                n = len(region)
                sum_x = sum(p[0] for p in region)
                sum_y = sum(p[1] for p in region)
                sum_d = sum(p[2] for p in region)
                results.append({
                    "row": sum_y / n,
                    "col": sum_x / n,
                    "depth": sum_d / n,
                    "size": n,
                })
    return results


class DeltaRobotApp:
    # Placeholder home position -- MEASURE where your effector actually
    # sits after homing/backoff on the real machine and update this.
    HOME_XYZ = (0.0, 0.0, -200.0)

    def __init__(self, master_window: tk.Tk):
        self.root = master_window
        self.root.title("Delta Robot Master Controller")
        self.root.geometry("980x900")

        self.rx_queue: queue.Queue = queue.Queue()
        self.link = SerialLink(self.rx_queue)
        self.kin = DeltaKinematics()

        # State Variables
        self.current_steps: List[int] = [0, 0, 0]
        self.current_xyz: List[float] = list(self.HOME_XYZ)
        self.xyz_valid: bool = True

        self.busy: bool = False
        self.pending_mode: Optional[str] = None
        self.pending_steps: Optional[List[int]] = None
        self.pending_xyz: Optional[List[float]] = None

        # Camera state (existing live random simulator)
        self.camera_target_queue: queue.Queue = queue.Queue()
        self.vision_thread = CameraVisionThread(self.camera_target_queue)
        self.vision_thread.start()
        self.tracking_mode = False

        # Depth Map / POI tour state
        self.depth_map: Optional[List[List[float]]] = None
        self.depth_map_w: int = 0
        self.depth_map_h: int = 0
        self.poi_list: List[Dict] = []
        self.poi_tour_queue: List[Dict] = []
        self.tour_active: bool = False
        self.current_tour_poi: Optional[Dict] = None

        # Workspace box (real measured envelope, in mm) -- computed from
        # the default inch values below; editable live via _apply_workspace.
        self.workspace_x = (
            -DEFAULT_WORKSPACE_WIDTH_IN * INCH_TO_MM / 2.0,
            DEFAULT_WORKSPACE_WIDTH_IN * INCH_TO_MM / 2.0,
        )
        self.workspace_y = (
            -DEFAULT_WORKSPACE_LENGTH_IN * INCH_TO_MM / 2.0,
            DEFAULT_WORKSPACE_LENGTH_IN * INCH_TO_MM / 2.0,
        )
        self.workspace_z = (
            DEFAULT_WORKSPACE_Z_TOP_MM - DEFAULT_WORKSPACE_HEIGHT_IN * INCH_TO_MM,
            DEFAULT_WORKSPACE_Z_TOP_MM,
        )

        # Live Camera (Raspberry Pi) state
        self.cam_client: Optional["CameraClient"] = None
        self.cam_frame_queue: queue.Queue = queue.Queue()
        self.cam_video_photo = None   # keep a reference so Tkinter doesn't GC it
        self.cam_depth_photo = None
        self.cam_show_overlay = True

        self._build_ui()
        self._poll_serial()
        self._poll_camera()
        if _CAMERA_CLIENT_AVAILABLE and _PIL_AVAILABLE:
            self._poll_live_camera()

    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # --- Top: Connection frame ---
        conn = ttk.LabelFrame(self.root, text="Connection")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self.port_combo = ttk.Combobox(conn, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, **pad)
        self._refresh_ports()

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, **pad)
        ttk.Label(conn, text="Baud:").grid(row=0, column=3, **pad)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, **pad)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=5, **pad)
        self.status_lbl = ttk.Label(conn, text="Disconnected", foreground="red")
        self.status_lbl.grid(row=0, column=6, **pad)
        self.motion_lbl = ttk.Label(conn, text="Idle", foreground="gray")
        self.motion_lbl.grid(row=0, column=7, **pad)

        # --- Middle: Notebook (Tabs) ---
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, **pad)

        self.tab_main = ttk.Frame(self.notebook)
        self.tab_debug = ttk.Frame(self.notebook)
        self.tab_depth = ttk.Frame(self.notebook)
        self.tab_live_camera = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text="Main Operations")
        self.notebook.add(self.tab_debug, text="Debug & Calibration")
        self.notebook.add(self.tab_depth, text="Depth Map")
        self.notebook.add(self.tab_live_camera, text="Live Camera (Pi)")

        self._build_main_tab(pad)
        self._build_debug_tab(pad)
        self._build_depth_tab(pad)
        self._build_live_camera_tab(pad)

        # --- Bottom: Global Actions & Log ---
        act = ttk.Frame(self.root)
        act.pack(fill="x", **pad)
        ttk.Button(act, text="Enable Motors", command=lambda: self._send("EN 1")).pack(side="left", **pad)
        ttk.Button(act, text="Disable Motors", command=lambda: self._send("EN 0")).pack(side="left", **pad)
        ttk.Button(act, text="EMERGENCY STOP", command=self._stop).pack(side="left", **pad)
        ttk.Button(act, text="Query Position", command=lambda: self._send("P?")).pack(side="left", **pad)

        log_frame = ttk.LabelFrame(self.root, text="System Log")
        log_frame.pack(fill="both", expand=True, **pad)
        self.log_text = tk.Text(log_frame, height=10, state="disabled", wrap="none")
        self.log_text.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)

        self._apply_geometry()

    def _build_main_tab(self, pad):
        # Geometry
        geo = ttk.LabelFrame(self.tab_main, text="Robot Geometry / Kinematics")
        geo.pack(fill="x", **pad)

        self.geo_vars = {}
        fields = [
            ("Base f (mm) -- MEASURE THIS", "f", 100.0), ("Effector e (mm)", "e", 76.2),
            ("Bicep rf (mm)", "rf", 254.0), ("Forearm re (mm)", "re", 342.9),
            ("Steps/rev", "steps_per_rev", 3200.0), ("Max speed (mm/s)", "max_speed", 80.0),
        ]
        for i, (label, key, default) in enumerate(fields):
            ttk.Label(geo, text=label).grid(row=i // 3, column=(i % 3) * 2, sticky="e", **pad)
            v = tk.StringVar(value=str(default))
            ttk.Entry(geo, textvariable=v, width=8).grid(row=i // 3, column=(i % 3) * 2 + 1, **pad)
            self.geo_vars[key] = v
        ttk.Button(geo, text="Apply Geo", command=self._apply_geometry).grid(row=2, column=0, columnspan=2, **pad)

        ttk.Label(geo, text="'f' (base radius) could not be derived from your measurements -- "
                             "measure the real distance from center to each motor shaft.",
                  foreground="#b35c00", wraplength=820, justify="left").grid(
            row=6, column=0, columnspan=6, sticky="w", **pad)

        self.axis_offset = [tk.StringVar(value="0.0") for _ in range(3)]
        self.axis_invert = [tk.BooleanVar(value=False) for _ in range(3)]
        for i in range(3):
            ttk.Label(geo, text=f"Offset {i+1} (deg)").grid(row=3, column=i * 2, **pad)
            ttk.Entry(geo, textvariable=self.axis_offset[i], width=6).grid(row=3, column=i * 2 + 1, **pad)
            ttk.Checkbutton(geo, text=f"Invert {i+1}", variable=self.axis_invert[i]).grid(
                row=4, column=i * 2, columnspan=2, **pad)

        ws = ttk.LabelFrame(self.tab_main, text="Workspace Box (real measured working envelope)")
        ws.pack(fill="x", **pad)

        self.ws_width_var = tk.StringVar(value=str(DEFAULT_WORKSPACE_WIDTH_IN))
        self.ws_length_var = tk.StringVar(value=str(DEFAULT_WORKSPACE_LENGTH_IN))
        self.ws_height_var = tk.StringVar(value=str(DEFAULT_WORKSPACE_HEIGHT_IN))
        self.ws_ztop_var = tk.StringVar(value=str(DEFAULT_WORKSPACE_Z_TOP_MM))

        ttk.Label(ws, text="Width (X, in)").grid(row=0, column=0, **pad)
        ttk.Entry(ws, textvariable=self.ws_width_var, width=8).grid(row=0, column=1, **pad)
        ttk.Label(ws, text="Length (Y, in)").grid(row=0, column=2, **pad)
        ttk.Entry(ws, textvariable=self.ws_length_var, width=8).grid(row=0, column=3, **pad)
        ttk.Label(ws, text="Height (Z, in)").grid(row=0, column=4, **pad)
        ttk.Entry(ws, textvariable=self.ws_height_var, width=8).grid(row=0, column=5, **pad)
        ttk.Label(ws, text="Z top (mm)").grid(row=0, column=6, **pad)
        ttk.Entry(ws, textvariable=self.ws_ztop_var, width=8).grid(row=0, column=7, **pad)
        ttk.Button(ws, text="Apply Workspace", command=self._apply_workspace).grid(row=0, column=8, **pad)

        ttk.Label(ws, text="X/Y assumed centered on the robot's origin. Z top is the least-negative "
                            "(closest to base) edge of the box; the bottom is Z top minus Height. "
                            "Any XYZ move outside this box is rejected before kinematics even runs.",
                  foreground="#555", wraplength=850, justify="left").grid(
            row=1, column=0, columnspan=9, sticky="w", **pad)

        self.ws_bounds_lbl = ttk.Label(ws, text="", foreground="#555")
        self.ws_bounds_lbl.grid(row=2, column=0, columnspan=9, sticky="w", **pad)
        self._update_workspace_label()

        # Position control
        pos = ttk.LabelFrame(self.tab_main, text="XYZ Position Control")
        pos.pack(fill="x", **pad)

        self.xyz_vars = [tk.StringVar(value=str(v)) for v in self.current_xyz]
        for i, axis in enumerate(["X", "Y", "Z"]):
            ttk.Label(pos, text=axis).grid(row=0, column=i * 2, **pad)
            ttk.Entry(pos, textvariable=self.xyz_vars[i], width=10).grid(row=0, column=i * 2 + 1, **pad)

        ttk.Button(pos, text="Move to XYZ", command=self._move_to_entry_position).grid(row=0, column=6, **pad)
        ttk.Button(pos, text="HOME ALL", command=self._home).grid(row=0, column=7, **pad)

        # Jog controls
        jog = ttk.Frame(pos)
        jog.grid(row=1, column=0, columnspan=8, pady=6)
        ttk.Label(jog, text="Jog step (mm):").grid(row=0, column=0, **pad)
        self.jog_step = tk.StringVar(value="5.0")
        ttk.Entry(jog, textvariable=self.jog_step, width=6).grid(row=0, column=1, **pad)

        jog_btns = [("X-", (-1, 0, 0)), ("X+", (1, 0, 0)), ("Y-", (0, -1, 0)),
                    ("Y+", (0, 1, 0)), ("Z-", (0, 0, -1)), ("Z+", (0, 0, 1))]
        for i, (label, delta) in enumerate(jog_btns):
            ttk.Button(jog, text=label, width=4, command=lambda d=delta: self._jog(d)).grid(
                row=0, column=2 + i, **pad)

        # Camera integration (existing live random simulator)
        cam = ttk.LabelFrame(self.tab_main, text="Camera / Vision Tracking Integration (live random sim)")
        cam.pack(fill="x", **pad)
        ttk.Label(cam, text="Listens for automated XYZ coordinates from the vision thread.",
                  foreground="blue").pack(anchor="w", **pad)
        self.cam_btn = ttk.Button(cam, text="Start Camera Tracking", command=self._toggle_camera_tracking)
        self.cam_btn.pack(side="left", **pad)
        self.cam_status_lbl = ttk.Label(cam, text="Camera Offline", foreground="gray")
        self.cam_status_lbl.pack(side="left", **pad)

    def _build_debug_tab(self, pad):
        setup = ttk.LabelFrame(self.tab_debug, text="\u26a0 Manual Axis Setup (Bypasses Kinematics) \u26a0")
        setup.pack(fill="both", expand=True, **pad)

        warn = ttk.Label(
            setup,
            text=("Moves each motor directly by step count. Using this invalidates XYZ tracking.\n"
                  "You MUST Home the robot afterwards before returning to the Main Tab.\n"
                  "Raw targets are NOT checked against the workspace box (that check only applies "
                  "to kinematic XYZ moves) -- be careful, especially with the fixed gearboxes "
                  "now allowing full travel that wasn't reachable before."),
            foreground="#b35c00", justify="left"
        )
        warn.grid(row=0, column=0, columnspan=8, sticky="w", **pad)

        ttk.Label(setup, text="Absolute target steps:").grid(row=1, column=0, sticky="e", **pad)
        self.raw_step_vars = [tk.StringVar(value=str(s)) for s in self.current_steps]
        for i in range(3):
            ttk.Label(setup, text=f"Axis {i+1}").grid(row=1, column=1 + i * 2, **pad)
            ttk.Entry(setup, textvariable=self.raw_step_vars[i], width=10).grid(row=1, column=2 + i * 2, **pad)

        ttk.Label(setup, text="Duration (ms):").grid(row=2, column=0, sticky="e", **pad)
        self.raw_duration_var = tk.StringVar(value="800")
        ttk.Entry(setup, textvariable=self.raw_duration_var, width=8).grid(row=2, column=1, **pad)
        ttk.Button(setup, text="Send Raw Move", command=self._raw_move_from_entries).grid(
            row=2, column=2, columnspan=2, **pad)

        ttk.Label(setup, text="Jog steps:").grid(row=3, column=0, sticky="e", **pad)
        self.raw_jog_step = tk.StringVar(value="100")
        ttk.Entry(setup, textvariable=self.raw_jog_step, width=8).grid(row=3, column=1, **pad)

        for i in range(3):
            ttk.Button(setup, text=f"A{i+1} -", width=5,
                       command=lambda ax=i: self._raw_jog_axis(ax, -1)).grid(row=3, column=2 + i * 2, **pad)
            ttk.Button(setup, text=f"A{i+1} +", width=5,
                       command=lambda ax=i: self._raw_jog_axis(ax, 1)).grid(row=3, column=3 + i * 2, **pad)

        ttk.Button(setup, text="Set Current Position as Home (Zero ESP32)",
                   command=lambda: self._send("Z")).grid(row=4, column=0, columnspan=4,
                                                          padx=pad["padx"], pady=20)
        ttk.Label(setup, text="Note: requires the ESP32 sketch's 'Z' command "
                              "(see the updated firmware) to actually zero its counters.",
                  foreground="#555").grid(row=5, column=0, columnspan=6, sticky="w", **pad)

    def _build_depth_tab(self, pad):
        top = ttk.LabelFrame(self.tab_depth, text="Depth Map Source (simulated -- swap in a real camera feed later)")
        top.pack(fill="x", **pad)

        ttk.Label(top, text="Width").grid(row=0, column=0, **pad)
        self.depth_map_w_var = tk.StringVar(value="64")
        ttk.Entry(top, textvariable=self.depth_map_w_var, width=6).grid(row=0, column=1, **pad)

        ttk.Label(top, text="Height").grid(row=0, column=2, **pad)
        self.depth_map_h_var = tk.StringVar(value="64")
        ttk.Entry(top, textvariable=self.depth_map_h_var, width=6).grid(row=0, column=3, **pad)

        ttk.Label(top, text="Blobs").grid(row=0, column=4, **pad)
        self.depth_num_blobs_var = tk.StringVar(value="4")
        ttk.Entry(top, textvariable=self.depth_num_blobs_var, width=6).grid(row=0, column=5, **pad)

        ttk.Button(top, text="Generate Simulated Depth Map", command=self._on_generate_depth_map).grid(
            row=0, column=6, **pad)

        self.depth_map_status_lbl = ttk.Label(top, text="No depth map loaded.", foreground="gray")
        self.depth_map_status_lbl.grid(row=1, column=0, columnspan=7, sticky="w", **pad)

        scan = ttk.LabelFrame(self.tab_depth, text="Scan for Points of Interest")
        scan.pack(fill="x", **pad)

        ttk.Label(scan, text="Threshold").grid(row=0, column=0, **pad)
        self.depth_threshold_var = tk.StringVar(value="0.5")
        ttk.Entry(scan, textvariable=self.depth_threshold_var, width=8).grid(row=0, column=1, **pad)

        ttk.Label(scan, text="Min region size (px)").grid(row=0, column=2, **pad)
        self.depth_min_region_var = tk.StringVar(value="5")
        ttk.Entry(scan, textvariable=self.depth_min_region_var, width=8).grid(row=0, column=3, **pad)

        ttk.Button(scan, text="Scan Depth Map for POIs", command=self._on_scan_depth_map).grid(
            row=0, column=4, **pad)

        ttk.Label(scan, text="Pixel/depth -> robot mm mapping: robot = (pixel - center) * scale + offset",
                  foreground="#555").grid(row=1, column=0, columnspan=6, sticky="w", **pad)

        self.depth_scale = [tk.StringVar(value="1.0") for _ in range(3)]
        self.depth_offset = [tk.StringVar(value="0.0") for _ in range(3)]
        for i, axis in enumerate(["X", "Y", "Z(depth)"]):
            ttk.Label(scan, text=f"{axis} scale").grid(row=2, column=i * 2, **pad)
            ttk.Entry(scan, textvariable=self.depth_scale[i], width=8).grid(row=2, column=i * 2 + 1, **pad)
        for i, axis in enumerate(["X", "Y", "Z(depth)"]):
            ttk.Label(scan, text=f"{axis} offset").grid(row=3, column=i * 2, **pad)
            ttk.Entry(scan, textvariable=self.depth_offset[i], width=8).grid(row=3, column=i * 2 + 1, **pad)

        tour = ttk.LabelFrame(self.tab_depth, text="Automatic Tour (closest-to-home first)")
        tour.pack(fill="x", **pad)
        ttk.Button(tour, text="Start Moving to POIs", command=self._start_poi_tour).pack(side="left", **pad)
        ttk.Button(tour, text="Stop Tour", command=self._stop_poi_tour).pack(side="left", **pad)
        ttk.Button(tour, text="Reset Visited Flags", command=self._reset_poi_visited).pack(side="left", **pad)
        self.tour_status_lbl = ttk.Label(tour, text="Tour idle.", foreground="gray")
        self.tour_status_lbl.pack(side="left", **pad)

        list_frame = ttk.LabelFrame(self.tab_depth, text="Points of Interest")
        list_frame.pack(fill="both", expand=True, **pad)
        columns = ("idx", "row", "col", "depth", "x", "y", "z", "visited")
        self.poi_tree = ttk.Treeview(list_frame, columns=columns, show="headings", height=10)
        headings = {"idx": "#", "row": "Row(px)", "col": "Col(px)", "depth": "Depth",
                    "x": "X(mm)", "y": "Y(mm)", "z": "Z(mm)", "visited": "Visited"}
        for col in columns:
            self.poi_tree.heading(col, text=headings[col])
            self.poi_tree.column(col, width=90, anchor="center")
        self.poi_tree.pack(fill="both", expand=True, side="left")
        poi_scroll = ttk.Scrollbar(list_frame, command=self.poi_tree.yview)
        poi_scroll.pack(side="right", fill="y")
        self.poi_tree.config(yscrollcommand=poi_scroll.set)

    def _build_live_camera_tab(self, pad):
        if not (_CAMERA_CLIENT_AVAILABLE and _PIL_AVAILABLE):
            missing = []
            if not _PIL_AVAILABLE:
                missing.append("pillow")
            if not _CAMERA_CLIENT_AVAILABLE:
                missing.append("camera_client.py (must be in the same folder as this script)")
            ttk.Label(
                self.tab_live_camera,
                text=("Live Camera tab is disabled -- missing: " + ", ".join(missing) + ".\n\n"
                      "Install with: pip install pillow opencv-python\n"
                      "and make sure camera_client.py is next to this file, then restart."),
                foreground="#b35c00", justify="left", wraplength=800,
            ).pack(padx=20, pady=20, anchor="w")
            return

        conn = ttk.LabelFrame(self.tab_live_camera, text="Raspberry Pi Camera Server Connection")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="Pi IP address:").grid(row=0, column=0, **pad)
        self.cam_ip_var = tk.StringVar(value="192.168.1.50")
        ttk.Entry(conn, textvariable=self.cam_ip_var, width=16).grid(row=0, column=1, **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=2, **pad)
        self.cam_port_var = tk.StringVar(value="8485")
        ttk.Entry(conn, textvariable=self.cam_port_var, width=8).grid(row=0, column=3, **pad)

        self.cam_connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_camera_connection)
        self.cam_connect_btn.grid(row=0, column=4, **pad)

        self.cam_conn_status_lbl = ttk.Label(conn, text="Not connected", foreground="gray")
        self.cam_conn_status_lbl.grid(row=0, column=5, **pad)

        self.cam_overlay_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(conn, text="Show target-position overlay", variable=self.cam_overlay_var,
                        command=lambda: setattr(self, "cam_show_overlay", self.cam_overlay_var.get())
                        ).grid(row=0, column=6, **pad)

        ttk.Label(conn, text="Run pi_camera_server.py on the Raspberry Pi first (see comments in "
                              "that file for setup). This GUI is only the client -- it can't launch "
                              "code on the Pi itself over the network.",
                  foreground="#555", wraplength=850, justify="left").grid(
            row=1, column=0, columnspan=7, sticky="w", **pad)

        panels = ttk.Frame(self.tab_live_camera)
        panels.pack(fill="both", expand=True, **pad)

        video_frame = ttk.LabelFrame(panels, text="Live Video (with target overlay)")
        video_frame.pack(side="left", fill="both", expand=True, **pad)
        self.cam_video_label = ttk.Label(video_frame, text="No video yet", anchor="center")
        self.cam_video_label.pack(fill="both", expand=True)

        depth_frame = ttk.LabelFrame(panels, text="Live Depth Map")
        depth_frame.pack(side="left", fill="both", expand=True, **pad)
        self.cam_depth_label = ttk.Label(depth_frame, text="No depth map yet", anchor="center")
        self.cam_depth_label.pack(fill="both", expand=True)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _toggle_connect(self):
        if self.link.is_connected():
            self.link.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_lbl.config(text="Disconnected", foreground="red")
        else:
            port = self.port_combo.get()
            if not port:
                messagebox.showerror("Error", "Select a COM port first.")
                return
            try:
                self.link.connect(port, int(self.baud_var.get()))
                self.connect_btn.config(text="Disconnect")
                self.status_lbl.config(text="Connected", foreground="green")
            except Exception as ex:
                messagebox.showerror("Connection failed", str(ex))

    def _send(self, text: str):
        try:
            self.link.send_line(text)
        except ConnectionError as ex:
            if not self.tracking_mode and not self.tour_active:  # suppress popup spam during automation
                messagebox.showerror("Not connected", str(ex))

    def _poll_serial(self):
        while not self.rx_queue.empty():
            direction, text = self.rx_queue.get_nowait()
            self._log(f"{direction}: {text}")
            if direction == "RX":
                self._handle_rx(text)
        self.root.after(50, self._poll_serial)

    def _handle_rx(self, text: str):
        if text == "OK":
            was_tour_move = (self.pending_mode == "move" and self.tour_active
                              and self.current_tour_poi is not None)
            if self.pending_mode == "move" and self.pending_steps is not None:
                self.current_steps = self.pending_steps
                self.current_xyz = self.pending_xyz
                for i, v in enumerate(self.raw_step_vars):
                    v.set(str(self.current_steps[i]))
            elif self.pending_mode == "raw" and self.pending_steps is not None:
                self.current_steps = self.pending_steps
                for i, v in enumerate(self.raw_step_vars):
                    v.set(str(self.current_steps[i]))
            self._clear_pending()

            if was_tour_move:
                poi = self.current_tour_poi
                self.current_tour_poi = None
                self._mark_poi_visited(poi, skipped=False)
                self._log(f"[TOUR] Reached POI #{poi['idx']} at "
                          f"({poi['xyz'][0]:.1f}, {poi['xyz'][1]:.1f}, {poi['xyz'][2]:.1f}) -- marked done.")
                self._advance_tour()

        elif text == "HOMED":
            self.current_steps = [0, 0, 0]
            self.current_xyz = list(self.HOME_XYZ)
            for v in self.raw_step_vars:
                v.set("0")
            for i in range(3):
                self.xyz_vars[i].set(f"{self.HOME_XYZ[i]:.2f}")
            self.xyz_valid = True
            self._log("[INFO] Robot has successfully homed. XYZ Tracking Valid.")
            self._clear_pending()

        elif text == "ZEROED":
            # Manual "Set Current Position as Home" from the Debug tab.
            # We trust the user visually verified alignment, so treat it
            # like a real home for XYZ-tracking purposes.
            self.current_steps = [0, 0, 0]
            self.current_xyz = list(self.HOME_XYZ)
            for v in self.raw_step_vars:
                v.set("0")
            for i in range(3):
                self.xyz_vars[i].set(f"{self.HOME_XYZ[i]:.2f}")
            self.xyz_valid = True
            self._log("[INFO] Position manually zeroed and accepted as home. XYZ Tracking Valid.")
            self._clear_pending()

        elif text.startswith("ERR"):
            self._clear_pending()
            if "BUSY" in text:
                pass  # Expected occasionally, ignore quietly
            elif "STOPPED" in text:
                self._log("[WARN] Move aborted by STOP.")
            else:
                self._log(f"[WARN] ESP32 error: {text}")
            if self.tour_active:
                self.tour_active = False
                self.current_tour_poi = None
                self.tour_status_lbl.config(text="Tour halted (ESP32 error).", foreground="#b35c00")
                self._log("[TOUR] Halted due to an ESP32 error. Fix the issue and restart the tour.")

        elif text.startswith("POS"):
            parts = text.split()
            if len(parts) == 4:
                try:
                    self.current_steps = [int(parts[1]), int(parts[2]), int(parts[3])]
                    for i, v in enumerate(self.raw_step_vars):
                        v.set(str(self.current_steps[i]))
                except ValueError:
                    pass

    def _clear_pending(self):
        self.busy = False
        self.pending_mode = None
        self.pending_steps = None
        self.pending_xyz = None
        self.motion_lbl.config(text="Idle", foreground="gray")

    def _log(self, text: str):
        self.log_text.config(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _apply_geometry(self):
        try:
            f = float(self.geo_vars["f"].get())
            e = float(self.geo_vars["e"].get())
            rf = float(self.geo_vars["rf"].get())
            re = float(self.geo_vars["re"].get())
            self.kin = DeltaKinematics(f=f, e=e, rf=rf, re=re)
            self.steps_per_rev: float = float(self.geo_vars["steps_per_rev"].get())
            self.steps_per_deg: float = self.steps_per_rev / 360.0
            self.max_speed: float = float(self.geo_vars["max_speed"].get())
            self._log("[INFO] Geometry updated.")
        except ValueError:
            messagebox.showerror("Error", "Geometry fields must be numeric.")

    def _poll_camera(self):
        while not self.camera_target_queue.empty():
            target_x, target_y, target_z = self.camera_target_queue.get_nowait()
            if self.tracking_mode and not self.busy:
                self._log(f"[VISION] Target acquired: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                self.xyz_vars[0].set(f"{target_x:.2f}")
                self.xyz_vars[1].set(f"{target_y:.2f}")
                self.xyz_vars[2].set(f"{target_z:.2f}")
                self._move_to_xyz(target_x, target_y, target_z)
        self.root.after(100, self._poll_camera)

    def _toggle_camera_tracking(self):
        if not self.xyz_valid:
            messagebox.showerror("Error", "Cannot start camera tracking: XYZ position is invalid. Run Home first.")
            return

        self.tracking_mode = not self.tracking_mode
        if self.tracking_mode:
            self.cam_btn.config(text="Stop Camera Tracking")
            self.cam_status_lbl.config(text="Camera Tracking ENABLED", foreground="green")
            self.vision_thread.start_tracking()
            self._log("[INFO] Camera Tracking ENABLED.")
        else:
            self.cam_btn.config(text="Start Camera Tracking")
            self.cam_status_lbl.config(text="Camera Offline", foreground="gray")
            self.vision_thread.stop_tracking()
            self._log("[INFO] Camera Tracking DISABLED.")

    def _angles_to_steps(self, angles: Tuple[float, float, float]) -> List[int]:
        steps: List[int] = []
        for i, theta in enumerate(angles):
            offset = float(self.axis_offset[i].get())
            corrected = theta + offset
            if self.axis_invert[i].get():
                corrected = -corrected
            steps.append(int(round(corrected * self.steps_per_deg)))
        return steps

    # ---------------------------------------------------
    # WORKSPACE BOX (real measured envelope)
    # ---------------------------------------------------
    def _apply_workspace(self):
        try:
            width_in = float(self.ws_width_var.get())
            length_in = float(self.ws_length_var.get())
            height_in = float(self.ws_height_var.get())
            z_top = float(self.ws_ztop_var.get())
        except ValueError:
            messagebox.showerror("Error", "Workspace fields must be numeric.")
            return
        width_mm = width_in * INCH_TO_MM
        length_mm = length_in * INCH_TO_MM
        height_mm = height_in * INCH_TO_MM
        self.workspace_x = (-width_mm / 2.0, width_mm / 2.0)
        self.workspace_y = (-length_mm / 2.0, length_mm / 2.0)
        self.workspace_z = (z_top - height_mm, z_top)
        self._update_workspace_label()
        self._log("[INFO] Workspace box updated.")

    def _update_workspace_label(self):
        self.ws_bounds_lbl.config(
            text=f"Current box (mm): X [{self.workspace_x[0]:.1f}, {self.workspace_x[1]:.1f}]  "
                 f"Y [{self.workspace_y[0]:.1f}, {self.workspace_y[1]:.1f}]  "
                 f"Z [{self.workspace_z[0]:.1f}, {self.workspace_z[1]:.1f}]")

    def _workspace_violation(self, x: float, y: float, z: float) -> Optional[str]:
        if not (self.workspace_x[0] <= x <= self.workspace_x[1]):
            return f"X={x:.1f} is outside workspace box [{self.workspace_x[0]:.1f}, {self.workspace_x[1]:.1f}]"
        if not (self.workspace_y[0] <= y <= self.workspace_y[1]):
            return f"Y={y:.1f} is outside workspace box [{self.workspace_y[0]:.1f}, {self.workspace_y[1]:.1f}]"
        if not (self.workspace_z[0] <= z <= self.workspace_z[1]):
            return f"Z={z:.1f} is outside workspace box [{self.workspace_z[0]:.1f}, {self.workspace_z[1]:.1f}]"
        return None

    def _reject_move(self, msg: str) -> None:
        self._log(f"[WARN] {msg}")
        if not self.tracking_mode and not self.tour_active:
            messagebox.showerror("Move rejected", msg)

    # ---------------------------------------------------
    # MOTION COMMANDS (kinematic)
    # ---------------------------------------------------
    def _move_to_xyz(self, x: float, y: float, z: float) -> bool:
        if self.busy:
            return False

        if not self.xyz_valid:
            messagebox.showerror(
                "XYZ Tracking Invalid",
                "A raw/manual axis move was used. XYZ coordinates are no longer trustworthy. "
                "Run HOME to reset the physical location before using XYZ moves again."
            )
            return False

        violation = self._workspace_violation(x, y, z)
        if violation:
            self._reject_move(f"Move to ({x:.1f}, {y:.1f}, {z:.1f}) rejected -- {violation}")
            return False

        angles = self.kin.inverse(x, y, z)
        if angles is None:
            self._log(f"[WARN] Target ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable.")
            return False

        target_steps = self._angles_to_steps(angles)

        dx = x - self.current_xyz[0]
        dy = y - self.current_xyz[1]
        dz = z - self.current_xyz[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = max(self.max_speed, 1.0)
        dur_ms = max(int((dist / speed) * 1000.0), 20)

        cmd = f"M {target_steps[0]} {target_steps[1]} {target_steps[2]} {dur_ms}"
        self.busy = True
        self.pending_mode = "move"
        self.pending_steps = target_steps
        self.pending_xyz = [x, y, z]
        self.motion_lbl.config(text="Moving...", foreground="orange")
        self._send(cmd)
        return True

    def _move_to_entry_position(self):
        try:
            x, y, z = float(self.xyz_vars[0].get()), float(self.xyz_vars[1].get()), float(self.xyz_vars[2].get())
            self._move_to_xyz(x, y, z)
        except ValueError:
            messagebox.showerror("Error", "X/Y/Z must be numeric.")

    def _jog(self, delta: Tuple[int, int, int]) -> None:
        if self.busy:
            return
        try:
            step = float(self.jog_step.get())
            x = self.current_xyz[0] + delta[0] * step
            y = self.current_xyz[1] + delta[1] * step
            z = self.current_xyz[2] + delta[2] * step
            self.xyz_vars[0].set(f"{x:.2f}")
            self.xyz_vars[1].set(f"{y:.2f}")
            self.xyz_vars[2].set(f"{z:.2f}")
            self._move_to_xyz(x, y, z)
        except ValueError:
            pass

    # ---------------------------------------------------
    # MOTION COMMANDS (raw / debug)
    # ---------------------------------------------------
    def _raw_move(self, target_steps: List[int], duration_ms: int) -> bool:
        if self.busy:
            return False

        self.xyz_valid = False  # Invalidate kinematic tracking!

        self.busy = True
        self.pending_mode = "raw"
        self.pending_steps = list(target_steps)
        self.motion_lbl.config(text="Moving (raw)...", foreground="#b35c00")

        cmd = f"M {target_steps[0]} {target_steps[1]} {target_steps[2]} {duration_ms}"
        self._send(cmd)
        self._log("[WARN] Raw move executed. XYZ Kinematics Tracking is now INVALID until Homing.")
        return True

    def _raw_move_from_entries(self) -> None:
        try:
            targets = [int(float(v.get())) for v in self.raw_step_vars]
            duration_ms = max(int(self.raw_duration_var.get()), 1)
            self._raw_move(targets, duration_ms)
        except ValueError:
            messagebox.showerror("Error", "Step targets and duration must be numeric.")

    def _raw_jog_axis(self, axis: int, direction: int) -> None:
        if self.busy:
            return
        try:
            jog_amount = int(float(self.raw_jog_step.get()))
            duration_ms = max(int(self.raw_duration_var.get()), 1)
        except ValueError:
            return

        targets = list(self.current_steps)
        targets[axis] += direction * jog_amount
        self.raw_step_vars[axis].set(str(targets[axis]))
        self._raw_move(targets, duration_ms)

    def _home(self):
        if self.busy:
            return
        self.busy = True
        self.pending_mode = "home"
        self.motion_lbl.config(text="Homing...", foreground="orange")
        self._send("H")

    def _stop(self):
        self._send("STOP")
        if self.tour_active:
            self.tour_active = False
            self.current_tour_poi = None
            self.tour_status_lbl.config(text="Tour stopped (E-STOP).", foreground="#b35c00")

    # ---------------------------------------------------
    # DEPTH MAP / POINTS OF INTEREST
    # ---------------------------------------------------
    def _on_generate_depth_map(self):
        try:
            w = int(self.depth_map_w_var.get())
            h = int(self.depth_map_h_var.get())
            n = int(self.depth_num_blobs_var.get())
        except ValueError:
            messagebox.showerror("Error", "Map width/height/blob count must be integers.")
            return
        self.depth_map = generate_simulated_depth_map(w, h, n)
        self.depth_map_w, self.depth_map_h = w, h
        self.poi_list = []
        self.poi_tree.delete(*self.poi_tree.get_children())
        self.depth_map_status_lbl.config(
            text=f"Simulated depth map ready: {w}x{h}, {n} blob(s). Scan below to find points of interest.",
            foreground="green")
        self._log(f"[DEPTH] Generated simulated depth map {w}x{h} with {n} blob(s).")

    def _poi_to_xyz(self, poi: Dict) -> Tuple[float, float, float]:
        sx, sy, sz = (float(v.get()) for v in self.depth_scale)
        ox, oy, oz = (float(v.get()) for v in self.depth_offset)
        col_centered = poi["col"] - self.depth_map_w / 2.0
        row_centered = poi["row"] - self.depth_map_h / 2.0
        x = col_centered * sx + ox
        y = row_centered * sy + oy
        z = poi["depth"] * sz + oz
        return (x, y, z)

    def _on_scan_depth_map(self):
        if self.depth_map is None:
            messagebox.showerror("Error", "Generate (or load) a depth map first.")
            return
        try:
            threshold = float(self.depth_threshold_var.get())
            min_size = int(self.depth_min_region_var.get())
        except ValueError:
            messagebox.showerror("Error", "Threshold/min region size must be numeric.")
            return

        raw_pois = find_points_of_interest(self.depth_map, threshold, min_size)
        self.poi_list = []
        self.poi_tree.delete(*self.poi_tree.get_children())

        for idx, p in enumerate(raw_pois):
            xyz = self._poi_to_xyz(p)
            item_id = self.poi_tree.insert("", "end", values=(
                idx + 1, f"{p['row']:.1f}", f"{p['col']:.1f}", f"{p['depth']:.2f}",
                f"{xyz[0]:.1f}", f"{xyz[1]:.1f}", f"{xyz[2]:.1f}", "no"))
            self.poi_list.append({
                "idx": idx + 1, "row": p["row"], "col": p["col"], "depth": p["depth"],
                "xyz": xyz, "visited": False, "_tree_item": item_id,
            })

        self._log(f"[DEPTH] Found {len(self.poi_list)} point(s) of interest.")
        if not self.poi_list:
            messagebox.showinfo("No points found", "No regions met the threshold/min-size criteria. "
                                                     "Try lowering the threshold or min region size.")

    def _mark_poi_visited(self, poi: Dict, skipped: bool) -> None:
        poi["visited"] = True
        status = "skipped" if skipped else "yes"
        vals = list(self.poi_tree.item(poi["_tree_item"], "values"))
        vals[-1] = status
        self.poi_tree.item(poi["_tree_item"], values=vals)

    def _start_poi_tour(self):
        if not self.poi_list:
            messagebox.showinfo("No points", "Scan the depth map for points of interest first.")
            return
        if not self.xyz_valid:
            messagebox.showerror("XYZ invalid", "XYZ tracking is currently invalid. Run Home first.")
            return
        if self.busy:
            messagebox.showinfo("Busy", "Wait for the current move to finish before starting a tour.")
            return

        hx, hy, hz = self.HOME_XYZ

        def dist_from_home(p):
            x, y, z = p["xyz"]
            return math.sqrt((x - hx) ** 2 + (y - hy) ** 2 + (z - hz) ** 2)

        self.poi_tour_queue = sorted(
            (p for p in self.poi_list if not p["visited"]), key=dist_from_home)

        if not self.poi_tour_queue:
            messagebox.showinfo("Nothing to do", "All points are already marked visited. "
                                                  "Click 'Reset Visited Flags' to run the tour again.")
            return

        self.tour_active = True
        self.tour_status_lbl.config(text=f"Tour running: {len(self.poi_tour_queue)} point(s) remaining.",
                                     foreground="orange")
        self._log(f"[TOUR] Starting: {len(self.poi_tour_queue)} point(s), closest-to-home order.")
        self._advance_tour()

    def _stop_poi_tour(self):
        self.tour_active = False
        self.current_tour_poi = None
        self.tour_status_lbl.config(text="Tour stopped by user.", foreground="gray")
        self._log("[TOUR] Stopped by user.")

    def _reset_poi_visited(self):
        for p in self.poi_list:
            p["visited"] = False
            vals = list(self.poi_tree.item(p["_tree_item"], "values"))
            vals[-1] = "no"
            self.poi_tree.item(p["_tree_item"], values=vals)
        self.tour_status_lbl.config(text="Visited flags reset.", foreground="gray")
        self._log("[TOUR] Visited flags reset on all points.")

    def _advance_tour(self):
        if not self.tour_active:
            return
        while self.poi_tour_queue:
            poi = self.poi_tour_queue.pop(0)
            if poi["visited"]:
                continue
            x, y, z = poi["xyz"]
            sent = self._move_to_xyz(x, y, z)
            if sent:
                self.current_tour_poi = poi
                remaining = len(self.poi_tour_queue)
                self.tour_status_lbl.config(
                    text=f"Tour running: heading to POI #{poi['idx']}, {remaining} more after this.",
                    foreground="orange")
                return  # wait for "OK" in _handle_rx to continue
            else:
                self._mark_poi_visited(poi, skipped=True)
                self._log(f"[TOUR] Skipping POI #{poi['idx']} at ({x:.1f}, {y:.1f}, {z:.1f}) "
                          f"-- unreachable or outside tested axis limits.")
        # queue exhausted
        self.tour_active = False
        self.tour_status_lbl.config(text="Tour complete.", foreground="green")
        self._log("[TOUR] Complete -- all points visited or skipped.")

    # ---------------------------------------------------
    # LIVE CAMERA (Raspberry Pi)
    # ---------------------------------------------------
    def _toggle_camera_connection(self):
        if self.cam_client is not None and self.cam_client.is_connected():
            self.cam_client.stop()
            self.cam_client = None
            self.cam_connect_btn.config(text="Connect")
            self.cam_conn_status_lbl.config(text="Not connected", foreground="gray")
            self._log("[CAM] Disconnected from Pi camera server.")
            return

        ip = self.cam_ip_var.get().strip()
        try:
            port = int(self.cam_port_var.get())
        except ValueError:
            messagebox.showerror("Error", "Port must be numeric.")
            return

        try:
            self.cam_client = CameraClient(self.cam_frame_queue)
            self.cam_client.start(ip, port)
            self.cam_connect_btn.config(text="Disconnect")
            self.cam_conn_status_lbl.config(text=f"Connected to {ip}:{port}", foreground="green")
            self._log(f"[CAM] Connected to Pi camera server at {ip}:{port}.")
        except Exception as ex:
            messagebox.showerror("Camera connection failed", str(ex))
            self.cam_client = None

    def _xyz_to_video_pixel(self, x: float, y: float, img_w: int, img_h: int) -> Tuple[int, int]:
        """Projects a robot XY (mm) onto a pixel in the video frame, using
        the workspace box as the pixel<->mm calibration (box left/right
        edges map to image left/right edges, same for top/bottom)."""
        x0, x1 = self.workspace_x
        y0, y1 = self.workspace_y
        frac_x = (x - x0) / (x1 - x0) if x1 != x0 else 0.5
        frac_y = (y - y0) / (y1 - y0) if y1 != y0 else 0.5
        frac_x = min(max(frac_x, 0.0), 1.0)
        frac_y = min(max(frac_y, 0.0), 1.0)
        px = int(frac_x * img_w)
        # Image row 0 is the top; assume robot +Y is "away/up" in the frame.
        py = int((1.0 - frac_y) * img_h)
        return px, py

    def _poll_live_camera(self):
        latest_video = None
        latest_depth = None
        while not self.cam_frame_queue.empty():
            tag, img = self.cam_frame_queue.get_nowait()
            if tag == "V":
                latest_video = img
            elif tag == "D":
                latest_depth = img

        if latest_video is not None:
            self._display_video_frame(latest_video)
        if latest_depth is not None:
            self._display_depth_frame(latest_depth)

        self.root.after(66, self._poll_live_camera)  # roughly 15 fps UI refresh

    def _display_video_frame(self, img) -> None:
        # img is expected as an RGB numpy array (H, W, 3) from camera_client.
        pil_img = Image.fromarray(img)
        if self.cam_overlay_var.get():
            draw = ImageDraw.Draw(pil_img)
            w, h = pil_img.size
            px, py = self._xyz_to_video_pixel(self.current_xyz[0], self.current_xyz[1], w, h)
            r = 10
            draw.line((px - r, py, px + r, py), fill=(255, 0, 0), width=2)
            draw.line((px, py - r, px, py + r), fill=(255, 0, 0), width=2)
            draw.ellipse((px - r, py - r, px + r, py + r), outline=(255, 0, 0), width=2)
            draw.text((px + r + 4, py - r), f"({self.current_xyz[0]:.0f},{self.current_xyz[1]:.0f})",
                      fill=(255, 0, 0))
        self.cam_video_photo = ImageTk.PhotoImage(pil_img)
        self.cam_video_label.config(image=self.cam_video_photo, text="")

    def _display_depth_frame(self, img) -> None:
        pil_img = Image.fromarray(img)
        self.cam_depth_photo = ImageTk.PhotoImage(pil_img)
        self.cam_depth_label.config(image=self.cam_depth_photo, text="")

    def on_closing(self):
        """Handle graceful shutdown when closing the window or pressing Ctrl+C."""
        print("[INFO] Shutting down gracefully...")
        self.tour_active = False
        if self.cam_client is not None:
            self.cam_client.stop()
        if self.link.is_connected():
            try:
                self.link.send_line("STOP")
            except Exception:
                pass
            self.link.disconnect()
        self.vision_thread.stop_tracking()
        self.root.destroy()


if __name__ == "__main__":
    tk_root = tk.Tk()
    app = DeltaRobotApp(tk_root)
    tk_root.protocol("WM_DELETE_WINDOW", app.on_closing)
    try:
        tk_root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()

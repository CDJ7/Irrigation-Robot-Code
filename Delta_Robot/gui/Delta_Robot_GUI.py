"""
Delta Robot - Combined Master Controller & Debug GUI
----------------------------------------------------
This unified interface features tabs for standard XYZ operations
(with camera tracking support) and raw motor debugging.

If you use the Debug tab to move motors individually, the robot's
XYZ tracking becomes invalid. You must re-home the robot to restore
kinematic XYZ moves.

The Home Button Does NOT Work yet, if you press it during testing make sure you can disconnect the stepper drivers power.
"""

import math
import time
import threading
import queue
import random
import tkinter as tk
from tkinter import ttk, messagebox
from typing import List, Optional, Tuple

import serial
import serial.tools.list_ports

# Kinematics constants
SQRT3 = math.sqrt(3.0)
SIN120 = SQRT3 / 2.0
COS120 = -0.5
TAN60 = SQRT3
TAN30 = 1.0 / SQRT3

class DeltaKinematics:
    def __init__(self, f: float = 100.0, e: float = 50.0, rf: float = 150.0, re: float = 280.0):
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
        if t1 is None: return None
        t2 = self._calc_angle_yz(x0 * COS120 + y0 * SIN120, y0 * COS120 - x0 * SIN120, z0)
        if t2 is None: return None
        t3 = self._calc_angle_yz(x0 * COS120 - y0 * SIN120, y0 * COS120 + x0 * SIN120, z0)
        if t3 is None: return None
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
                # Simulated tracking data. Replace with cv2 logic.
                simulated_x = random.uniform(-40.0, 40.0)
                simulated_y = random.uniform(-40.0, 40.0)
                simulated_z = -200.0
                self.target_queue.put((simulated_x, simulated_y, simulated_z))
                time.sleep(1.0)
            else:
                time.sleep(0.1)

class DeltaRobotApp:
    HOME_XYZ = (0.0, 0.0, -200.0)

    def __init__(self, master_window: tk.Tk):
        self.root = master_window
        self.root.title("Delta Robot Master Controller")
        self.root.geometry("950x850")

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

        # Camera state
        self.camera_target_queue: queue.Queue = queue.Queue()
        self.vision_thread = CameraVisionThread(self.camera_target_queue)
        self.vision_thread.start()
        self.tracking_mode = False

        self._build_ui()
        self._poll_serial()
        self._poll_camera()

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

        self.notebook.add(self.tab_main, text="Main Operations")
        self.notebook.add(self.tab_debug, text="Debug & Calibration")

        self._build_main_tab(pad)
        self._build_debug_tab(pad)

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
            ("Base f (mm)", "f", 100.0), ("Effector e (mm)", "e", 50.0),
            ("Bicep rf (mm)", "rf", 150.0), ("Forearm re (mm)", "re", 280.0),
            ("Steps/rev", "steps_per_rev", 3200.0), ("Max speed (mm/s)", "max_speed", 80.0),
        ]
        for i, (label, key, default) in enumerate(fields):
            ttk.Label(geo, text=label).grid(row=i // 3, column=(i % 3) * 2, sticky="e", **pad)
            v = tk.StringVar(value=str(default))
            ttk.Entry(geo, textvariable=v, width=8).grid(row=i // 3, column=(i % 3) * 2 + 1, **pad)
            self.geo_vars[key] = v
        ttk.Button(geo, text="Apply Geo", command=self._apply_geometry).grid(row=2, column=0, columnspan=2, **pad)

        self.axis_offset = [tk.StringVar(value="0.0") for _ in range(3)]
        self.axis_invert = [tk.BooleanVar(value=False) for _ in range(3)]
        for i in range(3):
            ttk.Label(geo, text=f"Offset {i+1} (deg)").grid(row=3, column=i * 2, **pad)
            ttk.Entry(geo, textvariable=self.axis_offset[i], width=6).grid(row=3, column=i * 2 + 1, **pad)
            ttk.Checkbutton(geo, text=f"Invert {i+1}", variable=self.axis_invert[i]).grid(
                row=4, column=i * 2, columnspan=2, **pad)

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

        jog_btns = [("X-", (-1,0,0)), ("X+", (1,0,0)), ("Y-", (0,-1,0)), ("Y+", (0,1,0)), ("Z-", (0,0,-1)), ("Z+", (0,0,1))]
        for i, (label, delta) in enumerate(jog_btns):
            ttk.Button(jog, text=label, width=4, command=lambda d=delta: self._jog(d)).grid(row=0, column=2 + i, **pad)

        # Camera integration
        cam = ttk.LabelFrame(self.tab_main, text="Camera / Vision Tracking Integration")
        cam.pack(fill="x", **pad)
        ttk.Label(cam, text="Listens for automated XYZ coordinates from the vision thread.", foreground="blue").pack(anchor="w", **pad)
        self.cam_btn = ttk.Button(cam, text="Start Camera Tracking", command=self._toggle_camera_tracking)
        self.cam_btn.pack(side="left", **pad)
        self.cam_status_lbl = ttk.Label(cam, text="Camera Offline", foreground="gray")
        self.cam_status_lbl.pack(side="left", **pad)

    def _build_debug_tab(self, pad):
        setup = ttk.LabelFrame(self.tab_debug, text="⚠ Manual Axis Setup (Bypasses Kinematics) ⚠")
        setup.pack(fill="both", expand=True, **pad)

        warn = ttk.Label(
            setup,
            text=("Moves each motor directly by step count. Using this invalidates XYZ tracking.\n"
                  "You MUST Home the robot afterwards before returning to the Main Tab."),
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
        ttk.Button(setup, text="Send Raw Move", command=self._raw_move_from_entries).grid(row=2, column=2, columnspan=2, **pad)

        ttk.Label(setup, text="Jog steps:").grid(row=3, column=0, sticky="e", **pad)
        self.raw_jog_step = tk.StringVar(value="100")
        ttk.Entry(setup, textvariable=self.raw_jog_step, width=8).grid(row=3, column=1, **pad)

        for i in range(3):
            ttk.Button(setup, text=f"A{i+1} -", width=5, command=lambda ax=i: self._raw_jog_axis(ax, -1)).grid(row=3, column=2 + i * 2, **pad)
            ttk.Button(setup, text=f"A{i+1} +", width=5, command=lambda ax=i: self._raw_jog_axis(ax, 1)).grid(row=3, column=3 + i * 2, **pad)

        ttk.Button(setup, text="Set Current Position as Home (Zero ESP32)", command=lambda: self._send("Z")).grid(row=4, column=0, columnspan=4, padx=pad["padx"], pady=20)

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
            if not self.tracking_mode: # Suppress error box spam if tracking
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
            if self.pending_mode == "move" and self.pending_steps is not None:
                self.current_steps = self.pending_steps
                self.current_xyz = self.pending_xyz
                # Sync raw vars just in case user switches to debug tab
                for i, v in enumerate(self.raw_step_vars):
                    v.set(str(self.current_steps[i]))
                    
            elif self.pending_mode == "raw" and self.pending_steps is not None:
                self.current_steps = self.pending_steps
                for i, v in enumerate(self.raw_step_vars):
                    v.set(str(self.current_steps[i]))
            self._clear_pending()

        elif text == "HOMED":
            self.current_steps = [0, 0, 0]
            self.current_xyz = list(self.HOME_XYZ)
            for i, v in enumerate(self.raw_step_vars):
                v.set("0")
            for i in range(3):
                self.xyz_vars[i].set(f"{self.HOME_XYZ[i]:.2f}")
                
            self.xyz_valid = True 
            self._log("[INFO] Robot has successfully homed. XYZ Tracking Valid.")
            self._clear_pending()

        elif text.startswith("ERR"):
            self._clear_pending()
            if "BUSY" in text:
                pass # Expected occasionally, ignore quietly
            elif "STOPPED" in text:
                self._log("[WARN] Move aborted by STOP.")
            else:
                self._log(f"[WARN] ESP32 error: {text}")

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

    def _move_to_xyz(self, x: float, y: float, z: float) -> None:
        if self.busy: return
        
        if not self.xyz_valid:
            messagebox.showerror(
                "XYZ Tracking Invalid",
                "A raw/manual axis move was used. XYZ coordinates are no longer trustworthy. "
                "Run HOME to reset the physical location before using XYZ moves again."
            )
            return

        angles = self.kin.inverse(x, y, z)
        if angles is None:
            self._log(f"[WARN] Target ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable.")
            return

        target_steps = self._angles_to_steps(angles)
        dx = x - self.current_xyz[0]
        dy = y - self.current_xyz[1]
        dz = z - self.current_xyz[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        speed = max(self.max_speed, 1.0)
        dur_ms = max(int((dist / speed) * 1000.0), 20) 

        cmd = f"M {target_steps[0]} {target_steps[1]} {target_steps[2]} {dur_ms}"
        self.busy = True
        self.pending_mode = "move"
        self.pending_steps = target_steps
        self.pending_xyz = [x, y, z]
        self.motion_lbl.config(text="Moving...", foreground="orange")
        self._send(cmd)

    def _move_to_entry_position(self):
        try:
            x, y, z = float(self.xyz_vars[0].get()), float(self.xyz_vars[1].get()), float(self.xyz_vars[2].get())
            self._move_to_xyz(x, y, z)
        except ValueError:
            messagebox.showerror("Error", "X/Y/Z must be numeric.")

    def _jog(self, delta: Tuple[int, int, int]) -> None:
        if self.busy: return
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

    def _raw_move(self, target_steps: List[int], duration_ms: int) -> None:
        if self.busy: return
        self.xyz_valid = False # Invalidate kinematic tracking!
        
        self.busy = True
        self.pending_mode = "raw"
        self.pending_steps = list(target_steps)
        self.motion_lbl.config(text="Moving (raw)...", foreground="#b35c00")

        cmd = f"M {target_steps[0]} {target_steps[1]} {target_steps[2]} {duration_ms}"
        self._send(cmd)
        self._log(f"[WARN] Raw move executed. XYZ Kinematics Tracking is now INVALID until Homing.")

    def _raw_move_from_entries(self) -> None:
        try:
            targets = [int(float(v.get())) for v in self.raw_step_vars]
            duration_ms = max(int(self.raw_duration_var.get()), 1)
            self._raw_move(targets, duration_ms)
        except ValueError:
            messagebox.showerror("Error", "Step targets and duration must be numeric.")

    def _raw_jog_axis(self, axis: int, direction: int) -> None:
        if self.busy: return
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
        if self.busy: return
        
        self.busy = True
        self.pending_mode = "home"
        self.motion_lbl.config(text="Homing...", foreground="orange")
        self._send("H")

    def _stop(self):
        self._send("STOP")

    def on_closing(self):
        """Handle graceful shutdown when closing the window or pressing Ctrl+C."""
        print("[INFO] Shutting down gracefully...")
        if self.link.is_connected():
            try:
                # Send a final stop command just in case it was moving
                self.link.send_line("STOP")
            except Exception:
                pass
            self.link.disconnect()
        self.vision_thread.stop_tracking()
        self.root.destroy()

if __name__ == "__main__":
    tk_root = tk.Tk()
    app = DeltaRobotApp(tk_root)
    
    # Bind the window's "X" close button to our cleanup function
    tk_root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        tk_root.mainloop()
    except KeyboardInterrupt:
        # Catch Ctrl+C in the terminal and close cleanly
        app.on_closing()

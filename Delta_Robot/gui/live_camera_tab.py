"""
live_camera_tab.py -- a "Live Camera (Pi)" tab for the Tkinter GUI. VIEW ONLY: it shows the Pi's pictures and detections
and never sends anything to the robot or the ESP32.

Needs on the PC:  pip install pillow numpy opencv-python
"""
import queue
import time
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk

from camera_client import CameraClient
from camera_overlay import draw_detections, fit_inside, status_colour, summarize_frame

DEFAULT_HOST = "192.168.137.128"   # the Pi on the Ethernet-sharing link (it can change; edit it in the box)
DEFAULT_PORT = 8485
PANEL_W, PANEL_H = 460, 345        # each picture is shrunk to fit this box


class LiveCameraTab:
    def __init__(self, parent: tk.Misc, root: tk.Tk, log=print, client_factory=CameraClient):
        self.parent, self.root, self.log = parent, root, log
        self.frame_queue: "queue.Queue" = queue.Queue(maxsize=12)      # bounded: old frames are dropped
        self.client = client_factory(self.frame_queue)
        self._running = False
        self._last_video = None            # newest ('V') picture and its seq
        self._last_video_seq = None
        self._dets_by_seq = {}             # a few recent J messages, so each picture gets ITS detections
        self._latest_det = None
        self._video_photo = None           # keep references so Tk does not garbage-collect the images
        self._depth_photo = None
        self._frame_times = []
        self._after_id = None
        self._build()
        self._poll()

    # ------------------------------------------------------------------ layout
    def _build(self) -> None:
        pad = {"padx": 6, "pady": 4}
        top = ttk.LabelFrame(self.parent, text="Raspberry Pi camera server")
        top.pack(fill="x", **pad)
        ttk.Label(top, text="Pi address:").grid(row=0, column=0, **pad)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(top, textvariable=self.host_var, width=18).grid(row=0, column=1, **pad)
        ttk.Label(top, text="Port:").grid(row=0, column=2, **pad)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        ttk.Entry(top, textvariable=self.port_var, width=7).grid(row=0, column=3, **pad)
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle)
        self.connect_btn.grid(row=0, column=4, **pad)
        self.status_lbl = ttk.Label(top, text="Not connected", foreground="red")
        self.status_lbl.grid(row=0, column=5, sticky="w", **pad)
        ttk.Label(top, foreground="#555", wraplength=880, justify="left",
                  text="View only: this tab shows the Pi's camera and detections. It never moves the robot. "
                       "Start the server on the Pi first:  cd ~/Delta_Robot/pi && python pi_camera_server.py").grid(
            row=1, column=0, columnspan=6, sticky="w", **pad)

        self.info_lbl = ttk.Label(self.parent, text="", foreground="#555")
        self.info_lbl.pack(anchor="w", padx=8)

        panels = ttk.Frame(self.parent)
        panels.pack(fill="both", expand=True, **pad)
        vf = ttk.LabelFrame(panels, text="Left eye + detections (green = allowed class, red = not allowed)")
        vf.pack(side="left", fill="both", expand=True, **pad)
        self.video_lbl = ttk.Label(vf, text="No video yet", anchor="center")
        self.video_lbl.pack(fill="both", expand=True)
        df = ttk.LabelFrame(panels, text="Depth (red = near, blue = far, black = no data)")
        df.pack(side="left", fill="both", expand=True, **pad)
        self.depth_lbl = ttk.Label(df, text="No depth yet", anchor="center")
        self.depth_lbl.pack(fill="both", expand=True)

        tf = ttk.LabelFrame(self.parent, text="Current detections (camera frame, millimetres: X right, Y down, Z forward)")
        tf.pack(fill="x", **pad)
        cols = ("class", "conf", "allowed", "x", "y", "z")
        self.table = ttk.Treeview(tf, columns=cols, show="headings", height=4)
        for c, w in zip(cols, (140, 70, 80, 90, 90, 90)):
            self.table.heading(c, text=c.upper())
            self.table.column(c, width=w, anchor="center")
        self.table.pack(fill="x", padx=4, pady=4)

    # ----------------------------------------------------------------- actions
    def _toggle(self) -> None:
        if self._running:
            self.client.stop()
            self._running = False
            self.connect_btn.config(text="Connect")
            self._show_status({"state": "stopped", "message": "disconnected"})
            self.log("[CAM] Disconnected from the Pi camera server.")
            return
        host = self.host_var.get().strip()
        try:
            port = int(self.port_var.get())
        except ValueError:
            self.status_lbl.config(text="Port must be a number", foreground="red")
            return
        self._running = True
        self.connect_btn.config(text="Disconnect")
        self.client.start(host, port)           # returns at once; it connects (and reconnects) in the background
        self.log(f"[CAM] Connecting to {host}:{port} ...")

    def shutdown(self) -> None:
        """Call when the window closes."""
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self.client.stop()

    # ------------------------------------------------------------------ polling
    def _poll(self) -> None:
        newest_video = newest_depth = None
        for _ in range(60):                      # handle a burst, but never hog the GUI thread
            try:
                item = self.frame_queue.get_nowait()
            except queue.Empty:
                break
            kind = item[0]
            if kind == "STATUS":
                self._show_status(item[1])
            elif kind == "DET":
                self._latest_det = item[1]
                self._dets_by_seq[item[1].get("seq")] = item[1]
                if len(self._dets_by_seq) > 20:
                    self._dets_by_seq.pop(next(iter(self._dets_by_seq)))
                self._update_info(item[1])
                self._update_table(item[1])
            elif kind == "V":
                newest_video = item
            elif kind == "D":
                newest_depth = item
        if newest_video is not None:
            self._show_video(newest_video[1], newest_video[2])
        if newest_depth is not None:
            self._show_depth(newest_depth[1])
        self._after_id = self.root.after(40, self._poll)

    def _show_status(self, st: dict) -> None:
        state = st.get("state", "")
        text = {"connected": "Connected", "connecting": "Connecting...", "stopped": "Not connected"}.get(
            state, "Disconnected")
        msg = st.get("message", "")
        self.status_lbl.config(text=f"{text}  ({msg})" if msg and state != "connected" else text,
                               foreground=status_colour(state))
        if state != "connected":
            self.info_lbl.config(text="")

    def _show_video(self, img, seq) -> None:
        det = self._dets_by_seq.get(seq)         # the detections that belong to THIS picture
        dets = det.get("detections", []) if det else []
        shown = fit_inside(draw_detections(img, dets) if dets else img, PANEL_W, PANEL_H)
        self._video_photo = ImageTk.PhotoImage(Image.fromarray(shown))
        self.video_lbl.config(image=self._video_photo, text="")
        now = time.time()
        self._frame_times = [t for t in self._frame_times if now - t < 2.0] + [now]

    def _show_depth(self, img) -> None:
        self._depth_photo = ImageTk.PhotoImage(Image.fromarray(fit_inside(img, PANEL_W, PANEL_H)))
        self.depth_lbl.config(image=self._depth_photo, text="")

    def _update_info(self, det: dict) -> None:
        text, ok = summarize_frame(det)
        n = len(self._frame_times)
        gui_fps = (n - 1) / (self._frame_times[-1] - self._frame_times[0]) if n > 1 else 0.0
        self.info_lbl.config(text=f"{text} | shown here: {gui_fps:.1f} fps", foreground="#555" if ok else "#b35c00")

    def _update_table(self, det: dict) -> None:
        self.table.delete(*self.table.get_children())
        for d in det.get("detections", []):
            xyz = d.get("xyz_mm") or ["-", "-", "-"]
            fmt = lambda v: f"{v:.0f}" if isinstance(v, (int, float)) else str(v)
            self.table.insert("", "end", values=(
                d.get("class", "?"), f"{100 * float(d.get('confidence', 0)):.0f}%",
                "yes" if d.get("allowed") else "NO", fmt(xyz[0]), fmt(xyz[1]), fmt(xyz[2])))

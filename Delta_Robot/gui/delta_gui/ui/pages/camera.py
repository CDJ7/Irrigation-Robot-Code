"""
camera.py -- the Camera page: live video with detection boxes and labels (class, confidence, camera XYZ, robot XYZ), the colourised
depth picture, a table of the current detections and the "Move above selected" button.

The camera frame (X right, Y down, Z out of the lens, mm) is turned into the ROBOT frame with the saved camera-to-robot
calibration. Without that calibration the robot columns stay empty and "Move above selected" is disabled. The effector crosshair
(where the robot's tip appears in the picture) uses the same calibration instead of the old "workspace box = picture" guess.
"""
import time
from typing import Dict, List, Optional

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage
from PyQt6.QtWidgets import (QAbstractItemView, QCheckBox, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
                             QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from camera_overlay import draw_crosshair, draw_detections, summarize_frame
from ..widgets import ImageView, Pill, card, spin

COLUMNS = ["Class", "Conf", "Allowed", "Cam X", "Cam Y", "Cam Z", "Robot X", "Robot Y", "Robot Z"]


def to_qimage(rgb: np.ndarray) -> QImage:
    h, w = rgb.shape[:2]
    rgb = np.ascontiguousarray(rgb)
    return QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()     # copy: the numpy buffer may go away


class CameraPage(QWidget):
    def __init__(self, ctx, camera):
        super().__init__()
        self.setObjectName("page")
        self.ctx, self.camera, self.robot = ctx, camera, ctx.robot
        self._dets: Dict[int, dict] = {}            # recent J messages by seq, so each picture gets ITS detections
        self._latest: Optional[dict] = None
        self._rows: List[dict] = []                 # detections currently in the table
        self._frame_times: List[float] = []
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(12)

        # ------------------------------------------------------------- connection
        top = card("Raspberry Pi camera server")
        tl = QGridLayout(top)
        cam = ctx.settings.data["camera"]
        tl.addWidget(QLabel("Pi address"), 0, 0)
        self.host = QLineEdit(cam["host"]); self.host.setMinimumWidth(150); tl.addWidget(self.host, 0, 1)
        tl.addWidget(QLabel("Port"), 0, 2)
        self.port = QLineEdit(str(cam["port"])); self.port.setMaximumWidth(80); tl.addWidget(self.port, 0, 3)
        self.connect_btn = QPushButton("Connect"); self.connect_btn.setObjectName("primary"); self.connect_btn.clicked.connect(self._toggle)
        tl.addWidget(self.connect_btn, 0, 4)
        self.sim_chk = QCheckBox("Simulated camera (test mode)"); self.sim_chk.setChecked(bool(cam["simulate"]))
        self.sim_chk.setToolTip("Runs the real Pi server inside this program with a fake camera. For testing without a Pi.")
        tl.addWidget(self.sim_chk, 0, 5)
        self.status_pill = Pill("Not connected", "idle"); tl.addWidget(self.status_pill, 0, 6)
        tl.setColumnStretch(7, 1)
        self.info = QLabel(""); self.info.setObjectName("muted"); self.info.setWordWrap(True)
        tl.addWidget(self.info, 1, 0, 1, 8)
        root.addWidget(top)

        # ------------------------------------------------------------- pictures
        pics = QHBoxLayout()
        left, right = QVBoxLayout(), QVBoxLayout()
        self.video_title = QLabel("Left eye + detections   (green = allowed class, red = not allowed)"); self.video_title.setObjectName("muted")
        self.depth_title = QLabel("Depth   (red = near, blue = far, black = no data)"); self.depth_title.setObjectName("muted")
        self.video = ImageView("No video yet"); self.depth = ImageView("No depth yet")
        left.addWidget(self.video_title); left.addWidget(self.video, 1)
        right.addWidget(self.depth_title); right.addWidget(self.depth, 1)
        pics.addLayout(left, 1); pics.addLayout(right, 1)
        root.addLayout(pics, 1)

        # ------------------------------------------------------------- table + actions
        bottom = QHBoxLayout()
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setMinimumHeight(130); self.table.setMaximumHeight(190)
        self.table.itemSelectionChanged.connect(self._update_actions)
        bottom.addWidget(self.table, 3)
        act = card("Manual test")
        al = QVBoxLayout(act)
        al.addWidget(QLabel("Approach height above the target"))
        self.approach = spin(0, 300, float(cam["approach_height_mm"]), 0, " mm", 5)
        al.addWidget(self.approach)
        self.move_btn = QPushButton("Move above selected"); self.move_btn.setObjectName("primary"); self.move_btn.clicked.connect(self._move_above)
        al.addWidget(self.move_btn)
        self.why_lbl = QLabel(""); self.why_lbl.setObjectName("muted"); self.why_lbl.setWordWrap(True)
        al.addWidget(self.why_lbl)
        self.cross_chk = QCheckBox("Show effector crosshair"); self.cross_chk.setChecked(bool(cam["show_effector_crosshair"]))
        al.addWidget(self.cross_chk)
        bottom.addWidget(act, 1)
        root.addLayout(bottom)

        camera.status.connect(self._on_status)
        camera.detections.connect(self._on_detections)
        camera.video.connect(self._on_video)
        camera.depth.connect(self._on_depth)
        ctx.state.connect(self._update_actions)
        ctx.settings_applied.connect(self._on_settings_applied)
        self._update_actions()

    def _on_settings_applied(self):
        """A new calibration changes every robot-frame column straight away; do not wait for the next frame."""
        if self._latest:
            self._fill_table(self._latest)
        else:
            self._update_actions()

    # ---------------------------------------------------------------- connection
    def _toggle(self):
        st = self.camera.client.status["state"]
        if st in ("connected", "connecting") or self.camera.simulated_running:
            self.camera.stop()
            self.connect_btn.setText("Connect")
            self._on_status({"state": "stopped", "message": "disconnected"})
            return
        cam = self.ctx.settings.data["camera"]
        cam["simulate"] = self.sim_chk.isChecked()
        if self.sim_chk.isChecked():
            try:
                self.camera.start_simulated()
            except Exception as ex:                  # noqa: BLE001
                QMessageBox.warning(self, "Simulated camera", f"Could not start the simulated camera:\n{ex}")
                return
        else:
            try:
                port = int(self.port.text())
            except ValueError:
                QMessageBox.warning(self, "Camera", "The port must be a number.")
                return
            cam["host"], cam["port"] = self.host.text().strip(), port
            self.camera.start(cam["host"], port)
        self.connect_btn.setText("Disconnect")

    def _on_status(self, st: dict):
        state = st.get("state", "")
        level = {"connected": "ok", "connecting": "warn"}.get(state, "bad" if state == "disconnected" else "idle")
        word = {"connected": "Connected", "connecting": "Connecting…", "stopped": "Not connected"}.get(state, "Disconnected")
        self.status_pill.set(word, level)
        self.status_pill.setToolTip(st.get("message", ""))
        if state not in ("connected",):
            self.info.setText(st.get("message", "") if state == "disconnected" else "")

    # ---------------------------------------------------------------- data from the Pi
    def _on_detections(self, det: dict):
        seq = det.get("seq")
        self._latest = det
        self._dets[seq] = det
        while len(self._dets) > 20:
            self._dets.pop(next(iter(self._dets)))
        calibrating = det.get("mode") == "calibration_capture"
        self.video_title.setText("CALIBRATION: left eye with detected corners" if calibrating else
                                 "Left eye + detections   (green = allowed class, red = not allowed)")
        self.depth_title.setText("CALIBRATION: right eye with detected corners" if calibrating else
                                 "Depth   (red = near, blue = far, black = no data)")
        text, ok = summarize_frame(det)
        n = len(self._frame_times)
        fps = (n - 1) / (self._frame_times[-1] - self._frame_times[0]) if n > 1 else 0.0
        self.info.setText(f"{text}  |  shown here: {fps:.1f} fps")
        self._fill_table(det)

    def _robot_xyz(self, xyz) -> Optional[List[float]]:
        tf = self.ctx.calibration
        return [float(v) for v in tf.to_robot(xyz)] if (tf.valid and xyz) else None

    def _fill_table(self, det: dict):
        keep = self.table.currentRow()
        self._rows = []
        for d in det.get("detections", []):
            d = dict(d)
            d["robot_xyz"] = self._robot_xyz(d.get("xyz_mm"))
            self._rows.append(d)
        self.table.setRowCount(len(self._rows))
        for r, d in enumerate(self._rows):
            cam = d.get("xyz_mm") or [None] * 3
            rob = d.get("robot_xyz") or [None] * 3
            cells = [d.get("class", "?"), f"{100 * float(d.get('confidence', 0)):.0f}%", "yes" if d.get("allowed") else "NO"]
            cells += [("—" if v is None else f"{v:.0f}") for v in list(cam) + list(rob)]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(r, c, item)
        if 0 <= keep < len(self._rows):
            self.table.selectRow(keep)
        self._update_actions()

    def _on_video(self, rgb: np.ndarray, seq):
        det = self._dets.get(seq)
        dets = []
        if det:
            for d in det.get("detections", []):
                d = dict(d); d["robot_xyz"] = self._robot_xyz(d.get("xyz_mm")); dets.append(d)
        img = draw_detections(rgb, dets) if dets else rgb
        tf = self.ctx.calibration
        intr = (det or self._latest or {}).get("intrinsics")
        r = self.robot
        if self.cross_chk.isChecked() and tf.valid and r.xyz_valid and intr:
            px = tf.project(r.current_xyz, intr["fx"], intr["fy"], intr["cx"], intr["cy"])
            if px is not None:
                img = draw_crosshair(img, px, "effector")
        self.video.set_image(to_qimage(img))
        now = time.time()
        self._frame_times = [t for t in self._frame_times if now - t < 2.0] + [now]

    def _on_depth(self, rgb: np.ndarray, seq):
        self.depth.set_image(to_qimage(rgb))

    # ---------------------------------------------------------------- "Move above selected"
    def _target_for(self, d: dict):
        """(target robot XYZ, None) or (None, reason it is not allowed)."""
        tf = self.ctx.calibration
        if not tf.valid:
            return None, "Camera-to-robot calibration is not done yet (Calibration page)."
        if not (self._latest or {}).get("calibrated", False) and (self._latest or {}).get("mode") != "calibration_capture":
            return None, "The stereo camera is not calibrated: depth values are not trustworthy."
        if not d.get("allowed"):
            return None, f"'{d.get('class')}' is not an allowed (strawberry) class."
        if not d.get("robot_xyz"):
            return None, "The selected detection has no valid depth."
        x, y, z = d["robot_xyz"]
        target = (x, y, z + float(self.approach.value()))
        why = self.robot.check_target(*target)
        return (None, why) if why else (target, None)

    def _update_actions(self, *_):
        row = self.table.currentRow()
        r = self.robot
        if not (r.connected and not r.estopped and not r.busy and r.drivers_enabled and r.xyz_valid):
            reason = "The robot is not ready to move (connect, enable the motors and home it)."
            ok = False
        elif not (0 <= row < len(self._rows)):
            reason, ok = "Select a detection in the table.", False
        else:
            tgt, why = self._target_for(self._rows[row])
            reason, ok = (why or ""), tgt is not None
        self.move_btn.setEnabled(ok)
        self.why_lbl.setText("" if ok else reason)

    def _move_above(self):
        row = self.table.currentRow()
        if not (0 <= row < len(self._rows)):
            return
        tgt, why = self._target_for(self._rows[row])
        if tgt is None:
            QMessageBox.information(self, "Move above selected", why or "Not allowed.")
            return
        if QMessageBox.question(self, "Move above selected", f"Move the effector to X {tgt[0]:.1f}  Y {tgt[1]:.1f}  Z {tgt[2]:.1f} mm "
                                f"({self.approach.value():.0f} mm above the target)?") == QMessageBox.StandardButton.Yes:
            self.robot.move_xyz(*tgt, source="camera")

    def save_to_settings(self):
        cam = self.ctx.settings.data["camera"]
        cam["host"] = self.host.text().strip() or cam["host"]
        try:
            cam["port"] = int(self.port.text())
        except ValueError:
            pass
        cam["simulate"] = self.sim_chk.isChecked()
        cam["approach_height_mm"] = float(self.approach.value())
        cam["show_effector_crosshair"] = self.cross_chk.isChecked()

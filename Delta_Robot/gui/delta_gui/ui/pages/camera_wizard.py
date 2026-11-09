"""
camera_wizard.py -- the guided CAMERA -> ROBOT calibration.

Idea: fix a high-contrast target mark to the effector (at the tool centre point). For several robot positions the camera measures where
the mark is (camera XYZ) and the robot knows where it is (robot XYZ). The best rotation + translation for those pairs is the calibration.

  1. Click the middle of the target mark in the live picture (the Pi then measures the 3D position of that pixel).
  2. Move the robot to a point (suggested points are spread through the workspace) and press "Record point": about eight readings are
     averaged; noisy or depth-less readings are refused.
  3. With 4+ points (8 recommended) press "Solve": the error of every point is shown. Remove a bad point and solve again if needed.
  4. "Accept and save" stores it; the Camera page then shows robot coordinates, the effector crosshair and enables "Move above selected".
"""
import math
from typing import List, Optional

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QMessageBox, QPushButton, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)

from camera_overlay import draw_crosshair
from ...core.calibration import CameraCalibrationSession, ProbeCollector, SolveReport, suggest_points
from ..widgets import CheckLabel, ClickableImage, Pill, card
from .camera import to_qimage

COLS = ["#", "Robot X", "Robot Y", "Robot Z", "Cam X", "Cam Y", "Cam Z", "Error", "Status"]


class CameraWizard(QWidget):
    def __init__(self, ctx, camera):
        super().__init__()
        self.ctx, self.camera, self.robot = ctx, camera, ctx.robot
        self.rows: List[dict] = []                    # {"robot": (x,y,z), "cam": (x,y,z)|None, "err": float|None, "spread": float}
        self.pixel: Optional[tuple] = None
        self.collector: Optional[ProbeCollector] = None
        self._start_xyz = None
        self._misses = 0
        self._last_rgb = None
        self._det: dict = {}
        self.report: Optional[SolveReport] = None

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(12)
        intro = QLabel("Fix a high-contrast target mark (e.g. a small printed black-and-white checker or dot) to the effector at the tool centre point. "
                       "The camera must be connected with the stereo calibration done, and the robot homed with its home calibration done.")
        intro.setWordWrap(True); intro.setObjectName("muted")
        root.addWidget(intro)
        pills = QHBoxLayout()
        self.p_cam, self.p_stereo, self.p_robot = Pill("Camera"), Pill("Stereo calibration"), Pill("Robot ready")
        for p in (self.p_cam, self.p_stereo, self.p_robot):
            pills.addWidget(p)
        pills.addStretch(1)
        root.addLayout(pills)

        mid = QHBoxLayout()
        left = QVBoxLayout()
        self.hint = QLabel("1.  Click the middle of the target mark in the picture."); self.hint.setObjectName("muted")
        self.image = ClickableImage("Connect the camera (Camera page) to see the picture here", 420, 315)
        self.image.clicked.connect(self._clicked)
        self.probe_lbl = QLabel("probe: not set"); self.probe_lbl.setObjectName("mono")
        left.addWidget(self.hint); left.addWidget(self.image, 1); left.addWidget(self.probe_lbl)
        mid.addLayout(left, 1)

        right = card("Calibration points")
        rl = QVBoxLayout(right)
        self.table = QTableWidget(0, len(COLS))
        self.table.setHorizontalHeaderLabels(COLS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True); self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setMinimumHeight(200)
        rl.addWidget(self.table)
        r1, r2 = QHBoxLayout(), QHBoxLayout()
        self.suggest_btn = QPushButton("Suggest points"); self.suggest_btn.clicked.connect(self._suggest)
        self.go_btn = QPushButton("Go to selected point"); self.go_btn.clicked.connect(self._go)
        self.record_btn = QPushButton("Record point at current position"); self.record_btn.setObjectName("primary")
        self.record_btn.clicked.connect(self._record)
        for b in (self.suggest_btn, self.go_btn):
            r1.addWidget(b)
        r1.addWidget(self.record_btn)
        self.remove_btn = QPushButton("Remove selected"); self.remove_btn.clicked.connect(self._remove)
        self.clear_btn = QPushButton("Clear all"); self.clear_btn.clicked.connect(self._clear)
        self.solve_btn = QPushButton("Solve"); self.solve_btn.clicked.connect(self._solve)
        for b in (self.remove_btn, self.clear_btn, self.solve_btn):
            r2.addWidget(b)
        rl.addLayout(r1); rl.addLayout(r2)
        mid.addWidget(right, 1)
        root.addLayout(mid)

        res = card("Result")
        sl = QVBoxLayout(res)
        self.result = CheckLabel(); self.result.setText("Not solved yet.")
        self.warn_lbl = QLabel(""); self.warn_lbl.setWordWrap(True); self.warn_lbl.setObjectName("muted")
        sl.addWidget(self.result); sl.addWidget(self.warn_lbl)
        rb = QHBoxLayout()
        self.accept_btn = QPushButton("Accept and save"); self.accept_btn.setObjectName("primary"); self.accept_btn.setEnabled(False)
        self.accept_btn.clicked.connect(self._accept)
        self.forget_btn = QPushButton("Forget the saved calibration"); self.forget_btn.clicked.connect(self._forget)
        self.saved_lbl = QLabel(""); self.saved_lbl.setObjectName("muted")
        rb.addWidget(self.accept_btn); rb.addWidget(self.forget_btn); rb.addWidget(self.saved_lbl, 1)
        sl.addLayout(rb)
        root.addWidget(res)

        camera.video.connect(self._on_video)
        camera.detections.connect(self._on_detection)
        camera.status.connect(lambda *_: self._refresh())
        ctx.state.connect(self._refresh)
        ctx.settings_applied.connect(self._refresh)
        self.table.itemSelectionChanged.connect(self._refresh)
        self._refresh()

    # ------------------------------------------------------------------------------------------ status
    def _refresh(self, *_):
        r = self.robot
        cam_ok = self.camera.client.is_connected()
        stereo_ok = bool(self._det.get("calibrated")) and cam_ok
        robot_ok = r.connected and r.xyz_valid and not r.busy and not r.estopped
        self.p_cam.set("Camera connected" if cam_ok else "Camera not connected", "ok" if cam_ok else "bad")
        self.p_stereo.set("Stereo calibrated" if stereo_ok else "Stereo NOT calibrated", "ok" if stereo_ok else "bad")
        self.p_robot.set("Robot ready" if robot_ok else "Robot not ready", "ok" if robot_ok else "bad")
        row = self.table.currentRow()
        self.go_btn.setEnabled(r.connected and not r.busy and not r.estopped and bool(r.drivers_enabled) and r.xyz_valid and 0 <= row < len(self.rows))
        self.record_btn.setEnabled(robot_ok and self.collector is None)
        self.suggest_btn.setEnabled(True)
        self.remove_btn.setEnabled(0 <= row < len(self.rows))
        tf = self.ctx.calibration
        self.saved_lbl.setText(f"Saved calibration: {tf.n_points} points, {tf.rms_mm:.1f} mm RMS, {tf.date}" if tf.valid else "No calibration saved yet.")
        self.forget_btn.setEnabled(tf.valid)

    def _fill_table(self):
        self.table.setRowCount(len(self.rows))
        for i, row in enumerate(self.rows):
            rec = row["cam"] is not None
            cells = [str(i + 1)] + [f"{v:.1f}" for v in row["robot"]] + ([f"{v:.1f}" for v in row["cam"]] if rec else ["—"] * 3)
            cells += ["" if row["err"] is None else f"{row['err']:.1f} mm", "recorded" if rec else "planned"]
            for c, text in enumerate(cells):
                it = QTableWidgetItem(text); it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(i, c, it)
        self._refresh()

    # ------------------------------------------------------------------------------------------ picture + probe
    def _on_video(self, rgb, seq):
        self._last_rgb = rgb
        if not self.isVisible():
            return
        img = rgb
        if self.pixel is not None:
            img = draw_crosshair(rgb, self.pixel, "probe")
        self.image.set_image(to_qimage(img))

    def _clicked(self, u: int, v: int):
        self.pixel = (u, v)
        self.camera.client.send_probe(u, v)
        self.collector = None
        self.hint.setText("2.  Move the robot to a point and press 'Record point'. (Click again to pick another spot.)")
        self.probe_lbl.setText(f"probe pixel ({u}, {v}) — waiting for the Pi...")
        self._refresh()

    def hideEvent(self, ev):                      # noqa: N802
        if self.pixel is not None:
            self.camera.client.clear_probe()      # do not leave the Pi measuring when the wizard is not on screen
        super().hideEvent(ev)

    def showEvent(self, ev):                      # noqa: N802
        if self.pixel is not None:
            self.camera.client.send_probe(*self.pixel)
        super().showEvent(ev)

    def _on_detection(self, det: dict):
        self._det = det
        probe = det.get("probe")
        if probe and self.pixel and (probe.get("u"), probe.get("v")) == self.pixel:
            xyz = probe.get("xyz_mm")
            self.probe_lbl.setText("probe: " + ("no depth at this pixel" if not xyz else f"X {xyz[0]:.1f}  Y {xyz[1]:.1f}  Z {xyz[2]:.1f} mm"))
            if self.collector is not None:
                if xyz:
                    self.collector.feed(xyz)
                else:
                    self._misses += 1
                    if self._misses >= 4:
                        self._abort("The camera has no depth at that spot (too dark, no texture, or too close/far). Click a different "
                                    "part of the mark, or improve the light or the mark's contrast.")
                        return
                if self.collector is not None and self.collector.done:
                    self._finish()
        self._refresh()

    # ------------------------------------------------------------------------------------------ recording
    def _abort(self, why: str):
        self.collector = None
        self.record_btn.setText("Record point at current position")
        QMessageBox.information(self, "Record point", why)
        self._refresh()

    def _record(self):
        if self.pixel is None:
            QMessageBox.information(self, "Record point", "First click the middle of the target mark in the picture.")
            return
        if not self.camera.client.is_connected():
            QMessageBox.information(self, "Record point", "The camera is not connected.")
            return
        if not self._det.get("calibrated"):
            QMessageBox.information(self, "Record point", "The stereo camera is not calibrated: its depth values cannot be trusted yet.")
            return
        r = self.robot
        if not (r.connected and r.xyz_valid and not r.busy and not r.estopped):
            QMessageBox.information(self, "Record point", "The robot must be connected, homed and standing still.")
            return
        self.collector = ProbeCollector(8, 4.0)
        self._start_xyz = list(r.current_xyz)
        self._misses = 0
        self.record_btn.setText("Measuring…"); self.record_btn.setEnabled(False)

    def _finish(self):
        col, start = self.collector, self._start_xyz
        self.collector = None
        self.record_btn.setText("Record point at current position")
        if math.dist(start, self.robot.current_xyz) > 0.5:
            self._abort("The robot moved while measuring. Hold still and record again.")
            return
        ok, why = col.acceptable()
        if not ok:
            self._abort(why)
            return
        cam, spread = col.result()
        for row in self.rows:                                    # fill a planned point if we are standing on one
            if row["cam"] is None and math.dist(row["robot"], start) < 1.0:
                row.update(cam=cam, spread=spread, robot=tuple(start))
                break
        else:
            self.rows.append({"robot": tuple(start), "cam": cam, "err": None, "spread": spread})
        self.report = None
        self.accept_btn.setEnabled(False)
        self._fill_table()

    # ------------------------------------------------------------------------------------------ list management
    def _suggest(self):
        recorded = [r for r in self.rows if r["cam"] is not None]
        planned = suggest_points(self.robot.check_target, self.robot.workspace)
        new = [{"robot": p, "cam": None, "err": None, "spread": 0.0} for p in planned
               if not any(math.dist(p, r["robot"]) < 20 for r in recorded)]
        self.rows = recorded + new
        self._fill_table()

    def _go(self):
        row = self.table.currentRow()
        if not (0 <= row < len(self.rows)):
            return
        x, y, z = self.rows[row]["robot"]
        why = self.robot.check_target(x, y, z)
        if why:
            QMessageBox.information(self, "Go to point", why)
            return
        if QMessageBox.question(self, "Go to point", f"Move the effector to X {x:.1f}  Y {y:.1f}  Z {z:.1f} mm?") == QMessageBox.StandardButton.Yes:
            self.robot.move_xyz(x, y, z, source="calibration")

    def _remove(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.rows):
            del self.rows[row]
            self.report = None
            self.accept_btn.setEnabled(False)
            self._fill_table()

    def _clear(self):
        self.rows.clear(); self.report = None; self.accept_btn.setEnabled(False)
        self.result.setText("Not solved yet."); self.warn_lbl.setText("")
        self._fill_table()

    # ------------------------------------------------------------------------------------------ solve / save
    def _solve(self):
        rec = [r for r in self.rows if r["cam"] is not None]
        s = CameraCalibrationSession()
        for r in rec:
            s.add(r["robot"], r["cam"], 8, r["spread"])
        try:
            rep = s.solve()
        except ValueError as ex:
            self.report = None
            self.result.set(str(ex), False); self.warn_lbl.setText(""); self.accept_btn.setEnabled(False)
            return
        self.report = rep
        for r, e in zip(rec, rep.errors_mm):
            r["err"] = e
        good = rep.rms_mm <= 6.0 and not rep.warnings
        self.result.set(f"Solved from {len(rec)} points: error {rep.rms_mm:.2f} mm RMS, worst point {max(rep.errors_mm):.2f} mm.", good)
        self.warn_lbl.setText("\n".join("• " + w for w in rep.warnings))
        self.accept_btn.setEnabled(True)
        self._fill_table()

    def _accept(self):
        rep = self.report
        if rep is None:
            return
        if rep.rms_mm > 6.0 or rep.warnings:
            if QMessageBox.question(self, "Accept", f"The result has warnings (error {rep.rms_mm:.1f} mm). Save it anyway?") != QMessageBox.StandardButton.Yes:
                return
        self.ctx.settings.data["camera_to_robot"] = rep.transform.to_dict()
        self.ctx.settings.save()
        self.ctx.apply_settings()
        self.result.set(f"Saved. The Camera page now shows robot coordinates (error {rep.rms_mm:.2f} mm RMS).", True)

    def _forget(self):
        if QMessageBox.question(self, "Forget calibration", "Forget the saved camera-to-robot calibration?") == QMessageBox.StandardButton.Yes:
            self.ctx.settings.data["camera_to_robot"] = None
            self.ctx.settings.save()
            self.ctx.apply_settings()

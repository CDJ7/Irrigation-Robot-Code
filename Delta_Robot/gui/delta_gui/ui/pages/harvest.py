"""
harvest.py -- the Harvest page: pre-flight checklist, Start / Stop, live status, settings, and the pick log.

All the logic lives in core/harvest.py (tested without a screen). This page only shows it and asks for confirmation before a run.
The E-STOP (sidebar, or Esc) stays available and ends a run immediately.
"""
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (QCheckBox, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPushButton,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from ...core import harvest as H
from ..widgets import Badge, CheckLabel, card, ibox, spin

LEVEL = {H.IDLE: "idle", H.SEARCHING: "warn", H.RUNNING: "warn", H.STOPPING: "warn", H.DONE: "ok", H.STOPPED: "warn", H.ERROR: "bad"}


class HarvestPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.setObjectName("page")
        self.ctx, self.h = ctx, ctx.harvester
        grid = QGridLayout(self)
        grid.setContentsMargins(18, 14, 18, 14); grid.setSpacing(14)

        # ------------------------------------------------------------------ status + controls
        st = card("Automatic picking")
        sv = QVBoxLayout(st)
        top = QHBoxLayout()
        self.state_badge = Badge("IDLE", "idle")
        self.mode_badge = Badge("DRY RUN", "warn")
        top.addWidget(self.state_badge); top.addWidget(self.mode_badge); top.addStretch(1)
        self.message = QLabel(""); self.message.setWordWrap(True)
        self.counts = QLabel(""); self.counts.setObjectName("muted")
        self.blocked = QLabel(""); self.blocked.setObjectName("muted"); self.blocked.setWordWrap(True)
        btns = QHBoxLayout()
        self.start_btn = QPushButton("Start picking"); self.start_btn.setObjectName("primary"); self.start_btn.clicked.connect(self._start)
        self.stop_btn = QPushButton("Stop (finish move, vacuum off)"); self.stop_btn.clicked.connect(self.h.request_stop)
        btns.addWidget(self.start_btn); btns.addWidget(self.stop_btn)
        note = QLabel("The robot moves by itself during a run. Keep the area clear and a hand on the power switch. "
                      "EMERGENCY STOP (left, or Esc) ends the run immediately. It is a software stop, not a safety circuit.")
        note.setObjectName("muted"); note.setWordWrap(True)
        for w in (top, self.message, self.counts, self.blocked):
            (sv.addLayout if isinstance(w, QHBoxLayout) else sv.addWidget)(w)
        sv.addLayout(btns); sv.addWidget(note)
        grid.addWidget(st, 0, 0)

        # ------------------------------------------------------------------ checklist
        pf = card("Before you start")
        self.pf_layout = QVBoxLayout(pf)
        self.pf_labels = []
        for _ in range(12):
            lab = CheckLabel(); lab.setVisible(False)
            self.pf_layout.addWidget(lab); self.pf_labels.append(lab)
        grid.addWidget(pf, 0, 1)

        # ------------------------------------------------------------------ settings
        sc = card("Picking settings")
        sg = QGridLayout(sc)
        c = ctx.settings.data["harvest"]
        self.classes = QLineEdit(", ".join(c["allowed_classes"]))
        self.classes.setToolTip("Only these detector classes may ever be picked (comma separated).")
        self.conf = spin(0.0, 1.0, c["min_confidence"], 2, "", 0.05)
        self.frames = ibox(1, 50, c["stable_frames"], " frames")
        self.jitter = spin(0.5, 50, c["max_jitter_mm"], 1, " mm", 0.5)
        self.approach = spin(0, 300, c["approach_height_mm"], 0, " mm", 5)
        self.offset = [spin(-100, 100, v, 1, " mm", 1) for v in c["grasp_offset_mm"]]
        self.drop = [spin(-1000, 1000, v, 1, " mm", 5) for v in c["drop_xyz_mm"]]
        self.dwell = spin(0, 10, c["dwell_s"], 1, " s", 0.1)
        self.release = spin(0, 10, c["release_s"], 1, " s", 0.1)
        self.max_picks = ibox(1, 200, c["max_picks"], " picks")
        self.idle_to = spin(1, 600, c["idle_timeout_s"], 0, " s", 5)
        self.vac = QCheckBox("Use the vacuum (needs the new firmware; off = DRY RUN, moves only)")
        self.vac.setChecked(bool(c["vacuum_enabled"]))
        rows = [("Allowed classes", self.classes), ("Min confidence", self.conf), ("Stable for", self.frames),
                ("Max jitter", self.jitter), ("Hover height above berry", self.approach), ("Dwell with vacuum on", self.dwell),
                ("Wait after release", self.release), ("Max picks per run", self.max_picks), ("Give up after", self.idle_to)]
        for i, (name, w) in enumerate(rows):
            sg.addWidget(QLabel(name), i, 0); sg.addWidget(w, i, 1, 1, 3)
        r = len(rows)
        for label, boxes in (("Grasp offset X Y Z", self.offset), ("Drop point X Y Z", self.drop)):
            sg.addWidget(QLabel(label), r, 0)
            for j, b in enumerate(boxes):
                sg.addWidget(b, r, 1 + j)
            r += 1
        sg.addWidget(self.vac, r, 0, 1, 4)
        self.apply_btn = QPushButton("Apply and save"); self.apply_btn.setObjectName("primary"); self.apply_btn.clicked.connect(self._apply)
        self.apply_msg = QLabel(""); self.apply_msg.setObjectName("muted"); self.apply_msg.setWordWrap(True)
        sg.addWidget(self.apply_btn, r + 1, 0, 1, 2); sg.addWidget(self.apply_msg, r + 1, 2, 1, 2)
        grid.addWidget(sc, 1, 0)

        # ------------------------------------------------------------------ pick log
        lg = card("Pick log (this session)")
        lv = QVBoxLayout(lg)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["#", "Class", "Conf", "Robot XYZ (mm)", "Result"])
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setMinimumHeight(220)
        lv.addWidget(self.table)
        lv.addWidget(QLabel("Every attempt is also appended to harvest_log.jsonl next to the settings file."))
        grid.addWidget(lg, 1, 1)
        grid.setColumnStretch(0, 1); grid.setColumnStretch(1, 1); grid.setRowStretch(2, 1)

        self.h.listeners.append(self._on_harvest_event)
        ctx.state.connect(self._refresh)
        ctx.settings_applied.connect(self._refresh)
        self.timer = QTimer(self); self.timer.setInterval(300); self.timer.timeout.connect(self._refresh); self.timer.start()
        self._refresh()

    # ---------------------------------------------------------------------------------------------- display
    def _on_harvest_event(self, event, data):
        if event == "state":
            QTimer.singleShot(0, self._refresh)

    def _refresh(self, *_):
        h = self.h
        self.state_badge.set(h.state, LEVEL.get(h.state, "idle"))
        live = bool(self.ctx.settings.data["harvest"]["vacuum_enabled"])
        self.mode_badge.set("LIVE: vacuum on" if live else "DRY RUN: no vacuum", "bad" if live else "warn")
        self.message.setText(h.message)
        self.counts.setText(f"picks done: {h.picks_done}   skipped: {h.skipped}   candidates: {len(h.tracks)}")
        self.blocked.setText(f"Ignoring detections: {h.blocked_reason}" if h.blocked_reason and h.active else "")
        checks = h.preflight()
        for lab, (what, ok, detail) in zip(self.pf_labels, checks):
            lab.setVisible(True); lab.set(what if ok else f"{what}  —  {detail}", ok)
        for lab in self.pf_labels[len(checks):]:
            lab.setVisible(False)
        self.start_btn.setEnabled(not h.active and all(ok for _, ok, _ in checks))
        self.stop_btn.setEnabled(h.active and h.state != H.STOPPING)
        for w in (self.classes, self.conf, self.frames, self.jitter, self.approach, self.dwell, self.release, self.max_picks,
                  self.idle_to, self.vac, self.apply_btn, *self.offset, *self.drop):
            w.setEnabled(not h.active)
        self._fill_table()

    def _fill_table(self):
        log = self.h.pick_log
        if self.table.rowCount() == len(log):
            return
        self.table.setRowCount(len(log))
        for i, e in enumerate(log):
            vals = [str(e["n"]), e["class"], f"{e['confidence']:.2f}", "%.0f, %.0f, %.0f" % tuple(e["robot_xyz"]), e["result"]]
            for j, v in enumerate(vals):
                self.table.setItem(i, j, QTableWidgetItem(v))
        self.table.scrollToBottom()

    # ---------------------------------------------------------------------------------------------- actions
    def _apply(self):
        c = self.ctx.settings.data["harvest"]
        old = dict(c)
        c.update(allowed_classes=[t.strip() for t in self.classes.text().split(",") if t.strip()], min_confidence=self.conf.value(),
                 stable_frames=self.frames.value(), max_jitter_mm=self.jitter.value(), approach_height_mm=self.approach.value(),
                 grasp_offset_mm=[b.value() for b in self.offset], drop_xyz_mm=[b.value() for b in self.drop],
                 dwell_s=self.dwell.value(), release_s=self.release.value(), max_picks=self.max_picks.value(),
                 idle_timeout_s=self.idle_to.value(), vacuum_enabled=self.vac.isChecked())
        if self.vac.isChecked() and not old["vacuum_enabled"]:
            if QMessageBox.question(self, "Enable the vacuum", "Vacuum commands (V 1 / V 0) only work with the new firmware, which "
                                    "has not been written or flashed yet. With the current firmware every vacuum command fails and the "
                                    "run stops.\n\nEnable it anyway?") != QMessageBox.StandardButton.Yes:
                c.clear(); c.update(old); self.vac.setChecked(False)
                return
        try:
            self.ctx.settings.save()
            self.apply_msg.setText("Saved.")
        except ValueError as ex:
            c.clear(); c.update(old)
            self.apply_msg.setText(f"Not saved: {ex}")
        self._refresh()

    def _start(self):
        probs = self.h.problems()
        if probs:
            QMessageBox.warning(self, "Cannot start", "\n".join(probs))
            return
        c = self.ctx.settings.data["harvest"]
        mode = "LIVE: the vacuum will be switched on and off" if c["vacuum_enabled"] else "DRY RUN: the arms move but the vacuum is not used"
        if QMessageBox.question(self, "Start automatic picking",
                                f"The robot will now move BY ITSELF to up to {c['max_picks']} detected strawberries.\n\n{mode}.\n"
                                f"Classes: {', '.join(c['allowed_classes'])}\nDrop point: {c['drop_xyz_mm']}\n\n"
                                "• Keep hands and objects out of the workspace.\n• Be ready to press EMERGENCY STOP (Esc) or cut power.\n"
                                "• Picking has not been validated on this machine yet.\n\nStart?") != QMessageBox.StandardButton.Yes:
            return
        self.h.start()
        self._refresh()

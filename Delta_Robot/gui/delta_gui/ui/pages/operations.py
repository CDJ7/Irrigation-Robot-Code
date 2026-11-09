"""
operations.py -- the everyday page: where the robot is, move it, jog it, enable/disable the motors, home it.
Every button is enabled only when the robot controller would accept the command, and the page says WHY when it would not.
"""
import random

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget)

from ...core.link import SimulatedESP32Link
from ..widgets import Badge, CheckLabel, card, spin


class OperationsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.setObjectName("page")
        self.ctx = ctx
        self.robot = ctx.robot
        grid = QGridLayout(self)
        grid.setContentsMargins(18, 14, 18, 14)
        grid.setSpacing(14)

        # ------------------------------------------------------------- position
        pos = card("Position")
        pl = QVBoxLayout(pos)
        row = QHBoxLayout()
        self.axis_labels = []
        for name in ("X", "Y", "Z"):
            col = QVBoxLayout()
            cap = QLabel(name + "  (mm)"); cap.setObjectName("muted")
            val = QLabel("—"); val.setObjectName("readout")
            col.addWidget(cap); col.addWidget(val)
            val.setAlignment(Qt.AlignmentFlag.AlignLeft)
            row.addLayout(col)
            self.axis_labels.append(val)
        pl.addLayout(row)
        self.valid_badge = Badge("POSITION UNKNOWN", "bad")
        self.reason_lbl = QLabel(""); self.reason_lbl.setObjectName("muted"); self.reason_lbl.setWordWrap(True)
        self.steps_lbl = QLabel("steps: -"); self.steps_lbl.setObjectName("mono")
        pl.addWidget(self.valid_badge); pl.addWidget(self.reason_lbl); pl.addWidget(self.steps_lbl)
        grid.addWidget(pos, 0, 0)

        # ------------------------------------------------------------- motors
        mot = card("Motors")
        mv = QVBoxLayout(mot)
        ml = QHBoxLayout()
        self.enable_btn = QPushButton("Enable motors"); self.enable_btn.clicked.connect(self._enable)
        self.disable_btn = QPushButton("Disable motors"); self.disable_btn.clicked.connect(self._disable)
        self.home_btn = QPushButton("HOME"); self.home_btn.setObjectName("primary"); self.home_btn.clicked.connect(self._home)
        for b in (self.enable_btn, self.disable_btn, self.home_btn):
            ml.addWidget(b)
        self.motor_hint = QLabel(""); self.motor_hint.setObjectName("muted"); self.motor_hint.setWordWrap(True)
        mv.addLayout(ml)
        mv.addWidget(self.motor_hint)
        grid.addWidget(mot, 1, 0)

        # ------------------------------------------------------------- move to XYZ
        mv_card = card("Move to XYZ")
        ml2 = QGridLayout(mv_card)
        self.target = [spin(-1000, 1000, 0.0), spin(-1000, 1000, 0.0), spin(-1000, 1000, -280.0)]
        for i, (name, s) in enumerate(zip("XYZ", self.target)):
            ml2.addWidget(QLabel(name), 0, i * 2); ml2.addWidget(s, 0, i * 2 + 1)
            s.valueChanged.connect(self._refresh)
        self.move_btn = QPushButton("Move"); self.move_btn.setObjectName("primary"); self.move_btn.clicked.connect(self._move)
        ml2.addWidget(self.move_btn, 0, 6)
        self.verdict = CheckLabel()
        ml2.addWidget(self.verdict, 1, 0, 1, 7)
        grid.addWidget(mv_card, 0, 1)

        # ------------------------------------------------------------- jog
        jog = card("Jog")
        jl = QGridLayout(jog)
        jl.addWidget(QLabel("Step"), 0, 0)
        self.jog_step = spin(0.1, 100.0, 5.0, 1, " mm", 1.0)
        jl.addWidget(self.jog_step, 0, 1)
        self.jog_btns = []
        for i, (label, d) in enumerate((("X−", (-1, 0, 0)), ("X+", (1, 0, 0)), ("Y−", (0, -1, 0)),
                                        ("Y+", (0, 1, 0)), ("Z−", (0, 0, -1)), ("Z+", (0, 0, 1)))):
            b = QPushButton(label); b.setMinimumWidth(52)
            b.clicked.connect(lambda _=False, d=d: self._jog(d))
            jl.addWidget(b, 1 + i // 3, i % 3)
            self.jog_btns.append(b)
        grid.addWidget(jog, 1, 1)

        # ------------------------------------------------------------- explicit test mode (simulation only)
        self.test_card = card("Test mode — random targets (SIMULATION ONLY)")
        tl = QHBoxLayout(self.test_card)
        self.test_btn = QPushButton("Start random targets"); self.test_btn.setCheckable(True)
        self.test_btn.toggled.connect(self._toggle_test)
        note = QLabel("Sends the simulated robot to random points inside the workspace box, to exercise the motion code without "
                      "a camera. Not available with real hardware."); note.setObjectName("muted"); note.setWordWrap(True)
        tl.addWidget(self.test_btn); tl.addWidget(note, 1)
        grid.addWidget(self.test_card, 2, 0, 1, 2)
        self.test_timer = QTimer(self); self.test_timer.setInterval(1500); self.test_timer.timeout.connect(self._random_step)

        grid.setRowStretch(3, 1)
        grid.setColumnStretch(0, 1); grid.setColumnStretch(1, 1)
        ctx.state.connect(self._refresh)
        ctx.settings_applied.connect(self._refresh)
        self._refresh()

    # ---------------------------------------------------------------- state -> widgets
    def _can_move(self) -> bool:
        r = self.robot
        return r.connected and not r.estopped and not r.busy and bool(r.drivers_enabled) and r.xyz_valid

    def _refresh(self, *_):
        r = self.robot
        snap = r.snapshot()
        for lbl, v in zip(self.axis_labels, snap["xyz"]):
            lbl.setText(f"{v:8.1f}")
            lbl.setEnabled(snap["xyz_valid"])
        if snap["xyz_valid"]:
            self.valid_badge.set("POSITION VALID", "ok"); self.reason_lbl.setText("")
        else:
            self.valid_badge.set("POSITION UNKNOWN", "bad")
            self.reason_lbl.setText(f"{snap['invalid_reason']}. Enable the motors and press HOME.")
        self.steps_lbl.setText("steps: %d  %d  %d" % tuple(snap["steps"]))
        idle = r.connected and not r.estopped and not r.busy
        self.enable_btn.setEnabled(idle and not r.drivers_enabled)
        self.disable_btn.setEnabled(idle and bool(r.drivers_enabled))
        self.home_btn.setEnabled(idle and bool(r.drivers_enabled))
        hints = []
        if not r.connected: hints.append("Connect to the ESP32 (top bar).")
        elif r.estopped: hints.append("E-STOP is active. Release it (bottom left), then enable the motors.")
        elif not r.drivers_enabled: hints.append("Motors are off. Enable them first.")
        elif r.busy: hints.append(f"Working ({snap['state']}).")
        self.motor_hint.setText(" ".join(hints))
        x, y, z = (s.value() for s in self.target)
        why = r.check_target(x, y, z)
        self.verdict.set("Target allowed." if why is None else why, why is None)
        ok = self._can_move()
        self.move_btn.setEnabled(ok and why is None)
        for b in self.jog_btns:
            b.setEnabled(ok)
        sim = isinstance(self.ctx.link, SimulatedESP32Link)
        self.test_card.setVisible(sim)
        if not (sim and ok) and self.test_btn.isChecked() and not sim:
            self.test_btn.setChecked(False)

    # ---------------------------------------------------------------- actions
    def _enable(self):
        self.robot.enable(True)

    def _disable(self):
        if QMessageBox.question(self, "Disable motors", "Disabling the motors releases the arms; they may sag or fall.\n\n"
                                "The robot will also forget its position and need homing.\n\nDisable the motors?") \
                == QMessageBox.StandardButton.Yes:
            self.robot.enable(False)

    def _home(self):
        if self.ctx.settings.data["safety"]["confirm_homing"]:
            if QMessageBox.question(self, "Home the robot",
                                    "Homing moves all three arms until their limit switches trigger.\n\n"
                                    "• Keep the working area clear.\n• Be ready to cut motor power or press E-STOP.\n"
                                    "• Homing has not been validated on this machine yet.\n\nStart homing?") \
                    != QMessageBox.StandardButton.Yes:
                return
        self.robot.home()

    def _move(self):
        self.robot.move_xyz(*(s.value() for s in self.target))

    def _jog(self, d):
        step = self.jog_step.value()
        res = self.robot.jog(d[0] * step, d[1] * step, d[2] * step)
        if res.ok:                                  # keep the target boxes in step with where we are going
            for s, v in zip(self.target, (self.robot.current_xyz[0] + d[0] * step, self.robot.current_xyz[1] + d[1] * step,
                                          self.robot.current_xyz[2] + d[2] * step)):
                s.blockSignals(True); s.setValue(v); s.blockSignals(False)

    # ---------------------------------------------------------------- test mode (simulation only)
    def _toggle_test(self, on: bool):
        if on and not isinstance(self.ctx.link, SimulatedESP32Link):
            self.test_btn.setChecked(False)
            return
        self.test_btn.setText("Stop random targets" if on else "Start random targets")
        (self.test_timer.start if on else self.test_timer.stop)()

    def _random_step(self):
        if not isinstance(self.ctx.link, SimulatedESP32Link) or not self._can_move():
            return
        w = self.robot.workspace
        for _ in range(20):
            t = (random.uniform(*w.x), random.uniform(*w.y), random.uniform(*w.z))
            if self.robot.check_target(*t) is None:
                self.robot.move_xyz(*t, source="test")
                return

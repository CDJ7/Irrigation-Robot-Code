"""
debug.py -- raw motor control (bypasses the kinematics and the workspace box), the 'set current position as home' command and
the serial traffic. Using raw moves makes the XYZ position UNKNOWN until the robot is homed again (same rule as the original).
"""
from PyQt6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget

from ..widgets import card, ibox


class DebugPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.setObjectName("page")
        self.ctx, self.robot = ctx, ctx.robot
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(14)

        warn = QLabel("⚠  Raw motor control bypasses the kinematics and the workspace box. Using it makes the XYZ position "
                      "UNKNOWN: you must HOME the robot again before using XYZ moves.")
        warn.setWordWrap(True); warn.setObjectName("badge"); warn.setProperty("level", "warn")
        root.addWidget(warn)

        raw = card("Raw move (absolute motor steps)")
        g = QGridLayout(raw)
        self.steps = []
        for i in range(3):
            g.addWidget(QLabel(f"Axis {i + 1}"), 0, i * 2)
            s = ibox(-50_000_000, 50_000_000, 0, "", 100)
            g.addWidget(s, 0, i * 2 + 1); self.steps.append(s)
        g.addWidget(QLabel("Duration"), 1, 0)
        self.duration = ibox(1, 600_000, 800, " ms", 50)
        g.addWidget(self.duration, 1, 1)
        self.send_btn = QPushButton("Send raw move"); self.send_btn.setObjectName("warning"); self.send_btn.clicked.connect(self._send_raw)
        g.addWidget(self.send_btn, 1, 2, 1, 2)
        g.addWidget(QLabel("Jog by"), 2, 0)
        self.jog_amount = ibox(1, 1_000_000, 100, " steps", 10)
        g.addWidget(self.jog_amount, 2, 1)
        self.jog_btns = []
        for i in range(3):
            for j, sign in enumerate((-1, 1)):
                b = QPushButton(f"A{i + 1} {'−' if sign < 0 else '+'}")
                b.clicked.connect(lambda _=False, ax=i, sg=sign: self._jog(ax, sg))
                self.jog_btns.append(b)
        jr = QHBoxLayout()
        for b in self.jog_btns:
            jr.addWidget(b)
        g.addLayout(jr, 3, 0, 1, 6)
        root.addWidget(raw)

        misc = card("Position tools")
        ml = QHBoxLayout(misc)
        self.query_btn = QPushButton("Query position (P?)"); self.query_btn.clicked.connect(self.robot.query_position)
        self.zero_btn = QPushButton("Set current position as home (Z)"); self.zero_btn.setObjectName("warning")
        self.zero_btn.clicked.connect(self._zero)
        self.recover_btn = QPushButton("Recover position from steps"); self.recover_btn.clicked.connect(self._recover)
        self.recover_btn.setToolTip("Computes XYZ from the motor step counts after a stop. Needs the home calibration to be done.")
        for b in (self.query_btn, self.zero_btn, self.recover_btn):
            ml.addWidget(b)
        root.addWidget(misc)

        con = card("Serial traffic (commands sent and replies received)")
        cl = QVBoxLayout(con)
        self.console = QPlainTextEdit(); self.console.setReadOnly(True); self.console.setMaximumBlockCount(1000)
        cl.addWidget(self.console)
        root.addWidget(con, 1)

        ctx.log.connect(self._on_log)
        ctx.state.connect(self._refresh)
        self._refresh()

    def _on_log(self, text: str, level: str):
        if level == "tx":
            self.console.appendPlainText("→ " + text)
        elif level == "rx":
            self.console.appendPlainText("← " + text)

    def _refresh(self, *_):
        r = self.robot
        idle = r.connected and not r.estopped and not r.busy
        can_raw = idle and bool(r.drivers_enabled)
        self.send_btn.setEnabled(can_raw)
        for b in self.jog_btns:
            b.setEnabled(can_raw)
        self.query_btn.setEnabled(idle)
        self.zero_btn.setEnabled(idle)
        self.recover_btn.setEnabled(idle and not r.xyz_valid and self.ctx.settings.data["home"]["calibrated"])
        if not r.busy:
            for s, v in zip(self.steps, r.current_steps):
                s.blockSignals(True); s.setValue(int(v)); s.blockSignals(False)

    def _confirm_big(self, targets) -> bool:
        deg = self.robot.raw_move_size_deg(targets)
        limit = float(self.ctx.settings.data["safety"]["confirm_raw_move_deg"])
        if deg <= limit:
            return True
        return QMessageBox.question(self, "Large raw move", f"This moves an arm by about {deg:.0f} degrees with no limits applied.\n\n"
                                    "Make sure nothing is in the way.\n\nSend it?") == QMessageBox.StandardButton.Yes

    def _send_raw(self):
        targets = [s.value() for s in self.steps]
        if self._confirm_big(targets):
            self.robot.raw_move(targets, self.duration.value())

    def _jog(self, axis: int, sign: int):
        targets = list(self.robot.current_steps)
        targets[axis] += sign * self.jog_amount.value()
        if self._confirm_big(targets):
            self.robot.raw_move(targets, self.duration.value())

    def _zero(self):
        if QMessageBox.question(self, "Set position as home", "This tells the ESP32 that the arms are exactly at the HOME pose "
                                "right now, without moving. Only do this after checking the arms by eye.\n\nContinue?") \
                == QMessageBox.StandardButton.Yes:
            self.robot.set_zero()

    def _recover(self):
        res = self.robot.recover_position_from_steps()
        if not res.ok:
            QMessageBox.information(self, "Recover position", res.message)

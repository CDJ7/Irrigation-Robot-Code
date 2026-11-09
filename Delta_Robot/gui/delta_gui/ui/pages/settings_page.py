"""
settings_page.py -- program settings that are not about the machine's geometry: safety behaviour, motion limits, the settings file.
(The ESP32 connection and the simulation switch live in the top bar.)
"""
import copy

from PyQt6.QtWidgets import QCheckBox, QFormLayout, QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget

from ...core.settings import DEFAULTS
from ..widgets import card, ibox, spin


class SettingsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.setObjectName("page")
        self.ctx = ctx
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(14)

        saf = card("Safety")
        sf = QFormLayout(saf)
        self.estop_drivers = QCheckBox("E-STOP also switches the motor drivers off  (your \"motor off\" behaviour; arms may sag)")
        self.confirm_home = QCheckBox("Ask for confirmation before homing")
        self.confirm_raw = spin(0, 360, 10.0, 0, " deg", 1.0)
        self.reply_margin = spin(0.5, 60, 3.0, 1, " s", 0.5)
        self.home_timeout = spin(10, 1200, 240.0, 0, " s", 10)
        sf.addRow(self.estop_drivers)
        sf.addRow(self.confirm_home)
        sf.addRow("Confirm raw moves larger than", self.confirm_raw)
        sf.addRow("Time allowed beyond a move's duration for the reply", self.reply_margin)
        sf.addRow("Homing time limit", self.home_timeout)
        root.addWidget(saf)

        mot = card("Motion limits   (the firmware has no acceleration yet: start LOW)")
        mf = QFormLayout(mot)
        self.max_speed = spin(1, 2000, 80.0, 0, " mm/s", 5)
        self.max_rate = spin(100, 200000, 4000.0, 0, " steps/s", 100)
        mf.addRow("Maximum tip speed", self.max_speed)
        mf.addRow("Maximum step rate per axis", self.max_rate)
        hint = QLabel("A move takes whichever is longer: the time the tip speed needs, or the time the step rate needs. Raise the step "
                      "rate slowly; if a motor stalls or buzzes, lower it."); hint.setObjectName("muted"); hint.setWordWrap(True)
        mf.addRow(hint)
        root.addWidget(mot)

        conn = card("Connection")
        cf = QFormLayout(conn)
        self.baud = ibox(1200, 2_000_000, 115200)
        cf.addRow("Serial baud rate", self.baud)
        note = QLabel("The port, Connect button and the 'Simulated ESP32' switch are in the top bar."); note.setObjectName("muted")
        cf.addRow(note)
        root.addWidget(conn)

        fil = card("Settings file")
        fl = QVBoxLayout(fil)
        self.path_lbl = QLabel(str(ctx.settings.path)); self.path_lbl.setObjectName("mono"); self.path_lbl.setWordWrap(True)
        self.load_msg = QLabel(ctx.settings.load_message); self.load_msg.setObjectName("muted"); self.load_msg.setWordWrap(True)
        fl.addWidget(self.path_lbl); fl.addWidget(self.load_msg)
        row = QHBoxLayout()
        self.save_btn = QPushButton("Save settings"); self.save_btn.setObjectName("primary"); self.save_btn.clicked.connect(self._save)
        self.reset_btn = QPushButton("Reset everything to defaults"); self.reset_btn.clicked.connect(self._reset)
        self.msg = QLabel(""); self.msg.setObjectName("muted")
        row.addWidget(self.save_btn); row.addWidget(self.reset_btn); row.addWidget(self.msg, 1)
        fl.addLayout(row)
        root.addWidget(fil)
        root.addStretch(1)
        ctx.settings_applied.connect(self.load_from_settings)
        self.load_from_settings()

    def load_from_settings(self, *_):
        s = self.ctx.settings.data
        sf, dr = s["safety"], s["drive"]
        self.estop_drivers.setChecked(bool(sf["estop_disables_drivers"])); self.confirm_home.setChecked(bool(sf["confirm_homing"]))
        self.confirm_raw.setValue(sf["confirm_raw_move_deg"]); self.reply_margin.setValue(sf["reply_margin_s"])
        self.home_timeout.setValue(sf["home_timeout_s"])
        self.max_speed.setValue(dr["max_speed_mm_s"]); self.max_rate.setValue(dr["max_step_rate"])
        self.baud.setValue(int(s["connection"]["baud"]))

    def _save(self):
        s = self.ctx.settings.data
        s["safety"].update(estop_disables_drivers=self.estop_drivers.isChecked(), confirm_homing=self.confirm_home.isChecked(),
                           confirm_raw_move_deg=self.confirm_raw.value(), reply_margin_s=self.reply_margin.value(),
                           home_timeout_s=self.home_timeout.value())
        s["drive"].update(max_speed_mm_s=self.max_speed.value(), max_step_rate=self.max_rate.value())
        s["connection"]["baud"] = self.baud.value()
        try:
            self.ctx.settings.save()
        except ValueError as ex:
            QMessageBox.warning(self, "Cannot save", str(ex))
            return
        self.ctx.apply_settings()
        self.msg.setText("Saved.")

    def _reset(self):
        if self.ctx.robot.busy:
            QMessageBox.information(self, "Busy", "The robot is moving.")
            return
        if QMessageBox.question(self, "Reset", "Reset ALL settings and calibration values to the defaults?\n\nThis also forgets the "
                                "camera-to-robot calibration.") != QMessageBox.StandardButton.Yes:
            return
        self.ctx.settings.data = copy.deepcopy(DEFAULTS)
        self.ctx.settings.save()
        self.ctx.apply_settings()
        if self.ctx.robot.xyz_valid:
            self.ctx.robot._invalidate("settings were reset")
        self.msg.setText("Defaults restored.")

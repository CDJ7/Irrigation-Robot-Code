"""
main_window.py -- the window: header (ESP32 connection), sidebar (pages + the E-STOP), the pages, the system log, the status bar.

The E-STOP lives in the SIDEBAR, below the page buttons, so it is on screen whichever page is open. Esc triggers it from anywhere.
"""
from typing import List

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QKeySequence, QShortcut, QTextCharFormat, QColor
from PyQt6.QtWidgets import (QButtonGroup, QCheckBox, QComboBox, QDockWidget, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
                             QPlainTextEdit, QPushButton, QStackedWidget, QVBoxLayout, QWidget)

from .icons import make_icon
from .pages.calibration import CalibrationPage
from .pages.camera import CameraPage
from .pages.debug import DebugPage
from .pages.harvest import HarvestPage
from .pages.operations import OperationsPage
from .pages.settings_page import SettingsPage
from .theme import PALETTE
from .widgets import Pill, repolish, scrollable

PAGES = [("operations", "Operations"), ("camera", "Camera"), ("calibration", "Calibration"), ("harvest", "Harvest"), ("debug", "Debug"),
         ("settings", "Settings")]
LOG_COLOURS = {"info": PALETTE["text"], "warn": PALETTE["warn"], "error": PALETTE["danger"], "tx": "#7aa2f7", "rx": "#9ece6a"}


class MainWindow(QMainWindow):
    def __init__(self, ctx, camera):
        super().__init__()
        self.ctx, self.camera, self.robot = ctx, camera, ctx.robot
        self.setWindowTitle("Delta Robot — Strawberry Harvester")
        self.resize(1320, 860)
        self.setMinimumSize(1100, 720)
        central = QWidget(); self.setCentralWidget(central)
        outer = QVBoxLayout(central); outer.setContentsMargins(0, 0, 0, 0); outer.setSpacing(0)
        outer.addWidget(self._build_header())
        body = QHBoxLayout(); body.setContentsMargins(0, 0, 0, 0); body.setSpacing(0)
        body.addWidget(self._build_sidebar())
        self.stack = QStackedWidget()
        self.pages = {"operations": OperationsPage(ctx), "camera": CameraPage(ctx, camera), "calibration": CalibrationPage(ctx, camera), "harvest": HarvestPage(ctx),
                      "debug": DebugPage(ctx), "settings": SettingsPage(ctx)}
        for key, _ in PAGES:
            self.stack.addWidget(scrollable(self.pages[key]))
        body.addWidget(self.stack, 1)
        outer.addLayout(body, 1)
        self._build_log()
        self._build_status_bar()

        QShortcut(QKeySequence("Esc"), self, context=Qt.ShortcutContext.ApplicationShortcut).activated.connect(self.robot.estop)
        ctx.state.connect(self._on_state)
        ctx.log.connect(self._on_log)
        ctx.estop_changed.connect(self._on_estop)
        ctx.connection_changed.connect(self._on_connection)
        camera.status.connect(self._on_camera_status)
        camera.detections.connect(self._on_camera_det)
        camera.detections.connect(ctx.harvester.on_detections)
        self._show_page(ctx.settings.data["ui"].get("page", "operations"))
        self._on_state(self.robot.snapshot())
        self._on_camera_status({"state": "stopped", "message": ""})
        self._refresh_ports()
        self._on_log(ctx.settings.load_message, "info")

    # ------------------------------------------------------------------------------------------------ header
    def _build_header(self) -> QWidget:
        h = QWidget(); h.setObjectName("header"); h.setFixedHeight(58)
        lay = QHBoxLayout(h); lay.setContentsMargins(18, 0, 18, 0); lay.setSpacing(10)
        title = QLabel("Delta Robot"); title.setObjectName("h1")
        sub = QLabel("strawberry harvester"); sub.setObjectName("muted")
        lay.addWidget(title); lay.addWidget(sub); lay.addStretch(1)
        lay.addWidget(QLabel("ESP32"))
        self.port_combo = QComboBox(); self.port_combo.setMinimumWidth(170)
        self.refresh_btn = QPushButton("Refresh"); self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QPushButton("Connect"); self.connect_btn.setObjectName("primary"); self.connect_btn.clicked.connect(self._connect_clicked)
        self.sim_chk = QCheckBox("Simulated ESP32")
        self.sim_chk.setToolTip("Test mode: a fake ESP32 answers like the firmware. Nothing real moves.")
        self.sim_chk.setChecked(self.ctx.simulated)
        self.sim_chk.toggled.connect(self._sim_toggled)
        for w in (self.port_combo, self.refresh_btn, self.connect_btn, self.sim_chk):
            lay.addWidget(w)
        return h

    def _refresh_ports(self):
        self.port_combo.clear()
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:                                 # noqa: BLE001 (pyserial missing: simulation still works)
            ports = []
        self.port_combo.addItems(ports)
        saved = self.ctx.settings.data["connection"]["port"]
        if saved in ports:
            self.port_combo.setCurrentText(saved)
        self.port_combo.setEnabled(not self.ctx.simulated and not self.robot.connected)

    def _sim_toggled(self, on: bool):
        self.ctx.settings.data["connection"]["simulate_esp32"] = on
        self.port_combo.setEnabled(not on)
        try:
            self.ctx.settings.save()
        except ValueError:
            pass
        self.pages["operations"]._refresh()

    def _connect_clicked(self):
        if self.robot.connected or self.connect_btn.text() == "Connecting…":
            self.ctx.disconnect_esp32()
            return
        port = self.port_combo.currentText()
        if not self.ctx.simulated and not port:
            QMessageBox.warning(self, "No port", "Choose the ESP32's COM port first (press Refresh if it is not listed).")
            return
        self.ctx.settings.data["connection"]["port"] = port
        self.connect_btn.setText("Connecting…")
        self.ctx.connect_esp32(port, int(self.ctx.settings.data["connection"]["baud"]))

    def _on_connection(self, connected: bool):
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.sim_chk.setEnabled(not connected)
        self.port_combo.setEnabled(not connected and not self.ctx.simulated)
        self.pages["operations"]._refresh()

    # ------------------------------------------------------------------------------------------------ sidebar
    def _build_sidebar(self) -> QWidget:
        sb = QWidget(); sb.setObjectName("sidebar"); sb.setFixedWidth(210)
        lay = QVBoxLayout(sb); lay.setContentsMargins(12, 16, 12, 14); lay.setSpacing(6)
        self.nav = QButtonGroup(self); self.nav.setExclusive(True)
        self.nav_buttons = {}
        for i, (key, label) in enumerate(PAGES):
            b = QPushButton("  " + label); b.setObjectName("nav"); b.setCheckable(True)
            b.setIcon(make_icon(key)); b.setIconSize(b.iconSize() * 1.4)
            b.clicked.connect(lambda _=False, k=key: self._show_page(k))
            self.nav.addButton(b); lay.addWidget(b); self.nav_buttons[key] = b
        lay.addStretch(1)
        self.estop_btn = QPushButton("EMERGENCY STOP"); self.estop_btn.setObjectName("estop"); self.estop_btn.setMinimumHeight(92)
        self.estop_btn.setProperty("latched", False)
        self.estop_btn.setToolTip("Stops the motors immediately (Esc). Software stop only: keep a hand on the power switch.")
        self.estop_btn.clicked.connect(self._estop_clicked)
        hint = QLabel("Esc = emergency stop\nSoftware stop — not a safety-rated circuit."); hint.setObjectName("muted"); hint.setWordWrap(True)
        lay.addWidget(self.estop_btn); lay.addWidget(hint)
        return sb

    def _show_page(self, key: str):
        keys = [k for k, _ in PAGES]
        if key not in keys:
            key = "operations"
        self.stack.setCurrentIndex(keys.index(key))
        self.stack.widget(keys.index(key)).verticalScrollBar().setValue(0)
        self.nav_buttons[key].setChecked(True)
        self.ctx.settings.data["ui"]["page"] = key

    def _estop_clicked(self):
        if self.robot.estopped:
            if QMessageBox.question(self, "Release E-STOP", "Release the emergency stop?\n\nThe robot will not know where it is. Check that "
                                    "the area is clear, then enable the motors and home it.") == QMessageBox.StandardButton.Yes:
                self.robot.release_estop()
        else:
            self.robot.estop()

    def _on_estop(self, active: bool):
        self.estop_btn.setText("E-STOP ACTIVE\nclick to release" if active else "EMERGENCY STOP")
        self.estop_btn.setProperty("latched", active)
        repolish(self.estop_btn)

    # ------------------------------------------------------------------------------------------------ log
    def _build_log(self):
        self.log_view = QPlainTextEdit(); self.log_view.setReadOnly(True); self.log_view.setMaximumBlockCount(3000)
        dock = QDockWidget("System log", self)
        dock.setWidget(self.log_view)
        dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        dock.setMinimumHeight(130); dock.setMaximumHeight(190)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    def _on_log(self, text: str, level: str):
        fmt = QTextCharFormat(); fmt.setForeground(QColor(LOG_COLOURS.get(level, PALETTE["text"])))
        cur = self.log_view.textCursor()
        cur.movePosition(cur.MoveOperation.End)
        prefix = {"tx": "→ ", "rx": "← ", "warn": "[WARN] ", "error": "[ERROR] "}.get(level, "[INFO] ")
        cur.insertText(prefix + text + "\n", fmt)
        self.log_view.setTextCursor(cur)
        self.log_view.ensureCursorVisible()

    # ------------------------------------------------------------------------------------------------ status bar
    def _build_status_bar(self):
        sb = self.statusBar()
        self.pill_esp, self.pill_robot, self.pill_pos, self.pill_cam = Pill("ESP32 disconnected"), Pill("Robot"), Pill("XYZ unknown"), Pill("Camera")
        for p in (self.pill_esp, self.pill_robot, self.pill_pos, self.pill_cam):
            sb.addWidget(p)
        self.cam_fps = ""

    def _on_state(self, snap: dict):
        if snap["connected"]:
            self.pill_esp.set("ESP32 simulated" if self.ctx.simulated else "ESP32 connected", "ok")
        else:
            self.pill_esp.set("ESP32 disconnected", "idle")
        name = snap["state"]
        level = {"IDLE": "ok", "MOVING": "warn", "HOMING": "warn", "E-STOP": "bad", "POSITION UNKNOWN": "warn"}.get(name, "idle")
        self.pill_robot.set("Robot: " + name.title() if name != "E-STOP" else "Robot: E-STOP", level)
        x, y, z = snap["xyz"]
        if snap["xyz_valid"]:
            self.pill_pos.set(f"XYZ  {x:.1f}, {y:.1f}, {z:.1f} mm", "ok")
        else:
            self.pill_pos.set("XYZ unknown", "idle" if not snap["connected"] else "warn")
            self.pill_pos.setToolTip(snap["invalid_reason"])

    def _on_camera_status(self, st: dict):
        state = st.get("state", "")
        if state == "connected":
            self.pill_cam.set("Camera connected" + self.cam_fps, "ok")
        elif state == "connecting":
            self.pill_cam.set("Camera connecting…", "warn")
        elif state == "disconnected":
            self.pill_cam.set("Camera disconnected", "bad")
        else:
            self.pill_cam.set("Camera off", "idle")

    def _on_camera_det(self, det: dict):
        self.cam_fps = f"  {det.get('server_fps', 0):.1f} fps"
        if self.camera.client.is_connected():
            self.pill_cam.set("Camera connected" + self.cam_fps, "ok")

    # ------------------------------------------------------------------------------------------------ closing
    def closeEvent(self, ev):                    # noqa: N802
        try:
            self.pages["camera"].save_to_settings()
            self.ctx.settings.save()
        except Exception:                        # noqa: BLE001 (never block closing because of a settings problem)
            pass
        self.ctx.shutdown()
        self.camera.shutdown()
        super().closeEvent(ev)

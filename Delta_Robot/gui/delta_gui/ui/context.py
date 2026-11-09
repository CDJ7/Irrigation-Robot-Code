"""
context.py -- ties the Qt-free robot logic (core/) to the Qt interface.

Why it exists: the link's callbacks arrive on BACKGROUND threads, but the robot controller (and every widget) must only be touched
from the GUI thread. Qt signals solve that: a signal emitted from any thread is delivered to its slot on the GUI thread.
"""
from typing import Callable, Optional

from PyQt6.QtCore import QObject, QTimer, pyqtSignal

from ..core.link import LinkBase, SerialLink, SimulatedESP32Link
from ..core.robot import Robot
from ..core.settings import Settings
from ..core.transform import CameraToRobot


class AppContext(QObject):
    state = pyqtSignal(dict)            # robot snapshot (position, validity, state name, ...)
    log = pyqtSignal(str, str)          # text, level (info | warn | error | tx | rx)
    estop_changed = pyqtSignal(bool)
    connection_changed = pyqtSignal(bool)
    settings_applied = pyqtSignal()
    _line = pyqtSignal(str)             # internal: from the link thread
    _link_state = pyqtSignal(bool, str)

    def __init__(self, settings: Settings, link_factory: Optional[Callable[[], LinkBase]] = None):
        super().__init__()
        self.settings = settings
        self.robot = Robot(settings)
        self.link_factory = link_factory            # tests can inject a link
        self.link: Optional[LinkBase] = None
        self.robot.listeners.append(self._forward)
        self._line.connect(self.robot.on_line)      # queued across threads automatically
        self._link_state.connect(self.robot.on_link_state)
        self.watchdog = QTimer(self)
        self.watchdog.setInterval(200)
        self.watchdog.timeout.connect(self.robot.tick)
        self.watchdog.start()

    # --------------------------------------------------------------------------- robot -> Qt
    def _forward(self, event: str, data: dict) -> None:
        if event == "log":
            self.log.emit(data["text"], data["level"])
        elif event == "state":
            self.state.emit(self.robot.snapshot())
        elif event == "estop":
            self.estop_changed.emit(data["active"])
        elif event == "connection":
            self.connection_changed.emit(data["connected"])

    # --------------------------------------------------------------------------- connection
    @property
    def simulated(self) -> bool:
        return bool(self.settings.data["connection"]["simulate_esp32"])

    def connect_esp32(self, port: str, baud: int) -> None:
        self.disconnect_esp32(quiet=True)
        if self.link_factory is not None:
            self.link = self.link_factory()
        else:
            self.link = SimulatedESP32Link() if self.simulated else SerialLink()
        self.link.on_line = self._line.emit
        self.link.on_state = self._link_state.emit
        self.robot.attach(self.link)
        self.log.emit(("Connecting to the SIMULATED ESP32 (no hardware)..." if self.simulated else f"Connecting to {port} ..."), "info")
        self.link.connect("SIM" if self.simulated else port, baud)

    def disconnect_esp32(self, quiet: bool = False) -> None:
        if self.link is None:
            return
        link, self.link = self.link, None
        try:
            if self.robot.connected and self.robot.busy:
                link.send_line("STOP")              # never leave a move running unattended (as the original did on close)
        except ConnectionError:
            pass
        link.on_line = link.on_state = None
        link.disconnect()
        if self.robot.connected or not quiet:
            self.robot.on_link_state(False, "Disconnected from the ESP32.")

    def shutdown(self) -> None:
        self.watchdog.stop()
        self.disconnect_esp32(quiet=True)

    # --------------------------------------------------------------------------- settings
    def apply_settings(self) -> None:
        """Rebuild the kinematics etc. from the (already validated) settings. Refused while the robot is moving."""
        self.robot.apply_settings()
        self.settings_applied.emit()

    @property
    def calibration(self) -> CameraToRobot:
        return CameraToRobot.from_dict(self.settings.data.get("camera_to_robot"))

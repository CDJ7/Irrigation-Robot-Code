"""
Delta_Robot_GUI.py -- the Delta Robot control program (PyQt6).   Run:  python Delta_Robot_GUI.py

    --sim-esp32     start with the simulated ESP32 selected (nothing real can move)
    --settings F    use another settings file (default: ~/.delta_robot/settings.json)

The program is split so this file stays short:
    delta_gui/core/   robot logic with no GUI: kinematics, serial link (real + simulated), the safety controller, settings
    delta_gui/ui/     the interface: theme, pages, main window, camera controller
    camera_client.py  the stream from the Pi (plain Python)
The previous Tkinter program is kept as Delta_Robot_GUI_tkinter.py until this one has been checked on your PC.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-esp32", action="store_true")
    ap.add_argument("--settings", default=None)
    args = ap.parse_args()

    from PyQt6.QtWidgets import QApplication
    from delta_gui.core.settings import Settings
    from delta_gui.ui import theme
    from delta_gui.ui.camera_controller import CameraController
    from delta_gui.ui.context import AppContext
    from delta_gui.ui.main_window import MainWindow

    app = QApplication(sys.argv)
    app.setStyle("Fusion")                      # same look on every system; the stylesheet does the rest
    app.setStyleSheet(theme.stylesheet())
    settings = Settings(args.settings)
    settings.load()
    if args.sim_esp32:
        settings.data["connection"]["simulate_esp32"] = True
    ctx = AppContext(settings)
    camera = CameraController()
    win = MainWindow(ctx, camera)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

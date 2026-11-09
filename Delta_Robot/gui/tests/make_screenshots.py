"""
Takes the GUI screenshots used in the documentation, from the SIMULATED ESP32 and the SIMULATED camera (so the camera pictures are
a synthetic test pattern, not a real scene).   Run:  python tests/make_screenshots.py [output_folder]
"""
import os
import sys
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
os.environ["DELTA_ROBOT_SETTINGS"] = os.path.join(tempfile.mkdtemp(), "settings.json")
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(os.path.join(HERE, "..", "..", "docs", "screenshots"))
os.makedirs(OUT, exist_ok=True)

import numpy as np                                      # noqa: E402
from PyQt6.QtTest import QTest                           # noqa: E402
from PyQt6.QtWidgets import QApplication                 # noqa: E402

app = QApplication(sys.argv)
app.setStyle("Fusion")
from delta_gui.core.settings import Settings            # noqa: E402
from delta_gui.core.transform import CameraToRobot      # noqa: E402
from delta_gui.ui import theme                           # noqa: E402
from delta_gui.ui.camera_controller import CameraController  # noqa: E402
from delta_gui.ui.context import AppContext              # noqa: E402
from delta_gui.ui.main_window import MainWindow          # noqa: E402

app.setStyleSheet(theme.stylesheet())


def wait_for(pred, ms=15000):
    end = time.time() + ms / 1000
    while time.time() < end:
        QTest.qWait(30)
        if pred():
            return True
    return False


st = Settings(); st.load(); st.data["connection"]["simulate_esp32"] = True
ctx = AppContext(st); cam = CameraController(); win = MainWindow(ctx, cam); win.resize(1320, 860); win.show()
r = ctx.robot
win.connect_btn.click(); wait_for(lambda: r.connected)
r.enable(True); wait_for(lambda: r.drivers_enabled)
r.home(); wait_for(lambda: r.xyz_valid and not r.busy)
st.data["axes"]["offsets_deg"] = [-v for v in r.kin.inverse(*r.home_xyz)]
st.data["camera_to_robot"] = CameraToRobot(np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, 800.0]), True, 0.6, 7, "demo").to_dict()
ctx.apply_settings()
r.move_xyz(30, -20, -280); wait_for(lambda: not r.busy)

def shot(name):
    QTest.qWait(250)
    win.grab().save(os.path.join(OUT, name + ".png"))
    print("saved", name)

win._show_page("operations"); shot("operations")
win._show_page("camera")
win.pages["camera"].sim_chk.setChecked(True); win.pages["camera"].connect_btn.click()
wait_for(lambda: win.pages["camera"].table.rowCount() == 3 and win.pages["camera"].video._pix is not None)
win.pages["camera"].table.selectRow(0)
shot("camera")
for key in ("calibration", "harvest", "debug", "settings"):
    win._show_page(key); shot(key)
r.estop(); QTest.qWait(300); win._show_page("operations"); shot("estop_active")
win.close()

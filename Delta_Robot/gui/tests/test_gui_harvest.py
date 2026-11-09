"""
Drives the Harvest page through the real interface (offscreen Qt) with the SIMULATED ESP32 and detections injected as the camera
would deliver them. Run:  python tests/test_gui_harvest.py
"""
import os
import sys
import tempfile
import time
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
SETTINGS_FILE = os.path.join(tempfile.mkdtemp(), "settings.json")
os.environ["DELTA_ROBOT_SETTINGS"] = SETTINGS_FILE

import numpy as np                                                    # noqa: E402
from PyQt6.QtTest import QTest                                         # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox                  # noqa: E402

app = QApplication(sys.argv)
app.setStyle("Fusion")
from delta_gui.core.settings import Settings                           # noqa: E402
from delta_gui.core.transform import CameraToRobot                     # noqa: E402
from delta_gui.ui import theme                                         # noqa: E402
from delta_gui.ui.camera_controller import CameraController           # noqa: E402
from delta_gui.ui.context import AppContext                            # noqa: E402
from delta_gui.ui.main_window import MainWindow                        # noqa: E402

app.setStyleSheet(theme.stylesheet())
YES = QMessageBox.StandardButton.Yes
th = np.radians(25)
R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]]) @ np.diag([1.0, -1.0, -1.0])
TRUTH = CameraToRobot(R, np.array([10.0, -20.0, 800.0]), True, 0.3, 8)


def wait_for(pred, ms=15000):
    end = time.time() + ms / 1000
    while time.time() < end:
        QTest.qWait(20)
        if pred():
            return True
    return False


def main():
    st = Settings(SETTINGS_FILE); st.load()
    st.data["connection"]["simulate_esp32"] = True
    st.data["drive"].update(max_step_rate=60000.0, max_speed_mm_s=2000.0)
    st.data["home"]["calibrated"] = True
    st.data["harvest"].update(dwell_s=0.05, release_s=0.05, stable_frames=3, max_picks=2, idle_timeout_s=3.0)
    ctx = AppContext(st); cam = CameraController(); win = MainWindow(ctx, cam); win.show()
    r, h = ctx.robot, ctx.harvester
    page = win.pages["harvest"]
    patches = [mock.patch.object(QMessageBox, "question", return_value=YES), mock.patch.object(QMessageBox, "warning")]
    for p in patches:
        p.start()
    win._show_page("harvest")
    assert win.stack.currentWidget().widget() is page and win.estop_btn.isVisible()

    # nothing is ready: Start is disabled and the checklist says why
    texts = " | ".join(l.text() for l in page.pf_labels if l.isVisible())
    assert not page.start_btn.isEnabled() and "ESP32 connected" in texts and "Camera sending frames" in texts, texts
    assert page.mode_badge.text().startswith("DRY RUN")
    print("1. nothing ready: Start disabled; checklist lists what is missing; badge says DRY RUN OK")

    win.connect_btn.click(); assert wait_for(lambda: r.connected)
    r.enable(True); assert wait_for(lambda: r.drivers_enabled)
    r.home(); assert wait_for(lambda: r.xyz_valid and not r.busy)
    scene = [[30.0, 20.0, -290.0], [-40.0, 10.0, -300.0]]
    seq = [0]
    stock = [True]

    def feed():
        seq[0] += 1
        dets = [{"class": "strawberry", "confidence": 0.9, "allowed": True, "bbox": [1, 1, 9, 9], "center": [5, 5],
                 "depth_valid_fraction": 0.9, "xyz_mm": [float(v) for v in TRUTH.to_camera(p)]} for p in scene]
        h.on_detections({"seq": seq[0], "calibrated": True, "model": "yolo11n.pt" if stock[0] else "straw.pt",
                         "model_has_allowed_class": not stock[0], "allowed_classes": ["strawberry"], "detections": dets})
    from PyQt6.QtCore import QTimer
    feeder = QTimer(); feeder.setInterval(40); feeder.timeout.connect(feed); feeder.start()
    QTest.qWait(200)
    st.data["camera_to_robot"] = TRUTH.to_dict()
    QTest.qWait(400)
    texts = " | ".join(l.text() for l in page.pf_labels if l.isVisible())
    assert not page.start_btn.isEnabled() and "no allowed strawberry class" in texts, texts
    print("2. everything ready EXCEPT the model has no strawberry class (stock YOLO): Start stays disabled, reason shown OK")

    stock[0] = False
    QTest.qWait(500)
    assert page.start_btn.isEnabled(), " | ".join(l.text() for l in page.pf_labels if l.isVisible())
    page.start_btn.click()
    assert h.active and page.stop_btn.isEnabled() and not page.apply_btn.isEnabled()
    assert wait_for(lambda: h.state in ("SEARCHING", "PICKING"))
    # remove each berry once it has been picked, like the real world
    def drop_picked():
        while len(scene) > 2 - h.picks_done:
            scene.pop(0)
        return h.state in ("DONE", "ERROR", "STOPPED")
    ok = wait_for(drop_picked, 40000)
    assert ok, (h.state, h.message, h.picks_done, r.busy, len(h.tracks), [(t.hits, len(t.positions), round(t.jitter,1)) for t in h.tracks.values()], h.blocked_reason, h._last_seq, seq[0])
    assert h.state == "DONE" and h.picks_done == 2 and page.table.rowCount() == 2, (h.state, h.message, h.picks_done)
    assert page.state_badge.text() == "DONE" and "completed" in page.table.item(0, 4).text()
    assert page.apply_btn.isEnabled()
    print("3. Start (confirmed) -> 2 picks, table shows both attempts, state DONE, settings unlocked again OK")

    # E-STOP from the sidebar ends a run at once
    scene[:] = [[30.0, 20.0, -290.0]]
    st.data["harvest"]["max_picks"] = 5; st.data["harvest"]["dwell_s"] = 0.05
    r.home(); assert wait_for(lambda: r.xyz_valid and not r.busy)
    QTest.qWait(300)
    page.start_btn.click(); assert wait_for(lambda: h.state == "PICKING")
    win.estop_btn.click()
    assert h.state == "STOPPED" and r.estopped and page.state_badge.text() == "STOPPED"
    QTest.qWait(400)
    assert not page.start_btn.isEnabled() and "E-STOP" in " | ".join(l.text() for l in page.pf_labels if l.isVisible())
    assert any("Harvest halted by E-STOP" in win.log_view.toPlainText() for _ in [0])
    print("4. E-STOP button during a run: STOPPED immediately, Start disabled until released, logged OK")

    # settings: validation + vacuum warning
    with mock.patch.object(QMessageBox, "question", return_value=QMessageBox.StandardButton.No):
        page.vac.setChecked(True); page._apply()
    assert page.vac.isChecked() is False and st.data["harvest"]["vacuum_enabled"] is False
    page.drop[0].setValue(500.0); page._apply()
    assert page.mode_badge.text().startswith("DRY RUN")
    page.drop[0].setValue(90.0); page.classes.setText("strawberry, ripe"); page._apply()
    fresh = Settings(SETTINGS_FILE); fresh.load()
    assert fresh.data["harvest"]["allowed_classes"] == ["strawberry", "ripe"] and page.apply_msg.text() == "Saved."
    print("5. enabling the vacuum asks first (declined -> stays off); settings saved to file OK")

    feeder.stop(); win.close()
    for p in patches:
        p.stop()
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

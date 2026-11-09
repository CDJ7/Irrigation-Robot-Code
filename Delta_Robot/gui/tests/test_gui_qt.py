"""
Tests the PyQt6 GUI end to end WITHOUT hardware or a display: offscreen Qt, the simulated ESP32 and the simulated camera
(the real Pi server running in-process with a fake camera).
Run:  python tests/test_gui_qt.py          (needs PyQt6, numpy, opencv, PyYAML, pyserial)
On a headless Linux machine Qt may also need the system library libEGL; on Windows/macOS/desktop Linux nothing special is needed.
"""
import copy
import os
import sys
import tempfile
import threading
import time
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
SETTINGS_FILE = os.path.join(tempfile.mkdtemp(), "settings.json")
os.environ["DELTA_ROBOT_SETTINGS"] = SETTINGS_FILE

import numpy as np                                                    # noqa: E402
from PyQt6.QtCore import Qt                                            # noqa: E402
from PyQt6.QtTest import QTest                                         # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox                  # noqa: E402

app = QApplication(sys.argv)
app.setStyle("Fusion")
from delta_gui.core.settings import Settings                           # noqa: E402
from delta_gui.core.transform import CameraToRobot                     # noqa: E402
from delta_gui.ui import theme                                         # noqa: E402
from delta_gui.ui.camera_controller import CameraController           # noqa: E402
from delta_gui.ui.context import AppContext                            # noqa: E402
from delta_gui.ui.main_window import PAGES, MainWindow                 # noqa: E402

app.setStyleSheet(theme.stylesheet())
YES = QMessageBox.StandardButton.Yes
NO = QMessageBox.StandardButton.No


def wait_for(pred, ms=6000):
    end = time.time() + ms / 1000
    while time.time() < end:
        QTest.qWait(20)
        if pred():
            return True
    return False


def build(simulate=True):
    st = Settings(SETTINGS_FILE); st.load()
    st.data["connection"]["simulate_esp32"] = simulate
    ctx = AppContext(st)
    cam = CameraController()
    win = MainWindow(ctx, cam)
    win.show()
    return ctx, cam, win


def main():
    ctx, cam, win = build()
    r = ctx.robot
    thread_log = []
    r.listeners.append(lambda ev, d: thread_log.append(threading.current_thread() is threading.main_thread()))
    patches = [mock.patch.object(QMessageBox, "question", return_value=YES), mock.patch.object(QMessageBox, "information"),
               mock.patch.object(QMessageBox, "warning")]
    for p in patches:
        p.start()

    # ----------------------------------------------------------------- 1. structure + E-STOP on every page
    assert [k for k, _ in PAGES] == ["operations", "camera", "calibration", "harvest", "debug", "settings"]
    for key, _ in PAGES:
        win._show_page(key); QTest.qWait(30)
        b = win.estop_btn
        assert b.isVisibleTo(win) and b.height() >= 80 and win.rect().contains(b.mapTo(win, b.rect().center())), key
        assert win.stack.currentWidget().widget() is win.pages[key]
    print("1. five pages; the E-STOP is visible, large and on-screen on every page OK")

    # ----------------------------------------------------------------- 2. connect, enable, home, move (simulated ESP32)
    ops = win.pages["operations"]
    win._show_page("operations")
    assert not ops.move_btn.isEnabled() and win.pill_esp.text() == "ESP32 disconnected"
    win.connect_btn.click()
    assert wait_for(lambda: r.connected), "simulated ESP32 should connect"
    assert win.connect_btn.text() == "Disconnect" and win.pill_esp.text() == "ESP32 simulated"
    assert r.invalid_reason == "not homed since connecting", r.invalid_reason
    assert not ops.enable_btn.isEnabled() is False and ops.enable_btn.isEnabled()
    assert not ops.move_btn.isEnabled(), "no move before enabling and homing"
    ops.enable_btn.click(); assert wait_for(lambda: r.drivers_enabled)
    assert ops.home_btn.isEnabled() and not ops.move_btn.isEnabled()
    ops.home_btn.click()                                     # confirm dialog patched to Yes
    assert wait_for(lambda: r.xyz_valid and not r.busy, 15000), r.snapshot()
    assert win.pill_robot.text() == "Robot: Idle" and "XYZ" in win.pill_pos.text() and "unknown" not in win.pill_pos.text()
    print("2a. connect / enable / home through the real buttons OK; status pills:", win.pill_robot.text(), "|", win.pill_pos.text())
    # make the saved offsets consistent with HOME so the first move is small, then move through the UI
    ctx.settings.data["axes"]["offsets_deg"] = [-v for v in r.kin.inverse(*r.home_xyz)]
    ctx.apply_settings()
    ops.target[0].setValue(30.0); ops.target[1].setValue(-20.0); ops.target[2].setValue(-280.0)
    assert ops.move_btn.isEnabled() and "allowed" in ops.verdict.text()
    ops.move_btn.click()
    assert wait_for(lambda: r.busy) or True
    assert wait_for(lambda: not r.busy and r.current_xyz == [30.0, -20.0, -280.0], 20000), r.snapshot()
    assert "30.0" in ops.axis_labels[0].text() and "-280.0" in ops.axis_labels[2].text()
    ops.target[0].setValue(500.0)
    assert not ops.move_btn.isEnabled() and "outside the workspace" in ops.verdict.text()
    ops.target[0].setValue(30.0)
    ops._jog((1, 0, 0)); assert wait_for(lambda: not r.busy and abs(r.current_xyz[0] - 35.0) < 1e-6, 20000)
    print("2b. move + out-of-box verdict + jog through the UI OK ->", [round(v, 1) for v in r.current_xyz])

    # ----------------------------------------------------------------- 3. E-STOP: button and Esc key, latch and release
    ops._jog((0, 0, 10)); QTest.qWait(30)
    QTest.keyClick(win, Qt.Key.Key_Escape)
    assert r.estopped and "ACTIVE" in win.estop_btn.text() and win.estop_btn.property("latched") is True
    assert win.pill_robot.text() == "Robot: E-STOP" and not ops.move_btn.isEnabled() and not ops.home_btn.isEnabled()
    QTest.qWait(300)
    assert not r.xyz_valid and r.drivers_enabled is False
    win.estop_btn.click()                                    # release (confirm patched to Yes)
    assert not r.estopped and win.estop_btn.text() == "EMERGENCY STOP"
    print("3. Esc triggers E-STOP; latched state shown; release works; controls locked meanwhile OK")

    # ----------------------------------------------------------------- 4. thread safety
    assert all(thread_log) and len(thread_log) > 20, "robot logic must only run on the GUI thread"
    print(f"4. all {len(thread_log)} controller events ran on the GUI thread (link callbacks were handed over by Qt signals) OK")

    # ----------------------------------------------------------------- 5. debug page: big raw move needs confirmation
    ops.enable_btn.click(); wait_for(lambda: r.drivers_enabled)
    ops.home_btn.click(); assert wait_for(lambda: r.xyz_valid and not r.busy, 15000)
    dbg = win.pages["debug"]; win._show_page("debug")
    dbg.steps[0].setValue(40000)                             # ~96 degrees: above the 10 degree confirmation threshold
    with mock.patch.object(QMessageBox, "question", return_value=NO) as q:
        dbg.send_btn.click()
        assert q.called and not r.busy and r.xyz_valid, "declined confirmation must send nothing"
    dbg.steps[0].setValue(300)
    dbg.send_btn.click(); assert wait_for(lambda: not r.busy) and not r.xyz_valid
    tx = dbg.console.toPlainText()
    assert "→ M 300" in tx and "← OK" in tx
    print("5. raw move: confirmation required when large, XYZ invalidated, serial console shows traffic OK")

    # ----------------------------------------------------------------- 6. simulated camera -> pictures, detections, safety of 'Move above'
    cp = win.pages["camera"]; win._show_page("camera")
    cp.sim_chk.setChecked(True)
    cp.connect_btn.click()
    assert wait_for(lambda: cp.table.rowCount() == 3 and cp.video._pix is not None and cp.depth._pix is not None, 15000), \
        (cp.table.rowCount(), cp.status_pill.text())
    assert cp.status_pill.text() == "Connected" and "Camera connected" in win.pill_cam.text()
    classes = [cp.table.item(i, 0).text() for i in range(3)]
    allowed = [cp.table.item(i, 2).text() for i in range(3)]
    assert sorted(classes) == ["person", "strawberry", "strawberry"] and allowed.count("NO") == 1, (classes, allowed)
    assert "UNCALIBRATED" in cp.info.text() and not cp.move_btn.isEnabled()
    cp.table.selectRow(classes.index("strawberry")); QTest.qWait(50)
    assert not cp.move_btn.isEnabled() and ("not ready" in cp.why_lbl.text() or "calibration" in cp.why_lbl.text())
    print("6a. simulated camera: pictures + 3 detections (1 person flagged NOT allowed), move button locked. Info:", cp.info.text()[:70])

    cam.stop(); cp.connect_btn.setText("Connect")        # freeze the stream: the rows and the last J stay, nothing overwrites them
    # ... now give it a camera->robot calibration and a calibrated stereo flag, and check each remaining refusal in turn
    z_cam = [d["xyz_mm"][2] for d in cp._latest["detections"] if d["class"] == "strawberry" and d["xyz_mm"]][0]
    R = np.diag([1.0, -1.0, -1.0]); t = np.array([0.0, 0.0, -300.0 + z_cam])       # the strawberry (0,0,z_cam) -> robot (0,0,-300)
    ctx.settings.data["camera_to_robot"] = CameraToRobot(R, t, True, 0.5, 6, "test").to_dict()
    ctx.settings.data["axes"]["offsets_deg"] = [-v for v in r.kin.inverse(*r.home_xyz)]; ctx.apply_settings()
    ops.home_btn.click(); assert wait_for(lambda: r.xyz_valid and not r.busy, 15000)
    wait_for(lambda: cp.table.rowCount() == 3)
    cp.table.selectRow(classes.index("strawberry")); QTest.qWait(50)
    assert "stereo camera is not calibrated" in cp.why_lbl.text(), cp.why_lbl.text()
    cp._latest["calibrated"] = True; cp._update_actions()                           # (the fake camera is uncalibrated)
    rows = [cp._rows[i] for i in range(len(cp._rows))]
    assert rows[0]["robot_xyz"] is not None and abs(rows[0]["robot_xyz"][2] + 300.0) < 1.0, rows[0]
    straw = [i for i, d in enumerate(cp._rows) if d["class"] == "strawberry" and d["xyz_mm"]][0]
    person = [i for i, d in enumerate(cp._rows) if d["class"] == "person"][0]
    cp.table.selectRow(person); QTest.qWait(30); cp._update_actions()
    assert not cp.move_btn.isEnabled() and "not an allowed" in cp.why_lbl.text(), cp.why_lbl.text()
    cp.table.selectRow(straw); QTest.qWait(30); cp._update_actions()
    assert cp.move_btn.isEnabled(), cp.why_lbl.text()
    assert "robot" in cp.table.item(straw, 6).text() or cp.table.item(straw, 8).text() == "-300"
    cp.move_btn.click()
    assert wait_for(lambda: not r.busy and abs(r.current_xyz[2] + 240.0) < 1e-6, 25000), r.snapshot()
    print("6b. calibrated: person refused, strawberry accepted -> robot moved 60 mm ABOVE it:", [round(v, 1) for v in r.current_xyz])

    # ----------------------------------------------------------------- 7. calibration page apply / refuse / invalidate
    cal = win.pages["calibration"]; win._show_page("calibration")
    assert "264.0" in cal.sides.text() and "150,400" in cal.spr.text()
    cal.base_r.setValue(80.0)
    assert "277.1" in cal.sides.text()
    cal.fore.setValue(40.0); cal.eff_r.setValue(60.0)                                # forearm shorter than effector radius
    cal.apply_btn.click()
    assert ctx.settings.data["geometry"]["forearm_mm"] == 280.0, "invalid geometry must be refused"
    cal.load_from_settings(); cal.base_r.setValue(80.0)
    assert r.xyz_valid
    cal.apply_btn.click()
    assert ctx.settings.data["geometry"]["base_radius_mm"] == 80.0 and not r.xyz_valid and "geometry changed" in r.invalid_reason
    cal.ws["z"][0].setValue(-900.0)
    assert "unreachable" in cal.reach.text(), cal.reach.text()
    cal.ws["z"][0].setValue(-350.0)
    cal.micro.setValue(8); assert "75,200" in cal.spr.text()
    cal.load_from_settings()
    print("7. calibration page: radius->side conversion, steps/rev, refusal of bad geometry, apply invalidates position, box check OK")

    # ----------------------------------------------------------------- 7b. number boxes must really DRAW their value
    # (an earlier version passed every value check while the boxes were blank on screen: pixel-check them)
    win._show_page("calibration"); QTest.qWait(200)
    for box in (cal.base_r, cal.bicep, cal.micro):
        img = box.grab().toImage()
        bg = img.pixelColor(img.width() // 2, 3).lightness()
        bright = sum(1 for x in range(0, img.width(), 2) for y in range(img.height() // 3, 2 * img.height() // 3)
                     if img.pixelColor(x, y).lightness() > bg + 60)
        assert bright > 15, f"{box.__class__.__name__} shows no text (bright pixels: {bright})"
    print("7b. number boxes visibly draw their values (pixel check) OK")

    # ----------------------------------------------------------------- 7c. no page may squash its inputs at ANY window size
    # (the first real run on the user's PC showed Settings boxes crushed to 4 px in a shorter window; the pages now scroll instead)
    from PyQt6.QtWidgets import QAbstractSpinBox, QComboBox, QLineEdit, QPushButton
    for (w, h) in ((1100, 720), (1310, 713), (1920, 1040)):
        win.resize(w, h)
        for key, _ in PAGES:
            win._show_page(key); QTest.qWait(120)
            squashed = []
            for x in win.pages[key].findChildren((QAbstractSpinBox, QLineEdit, QComboBox, QPushButton)):
                if not x.isVisibleTo(win.pages[key]) or isinstance(x.parent(), QAbstractSpinBox):
                    continue                                   # (Qt's inner text field of a spin box is not a real input)
                need = 24 if not isinstance(x, QPushButton) else 22
                if x.height() < need:
                    squashed.append((type(x).__name__, x.height(), (getattr(x, "text", lambda: "")() or "")[:20]))
            assert not squashed, f"page {key} at {w}x{h}: squashed widgets {squashed[:4]}"
    win.resize(1320, 860); win._show_page("calibration")          # (the loop above ends on another page)
    print("7c. no input or button is squashed on any page at 1100x720, 1310x713 and 1920x1040 (pages scroll instead) OK")

    # ----------------------------------------------------------------- 8. wheel safety + test-mode visibility + settings file
    from PyQt6.QtGui import QWheelEvent
    from PyQt6.QtCore import QPoint, QPointF
    v0 = cal.bicep.value()
    ev = QWheelEvent(QPointF(5, 5), QPointF(5, 5), QPoint(0, 0), QPoint(0, 120), Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                     Qt.ScrollPhase.NoScrollPhase, False)
    cal.bicep.clearFocus(); QApplication.sendEvent(cal.bicep, ev)
    assert cal.bicep.value() == v0, "mouse wheel must not change an unfocused number box"
    assert ops.test_card.isVisibleTo(ops) is True
    win.close()
    s2 = Settings(SETTINGS_FILE); s2.load()
    assert s2.data["geometry"]["base_radius_mm"] == 80.0 and s2.data["ui"]["page"] == "calibration" and s2.data["camera"]["simulate"] is True
    assert s2.data["camera_to_robot"]["valid"] is True
    print("8. wheel ignored when unfocused; settings (geometry, page, camera mode, calibration) saved on close and reloaded OK")

    # ----------------------------------------------------------------- 9. real-hardware mode hides the random-target test mode
    ctx2, cam2, win2 = build(simulate=False)
    ctx2.robot.link = None
    assert not win2.pages["operations"].test_card.isVisibleTo(win2.pages["operations"]), "no random-target test mode without simulation"
    win2.close()
    for p in patches:
        p.stop()
    print("9. random-target test mode only exists with the simulated ESP32 OK")
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

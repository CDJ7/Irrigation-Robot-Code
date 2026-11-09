"""
Drives the two calibration wizards through the REAL interface (offscreen Qt, simulated ESP32, simulated camera):
  * home calibration: arm-direction nudges, computing offsets + home XYZ from measured angles, applying, consistency
  * camera -> robot calibration: suggested points, clicking the target, recording with averaging, solving against a KNOWN truth,
    saving, and every failure path (no click, uncalibrated stereo, no depth, noisy readings, robot moved, too few points, mirrored camera)
Run:  python tests/test_gui_wizards.py
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


def wait_for(pred, ms=8000):
    end = time.time() + ms / 1000
    while time.time() < end:
        QTest.qWait(20)
        if pred():
            return True
    return False


def build():
    st = Settings(SETTINGS_FILE); st.load()
    st.data["connection"]["simulate_esp32"] = True
    st.data["drive"].update(max_step_rate=60000.0, max_speed_mm_s=2000.0)      # fast simulated moves keep the test short
    ctx = AppContext(st); cam = CameraController(); win = MainWindow(ctx, cam); win.show()
    return ctx, cam, win


def main():
    ctx, cam, win = build()
    r = ctx.robot
    infos = []
    patches = [mock.patch.object(QMessageBox, "question", return_value=YES),
               mock.patch.object(QMessageBox, "information", side_effect=lambda *a, **k: infos.append(str(a[2]) if len(a) > 2 else "")),
               mock.patch.object(QMessageBox, "warning")]
    for p in patches:
        p.start()
    win.connect_btn.click(); assert wait_for(lambda: r.connected)
    r.enable(True); assert wait_for(lambda: r.drivers_enabled)
    r.home(); assert wait_for(lambda: r.xyz_valid and not r.busy, 15000)
    win._show_page("calibration")
    cal = win.pages["calibration"]; hw = cal.home_wizard; cw = cal.camera_wizard
    assert [cal.tabs.tabText(i) for i in range(3)] == ["Machine", "Home && axes", "Camera → robot"]

    # ================================================================= HOME WIZARD
    cal.tabs.setCurrentIndex(1)
    spd = ctx.settings.steps_per_deg
    assert all(b.isEnabled() for b in hw.nudge_btns) and not any(b.isEnabled() for b in hw.up_btns + hw.down_btns)
    hw.nudge_btns[0].click()                                          # confirm patched to Yes
    assert wait_for(lambda: not r.busy and r.current_steps[0] != 0), r.snapshot()
    assert r.current_steps[0] == round(3.0 * spd) and not r.xyz_valid, "a nudge is a raw move: position becomes unknown"
    assert hw.up_btns[0].isEnabled() and hw.down_btns[0].isEnabled()
    hw.up_btns[0].click()                                             # "it moved UP"  -> this arm is inverted
    assert cal.inv[0].isChecked() and "INVERTED" in hw.dir_lbls[0].text() and not hw.up_btns[0].isEnabled()
    hw.nudge_btns[1].click(); assert wait_for(lambda: not r.busy and r.current_steps[1] != 0)
    hw.down_btns[1].click()                                           # "it moved DOWN" -> direction is right
    assert not cal.inv[1].isChecked() and "correct" in hw.dir_lbls[1].text()
    print("H1. arm-direction nudges: arm 1 answered UP -> Invert ticked; arm 2 answered DOWN -> not inverted OK")

    r.home(); assert wait_for(lambda: r.xyz_valid and not r.busy, 15000)
    for s_ in hw.angles:
        s_.setValue(-60.0)                                            # impossible: outside the arm-angle limits
    hw.compute_btn.click()
    assert not hw.use_btn.isEnabled() and "allowed range" in hw.result_lbl.text(), hw.result_lbl.text()
    for s_ in hw.angles:
        s_.setValue(-22.0)
    hw.compute_btn.click()
    assert hw.use_btn.isEnabled() and "Computed" in hw.result_lbl.text() and "22.00" in hw.detail_lbl.text(), hw.detail_lbl.text()
    hw.use_btn.click()
    assert all(abs(o.value() - 22.0) < 1e-6 for o in cal.offs) and cal.home_cal.isChecked() and -175 < cal.home_xyz[2].value() < -165
    cal.inv[0].setChecked(False)                                      # (the simulated arms are not really inverted: keep consistent)
    cal.apply_btn.click()
    d = ctx.settings.data
    assert d["axes"]["offsets_deg"] == [22.0, 22.0, 22.0] and d["home"]["calibrated"] is True
    assert r.home_consistency() is None, r.home_consistency()
    print("H2. measured angles -> offsets 22 deg, home XYZ", [round(v, 1) for v in d["home"]["xyz_mm"]], "saved; controller says consistent OK")
    r.enable(True) if not r.drivers_enabled else None
    wait_for(lambda: r.drivers_enabled)
    r.home(); assert wait_for(lambda: r.xyz_valid and not r.busy, 15000)

    # ================================================================= CAMERA WIZARD
    cal.tabs.setCurrentIndex(2); QTest.qWait(100)
    cp = win.pages["camera"]
    cp.sim_chk.setChecked(True); cp.connect_btn.click()
    assert wait_for(lambda: cam.client.is_connected() and cw._det.get("seq"), 15000)
    assert cw.p_cam.text() == "Camera connected" and cw.p_stereo.text() == "Stereo NOT calibrated" and cw.p_robot.text() == "Robot ready"
    cw._record(); assert infos and "click the middle" in infos[-1]
    cw.image.clicked.emit(160, 120)                                   # click the mark
    assert cam.client._probe == (160, 120)
    assert wait_for(lambda: "X " in cw.probe_lbl.text(), 8000), cw.probe_lbl.text()
    cw._record(); assert "not calibrated" in infos[-1], infos[-1]
    print("C1. status pills, no-click refusal, click sends the probe and the live reading appears, uncalibrated stereo refused OK")

    cam.stop()                                                         # freeze the stream; from here we feed measurements ourselves
    cam.client.is_connected = lambda: True                              # (the wizard needs a 'connected' camera)
    th = np.radians(25)
    R = np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]]) @ np.diag([1.0, -1.0, -1.0])
    t = np.array([10.0, -20.0, 800.0])
    truth = CameraToRobot(R, t, True)
    rng = np.random.default_rng(11)

    def feed(xyz_cam, n=8, probe_ok=True, pixel=(160, 120)):
        for _ in range(n):
            xyz = None if not probe_ok else [float(v) for v in np.asarray(xyz_cam) + rng.normal(0, 0.4, 3)]
            cw._on_detection({"seq": 1, "calibrated": True, "probe": {"u": pixel[0], "v": pixel[1], "xyz_mm": xyz}})

    cw._det = {"calibrated": True}
    cw._suggest()
    assert len(cw.rows) == 8 and all(row["cam"] is None for row in cw.rows) and cw.table.rowCount() == 8
    pts = [row["robot"] for row in cw.rows]
    for p in pts:
        assert r.check_target(*p) is None
    # failure paths ---------------------------------------------------------------------------------------
    r.move_xyz(*pts[0]); assert wait_for(lambda: not r.busy and r.xyz_valid, 15000)
    cw._record(); assert cw.collector is not None
    feed((0, 0, 700), n=4, probe_ok=False)                             # no depth at the clicked spot
    assert cw.collector is None and "no depth" in infos[-1], infos[-1]
    cw._record()
    for z in (700, 725, 675, 700, 740, 660, 700, 700):                 # readings jumping by tens of mm
        cw._on_detection({"seq": 1, "calibrated": True, "probe": {"u": 160, "v": 120, "xyz_mm": [0.0, 0.0, float(z)]}})
    assert cw.collector is None and "jump around" in infos[-1] and not [x for x in cw.rows if x["cam"]]
    cw._record(); feed((0, 0, 700), n=3)
    r.jog(5, 0, 0); assert wait_for(lambda: not r.busy)                # the robot moves mid-measurement
    feed((0, 0, 700), n=5)
    assert cw.collector is None and "moved while measuring" in infos[-1]
    print("C2. failure paths: no depth, jumpy readings, robot moved during the measurement all refused with a plain message OK")
    # success: record every suggested point with readings from the TRUE transform --------------------------------
    for p in pts:
        r.move_xyz(*p); assert wait_for(lambda: not r.busy and r.xyz_valid, 15000), r.snapshot()
        cw._record(); assert cw.collector is not None
        feed(truth.to_camera(r.current_xyz))
        assert cw.collector is None
    rec = [x for x in cw.rows if x["cam"]]
    assert len(rec) == 8 and len(cw.rows) == 8, (len(rec), len(cw.rows))
    cw._solve()
    assert cw.report and cw.report.rms_mm < 1.5 and cw.accept_btn.isEnabled() and "Solved from 8 points" in cw.result.text(), cw.result.text()
    assert np.allclose(cw.report.transform.R, R, atol=0.01) and np.allclose(cw.report.transform.t, t, atol=2.0)
    assert all(e < 2.0 for e in [float(cw.table.item(i, 7).text().split()[0]) for i in range(8)])
    cw._accept()
    assert ctx.calibration.valid and np.allclose(ctx.calibration.R, R, atol=0.01) and "Saved" in cw.result.text()
    assert "8 points" in cw.saved_lbl.text()
    print(f"C3. 8 points recorded, solved (RMS {cw.report.rms_mm:.2f} mm), rotation/translation match the truth, saved OK")

    # the Camera page now turns a camera XYZ into the right robot XYZ
    probe_cam = truth.to_camera([20.0, 10.0, -280.0])
    cp._on_detections({"seq": 77, "calibrated": True, "detections": [{"class": "strawberry", "confidence": 0.9, "allowed": True,
                                                                      "xyz_mm": [float(v) for v in probe_cam], "bbox": [1, 1, 5, 5]}]})
    got = [float(cp.table.item(0, c).text()) for c in (6, 7, 8)]
    assert all(abs(a - b) < 1.5 for a, b in zip(got, (20.0, 10.0, -280.0))), got
    print("C4. the Camera page now shows robot XYZ", got, "for a berry at (20, 10, -280) OK")

    # list management + solve refusals + mirrored camera -------------------------------------------------------------
    cw.table.selectRow(2); cw._remove(); assert len(cw.rows) == 7 and cw.report is None and not cw.accept_btn.isEnabled()
    cw._clear(); assert not cw.rows
    cw.rows = [{"robot": p, "cam": tuple(truth.to_camera(p)), "err": None, "spread": 0.0} for p in pts[:3]]
    cw._solve(); assert "at least 4" in cw.result.text() and not cw.accept_btn.isEnabled()
    cw.rows = [{"robot": p, "cam": (-truth.to_camera(p)[0], truth.to_camera(p)[1], truth.to_camera(p)[2]), "err": None, "spread": 0.0} for p in pts]
    cw._solve()
    assert "MIRRORED" in cw.warn_lbl.text(), cw.warn_lbl.text()
    print("C5. remove/clear, 'need 4 points', and a mirrored camera frame diagnosed in plain words OK")

    # the probe is released when the wizard is not on screen and re-sent when it comes back ----------------------------
    cam.client._send_line = lambda *_a: None
    cal.tabs.setCurrentIndex(0); QTest.qWait(50)
    assert cam.client._probe is None, "leaving the wizard must clear the probe"
    cal.tabs.setCurrentIndex(2); QTest.qWait(50)
    cw.pixel = (10, 10); cal.tabs.setCurrentIndex(0); cal.tabs.setCurrentIndex(2); QTest.qWait(50)
    assert cam.client._probe == (10, 10)
    print("C6. probe cleared when leaving the tab and re-sent when returning OK")

    # forget -------------------------------------------------------------------------------------------------------
    cw._forget(); assert not ctx.calibration.valid and Settings(SETTINGS_FILE).data["camera_to_robot"] is None
    win.close()
    for p in patches:
        p.stop()
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()

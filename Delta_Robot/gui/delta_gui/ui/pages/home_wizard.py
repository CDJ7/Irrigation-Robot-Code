"""
home_wizard.py -- the guided HOME calibration (steps 1 and 2 of the calibration):

  1. ARM DIRECTIONS  nudge each arm a few degrees with positive steps; you say whether it moved DOWN or UP. (Sets "Invert N".)
  2. HOME ANGLES     after homing, measure each upper arm against horizontal with an angle gauge; the program computes the axis
                     offsets and, with forward kinematics, the HOME XYZ.
It only fills in the fields of the Home & axes tab; nothing is stored until "Apply and save".
"""
from PyQt6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget)

from ...core.calibration import home_calibration, invert_from_answer, nudge_steps
from ..widgets import CheckLabel, card, spin


class HomeWizard(QWidget):
    def __init__(self, ctx, calpage):
        super().__init__()
        self.ctx, self.robot, self.cal = ctx, ctx.robot, calpage
        self._result = None
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(12)

        # --------------------------------------------------------------------------------- step 1
        c1 = card("Step 1 — Arm directions")
        g = QGridLayout(c1)
        intro = QLabel("A POSITIVE step must swing an arm DOWN (toward the table). The robot nudges one arm at a time; look at it and answer. "
                       "Keep clear of the arms. Motors must be enabled. (A nudge makes the position unknown: home again afterwards.)")
        intro.setWordWrap(True); intro.setObjectName("muted")
        g.addWidget(intro, 0, 0, 1, 6)
        g.addWidget(QLabel("Nudge size"), 1, 0)
        self.nudge_deg = spin(0.5, 10.0, 3.0, 1, " deg", 0.5)
        g.addWidget(self.nudge_deg, 1, 1)
        self.nudge_btns, self.up_btns, self.down_btns, self.dir_lbls = [], [], [], []
        for i in range(3):
            n = QPushButton(f"Nudge arm {i + 1}"); n.clicked.connect(lambda _=False, i=i: self._nudge(i))
            dn = QPushButton("It moved DOWN"); dn.clicked.connect(lambda _=False, i=i: self._answer(i, True))
            up = QPushButton("It moved UP"); up.clicked.connect(lambda _=False, i=i: self._answer(i, False))
            lbl = QLabel("—"); lbl.setObjectName("muted")
            for b in (dn, up):
                b.setEnabled(False)
            g.addWidget(n, 2 + i, 0); g.addWidget(dn, 2 + i, 1); g.addWidget(up, 2 + i, 2); g.addWidget(lbl, 2 + i, 3, 1, 3)
            self.nudge_btns.append(n); self.down_btns.append(dn); self.up_btns.append(up); self.dir_lbls.append(lbl)
        root.addWidget(c1)

        # --------------------------------------------------------------------------------- step 2
        c2 = card("Step 2 — Home angles")
        g2 = QGridLayout(c2)
        txt = QLabel("Press HOME (Operations page). Then measure the angle of each UPPER ARM (bicep) against horizontal with a digital angle "
                     "gauge. Positive = swung DOWN below horizontal, negative = raised ABOVE horizontal. At the limit switches the arms are "
                     "usually raised, so expect negative numbers. All three are normally the same.")
        txt.setWordWrap(True); txt.setObjectName("muted")
        g2.addWidget(txt, 0, 0, 1, 6)
        self.angles = [spin(-90, 90, -20.0, 2, " deg", 0.1) for _ in range(3)]
        for i, s in enumerate(self.angles):
            g2.addWidget(QLabel(f"Arm {i + 1}"), 1, i * 2); g2.addWidget(s, 1, i * 2 + 1)
        self.compute_btn = QPushButton("Compute offsets and home position"); self.compute_btn.clicked.connect(self._compute)
        g2.addWidget(self.compute_btn, 2, 0, 1, 3)
        self.use_btn = QPushButton("Use these values"); self.use_btn.setObjectName("primary"); self.use_btn.setEnabled(False)
        self.use_btn.clicked.connect(self._use)
        g2.addWidget(self.use_btn, 2, 3, 1, 3)
        self.result_lbl = CheckLabel(); g2.addWidget(self.result_lbl, 3, 0, 1, 6)
        self.detail_lbl = QLabel(""); self.detail_lbl.setObjectName("mono"); self.detail_lbl.setWordWrap(True)
        g2.addWidget(self.detail_lbl, 4, 0, 1, 6)
        root.addWidget(c2)
        ctx.state.connect(self._refresh)
        self._refresh()

    def _refresh(self, *_):
        r = self.robot
        ready = r.connected and not r.estopped and not r.busy and bool(r.drivers_enabled)
        for b in self.nudge_btns:
            b.setEnabled(ready)

    # ---------------------------------------------------------------------------------------------
    def _nudge(self, arm: int):
        deg = self.nudge_deg.value()
        steps = nudge_steps(deg, self.robot.steps_per_deg)
        if QMessageBox.question(self, f"Nudge arm {arm + 1}", f"Arm {arm + 1} will move by about {deg:.1f} degrees ({steps} steps).\n\n"
                                "Keep your hands clear. Continue?") != QMessageBox.StandardButton.Yes:
            return
        target = list(self.robot.current_steps)
        target[arm] += steps
        res = self.robot.raw_move(target, max(400, int(abs(steps) / max(float(self.ctx.settings.data['drive']['max_step_rate']), 1) * 1000) + 200))
        if res.ok:
            self.dir_lbls[arm].setText("Did it move down or up?")
            self.down_btns[arm].setEnabled(True); self.up_btns[arm].setEnabled(True)
        else:
            QMessageBox.information(self, "Nudge", res.message)

    def _answer(self, arm: int, moved_down: bool):
        inv = invert_from_answer(moved_down)
        self.cal.inv[arm].setChecked(inv)
        self.dir_lbls[arm].setText("direction is correct (not inverted)" if not inv else "direction is INVERTED (Invert box ticked)")
        self.down_btns[arm].setEnabled(False); self.up_btns[arm].setEnabled(False)
        self.cal.msg.setText("Direction recorded (not saved yet). Home the robot again, then continue with step 2.")

    def _compute(self):
        k = self.cal._kin()
        try:
            res = home_calibration([s.value() for s in self.angles], k, self.cal.tmin.value(), self.cal.tmax.value(), self.cal._box())
        except ValueError as ex:
            self._result = None
            self.use_btn.setEnabled(False)
            self.result_lbl.set(str(ex), False)
            self.detail_lbl.setText("")
            return
        self._result = res
        x, y, z = res.home_xyz
        self.result_lbl.set("Computed." + ("  Notes: " + " ".join(res.warnings) if res.warnings else ""), not res.warnings)
        self.detail_lbl.setText("offsets (deg): " + "  ".join(f"{v:.2f}" for v in res.offsets_deg) + f"\nhome XYZ (mm): {x:.1f}  {y:.1f}  {z:.1f}")
        self.use_btn.setEnabled(True)

    def _use(self):
        if self._result is None:
            return
        for i in range(3):
            self.cal.offs[i].setValue(self._result.offsets_deg[i])
        for i in range(3):
            self.cal.home_xyz[i].setValue(self._result.home_xyz[i])
        self.cal.home_cal.setChecked(True)
        self.cal.msg.setText("Offsets and home position filled in (not saved yet): press 'Apply and save'.")

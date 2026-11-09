"""
calibration.py -- everything that describes the physical machine, in one place.

  * geometry: the RADII and arm lengths you measure (the page shows the triangle side lengths the formulas use)
  * motors:   steps per revolution = motor full steps x microsteps x gearbox ratio, as three separate fields
  * axes:     angle offsets, direction, allowed arm-angle range
  * home:     the XYZ the robot is assumed to be at after homing, and a check that it agrees with the axis offsets
  * workspace box, with a live check that the whole box is reachable
  * the guided home calibration and the guided camera-to-robot calibration (wizards)
Nothing takes effect until "Apply and save" (which is refused while the robot is moving).
"""
import copy
import itertools

from PyQt6.QtWidgets import (QCheckBox, QFormLayout, QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPushButton, QTabWidget,
                             QVBoxLayout, QWidget)

from ...core.kinematics import DeltaKinematics, Workspace, joint_limit_violation, radius_to_side
from ...core.settings import validate
from ..widgets import CheckLabel, card, ibox, spin
from .camera_wizard import CameraWizard
from .home_wizard import HomeWizard


class CalibrationPage(QWidget):
    def __init__(self, ctx, camera):
        super().__init__()
        self.setObjectName("page")
        self.ctx = ctx
        root = QVBoxLayout(self); root.setContentsMargins(18, 14, 18, 14); root.setSpacing(14)
        self.tabs = QTabWidget()
        root.addWidget(self.tabs)
        tab_layouts = []
        for title in ("Machine", "Home && axes", "Camera \u2192 robot"):
            holder = QWidget(); holder.setObjectName("page")
            lay = QVBoxLayout(holder); lay.setContentsMargins(0, 12, 0, 0); lay.setSpacing(14)
            self.tabs.addTab(holder, title)
            tab_layouts.append(lay)
        t_machine, t_home, t_cam = tab_layouts
        self.home_wizard = HomeWizard(ctx, self)         # (it only touches this page's fields when a button is pressed)
        t_home.addWidget(self.home_wizard)

        # ---------------------------------------------------------------- geometry
        g = card("Robot geometry   (measure these; values marked ? are unverified)")
        f = QFormLayout(g)
        self.base_r = spin(1, 2000, 76.2); self.eff_r = spin(1, 2000, 50.0)
        self.bicep = spin(1, 2000, 150.0); self.fore = spin(1, 4000, 280.0)
        f.addRow("Base radius  (centre → motor axis)", self.base_r)
        f.addRow("Effector radius  (centre → forearm joints) ?", self.eff_r)
        f.addRow("Bicep length  (shoulder → elbow) ?", self.bicep)
        f.addRow("Forearm length  (elbow → effector joint) ?", self.fore)
        self.sides = QLabel(""); self.sides.setObjectName("muted"); self.sides.setWordWrap(True)
        f.addRow(self.sides)
        t_machine.addWidget(g)

        # ---------------------------------------------------------------- motors
        m = card("Motors and gearbox")
        mf = QFormLayout(m)
        self.full_steps = ibox(1, 10000, 200)
        self.micro = ibox(1, 1024, 16)
        self.gear = spin(0.01, 10000, 47.0, 2, "  : 1", 1.0)
        mf.addRow("Motor full steps per revolution", self.full_steps)
        mf.addRow("Driver microsteps  (DM542T switches)", self.micro)
        mf.addRow("Gearbox ratio", self.gear)
        self.spr = QLabel(""); self.spr.setObjectName("mono")
        mf.addRow("Steps per arm revolution", self.spr)
        t_machine.addWidget(m)

        # ---------------------------------------------------------------- axes
        a = card("Axes")
        ag = QGridLayout(a)
        self.offs = [spin(-360, 360, 0.0, 2, " deg", 0.5) for _ in range(3)]
        self.inv = [QCheckBox(f"Invert {i + 1}") for i in range(3)]
        for i in range(3):
            ag.addWidget(QLabel(f"Offset {i + 1}"), 0, i * 2); ag.addWidget(self.offs[i], 0, i * 2 + 1)
            ag.addWidget(self.inv[i], 1, i * 2, 1, 2)
        self.tmin = spin(-180, 180, -30.0, 1, " deg"); self.tmax = spin(-180, 180, 80.0, 1, " deg")
        ag.addWidget(QLabel("Arm angle limits ?"), 2, 0); ag.addWidget(self.tmin, 2, 1); ag.addWidget(self.tmax, 2, 3)
        ag.addWidget(QLabel("to"), 2, 2)
        t_home.addWidget(a)

        # ---------------------------------------------------------------- home
        h = card("Home position")
        hg = QGridLayout(h)
        self.home_xyz = [spin(-2000, 2000, v) for v in (0.0, 0.0, -200.0)]
        for i, (n, s) in enumerate(zip("XYZ", self.home_xyz)):
            hg.addWidget(QLabel(n), 0, i * 2); hg.addWidget(s, 0, i * 2 + 1)
        self.home_cal = QCheckBox("Home is calibrated (allows 'Recover position from steps' after a stop)")
        hg.addWidget(self.home_cal, 1, 0, 1, 6)
        self.home_check = CheckLabel(); hg.addWidget(self.home_check, 2, 0, 1, 6)
        self.fit_btn = QPushButton("Set offsets so that step 0 = this home position")
        self.fit_btn.setToolTip("Assumes the real arms are exactly at this XYZ when the limit switches trigger. Measure first!")
        self.fit_btn.clicked.connect(self._fit_offsets)
        hg.addWidget(self.fit_btn, 3, 0, 1, 6)
        t_home.addWidget(h)

        # ---------------------------------------------------------------- workspace
        w = card("Workspace box  (moves outside it are refused)")
        wg = QGridLayout(w)
        self.ws = {}
        for r, ax in enumerate("xyz"):
            lo, hi = spin(-2000, 2000, 0), spin(-2000, 2000, 0)
            self.ws[ax] = (lo, hi)
            wg.addWidget(QLabel(ax.upper()), r, 0); wg.addWidget(lo, r, 1); wg.addWidget(QLabel("to"), r, 2); wg.addWidget(hi, r, 3)
        self.reach = CheckLabel(); wg.addWidget(self.reach, 3, 0, 1, 4)
        t_machine.addWidget(w)

        # ---------------------------------------------------------------- camera -> robot
        self.camera_wizard = CameraWizard(ctx, camera)
        t_cam.addWidget(self.camera_wizard)
        t_cam.addStretch(1)
        t_machine.addStretch(1); t_home.addStretch(1)

        # ---------------------------------------------------------------- buttons
        row = QHBoxLayout()
        self.apply_btn = QPushButton("Apply and save"); self.apply_btn.setObjectName("primary"); self.apply_btn.clicked.connect(self._apply)
        self.revert_btn = QPushButton("Revert changes"); self.revert_btn.clicked.connect(self.load_from_settings)
        self.msg = QLabel(""); self.msg.setObjectName("muted"); self.msg.setWordWrap(True)
        row.addWidget(self.apply_btn); row.addWidget(self.revert_btn); row.addWidget(self.msg, 1)
        root.addLayout(row)

        for wdg in (self.base_r, self.eff_r, self.bicep, self.fore, self.full_steps, self.micro, self.gear, self.tmin, self.tmax,
                    *self.offs, *self.home_xyz, *[x for pair in self.ws.values() for x in pair]):
            wdg.valueChanged.connect(self._derive)
        ctx.settings_applied.connect(self.load_from_settings)
        self.load_from_settings()

    # ------------------------------------------------------------------ settings <-> widgets
    def load_from_settings(self, *_):
        s = self.ctx.settings.data
        blocked = [self.base_r, self.eff_r, self.bicep, self.fore, self.full_steps, self.micro, self.gear, self.tmin, self.tmax,
                   *self.offs, *self.home_xyz, *[x for pair in self.ws.values() for x in pair]]
        for b in blocked:
            b.blockSignals(True)
        g, d, ax = s["geometry"], s["drive"], s["axes"]
        self.base_r.setValue(g["base_radius_mm"]); self.eff_r.setValue(g["effector_radius_mm"])
        self.bicep.setValue(g["bicep_mm"]); self.fore.setValue(g["forearm_mm"])
        self.full_steps.setValue(int(d["motor_full_steps"])); self.micro.setValue(int(d["microsteps"])); self.gear.setValue(d["gearbox_ratio"])
        for i in range(3):
            self.offs[i].setValue(ax["offsets_deg"][i]); self.inv[i].setChecked(bool(ax["invert"][i]))
        self.tmin.setValue(ax["theta_min_deg"]); self.tmax.setValue(ax["theta_max_deg"])
        for i in range(3):
            self.home_xyz[i].setValue(s["home"]["xyz_mm"][i])
        self.home_cal.setChecked(bool(s["home"]["calibrated"]))
        for k in "xyz":
            self.ws[k][0].setValue(s["workspace_mm"][k][0]); self.ws[k][1].setValue(s["workspace_mm"][k][1])
        for b in blocked:
            b.blockSignals(False)
        self._derive()

    def _box(self) -> Workspace:
        return Workspace(tuple(self.ws["x"][i].value() for i in (0, 1)), tuple(self.ws["y"][i].value() for i in (0, 1)),
                         tuple(self.ws["z"][i].value() for i in (0, 1)))

    def _kin(self) -> DeltaKinematics:
        return DeltaKinematics(self.base_r.value(), self.eff_r.value(), self.bicep.value(), self.fore.value())

    def _derive(self, *_):
        k = self._kin()
        self.sides.setText(f"The kinematics formulas use triangle SIDE lengths: base {radius_to_side(self.base_r.value()):.1f} mm, "
                           f"effector {radius_to_side(self.eff_r.value()):.1f} mm (side = 2√3 × radius). "
                           f"The old program asked for these directly, which is easy to mix up with radii.")
        spr = self.full_steps.value() * self.micro.value() * self.gear.value()
        self.spr.setText(f"{spr:,.0f}   ({spr / 360:.2f} steps per degree)")
        # home consistency: where do the arms have to be at step 0, versus the angles the home XYZ needs?
        a = k.inverse(*(s.value() for s in self.home_xyz))
        if a is None:
            self.home_check.set("The home XYZ is unreachable with this geometry.", False)
        else:
            worst = max(abs(a[i] + self.offs[i].value()) for i in range(3))
            if worst * spr / 360 > 50:
                self.home_check.set(f"HOME and the offsets disagree by up to {worst:.1f} deg: a move to the home XYZ would not be a "
                                    f"no-op. Calibrate home, or use the button below if you trust the home XYZ.", False)
            else:
                self.home_check.set("Offsets are consistent with the home XYZ.", True)
        # is the whole workspace box reachable?
        box = self._box()
        pts = [[lo + (hi - lo) * i / 4 for i in range(5)] for lo, hi in (box.x, box.y, box.z)]
        bad = 0
        for x, y, z in itertools.product(*pts):
            ang = k.inverse(x, y, z)
            if ang is None or joint_limit_violation(ang, self.tmin.value(), self.tmax.value()):
                bad += 1
        self.reach.set("The whole box is reachable (125 sampled points)." if bad == 0 else
                       f"{bad} of 125 sampled points in this box are unreachable or exceed the arm-angle limits: make the box smaller.",
                       bad == 0)

    def _fit_offsets(self):
        a = self._kin().inverse(*(s.value() for s in self.home_xyz))
        if a is None:
            return
        for i in range(3):
            self.offs[i].setValue(-a[i])
        self.msg.setText("Offsets set from the home XYZ (not saved yet). Only valid if the arms really are at that XYZ at the limit switches.")

    # ------------------------------------------------------------------ apply
    def _apply(self):
        if self.ctx.robot.busy:
            QMessageBox.information(self, "Busy", "The robot is moving. Wait until it stops, then apply.")
            return
        new = copy.deepcopy(self.ctx.settings.data)
        new["geometry"].update(base_radius_mm=self.base_r.value(), effector_radius_mm=self.eff_r.value(), bicep_mm=self.bicep.value(),
                               forearm_mm=self.fore.value())
        new["drive"].update(motor_full_steps=self.full_steps.value(), microsteps=self.micro.value(), gearbox_ratio=self.gear.value())
        new["axes"].update(offsets_deg=[s.value() for s in self.offs], invert=[c.isChecked() for c in self.inv],
                           theta_min_deg=self.tmin.value(), theta_max_deg=self.tmax.value())
        new["home"].update(xyz_mm=[s.value() for s in self.home_xyz], calibrated=self.home_cal.isChecked())
        new["workspace_mm"] = {k: [self.ws[k][0].value(), self.ws[k][1].value()] for k in "xyz"}
        problem = validate(new)
        if problem:
            QMessageBox.warning(self, "Cannot apply", problem)
            return
        changed_geometry = new["geometry"] != self.ctx.settings.data["geometry"] or new["drive"] != self.ctx.settings.data["drive"] \
            or new["axes"] != self.ctx.settings.data["axes"]
        self.ctx.settings.data = new
        self.ctx.settings.save()
        self.ctx.apply_settings()
        extra = ""
        if changed_geometry and self.ctx.robot.xyz_valid:
            self.ctx.robot._invalidate("the geometry changed")
            extra = "  The geometry changed, so the position is unknown: home the robot again."
        self.msg.setText("Applied and saved." + extra)

"""
kinematics.py -- delta-robot geometry: inverse kinematics (XYZ -> arm angles), forward kinematics (arm angles -> XYZ),
steps conversion, and the workspace / joint-limit checks. Pure Python (no GUI, no hardware), so it is easy to test.

Coordinates: millimetres, origin at the centre of the base plate, Z axis pointing UP, so the effector working below the base
has NEGATIVE Z. Arm angle 0 deg = upper arm (bicep) horizontal; positive = arm swung DOWN (the usual delta convention).

IMPORTANT -- radii versus side lengths
  The classic delta formulas (and the original GUI) work with the SIDE LENGTHS of the equilateral triangles that the shoulder
  axes and the effector joints form. People measure RADII (centre -> motor axis, centre -> effector joint). The two are related by
        side = 2 * sqrt(3) * radius          (radius = side * 0.2887)
  This class takes the measured RADII and converts, so the numbers in the GUI mean what they say.
"""
import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

SQRT3 = math.sqrt(3.0)
SIN120 = SQRT3 / 2.0
COS120 = -0.5
TAN30 = 1.0 / SQRT3
TAN60 = SQRT3
SIN30 = 0.5

Triple = Tuple[float, float, float]


def radius_to_side(radius_mm: float) -> float:
    return 2.0 * SQRT3 * radius_mm


def side_to_radius(side_mm: float) -> float:
    return side_mm / (2.0 * SQRT3)


class DeltaKinematics:
    def __init__(self, base_radius: float = 76.2, effector_radius: float = 50.0, bicep: float = 150.0,
                 forearm: float = 280.0):
        self.base_radius, self.effector_radius = base_radius, effector_radius
        self.rf, self.re = bicep, forearm                       # rf = upper arm (shoulder->elbow), re = forearm
        self.f = radius_to_side(base_radius)                    # triangle SIDE lengths used by the classic formulas
        self.e = radius_to_side(effector_radius)

    # ------------------------------------------------------------------ inverse
    def _angle_yz(self, x0: float, y0: float, z0: float) -> Optional[float]:
        """Angle of the arm that lies in the YZ plane; the other two arms reuse it after rotating the target by 120 deg."""
        f, e, rf, re = self.f, self.e, self.rf, self.re
        if abs(z0) < 1e-9:
            return None                                          # the formula divides by z; a target in the base plane is invalid
        y1 = -0.5 * TAN30 * f
        y0 = y0 - 0.5 * TAN30 * e
        a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
        b = (y1 - y0) / z0
        d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)    # discriminant: < 0 means the point cannot be reached
        if d < 0:
            return None
        yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
        zj = a + b * yj
        return math.degrees(math.atan2(-zj, y1 - yj))

    def inverse(self, x: float, y: float, z: float) -> Optional[Triple]:
        """XYZ (mm) -> (theta1, theta2, theta3) in degrees, or None if unreachable."""
        t1 = self._angle_yz(x, y, z)
        if t1 is None:
            return None
        t2 = self._angle_yz(x * COS120 + y * SIN120, y * COS120 - x * SIN120, z)
        if t2 is None:
            return None
        t3 = self._angle_yz(x * COS120 - y * SIN120, y * COS120 + x * SIN120, z)
        if t3 is None:
            return None
        return t1, t2, t3

    # ------------------------------------------------------------------ forward
    def forward(self, t1: float, t2: float, t3: float) -> Optional[Triple]:
        """(theta1, theta2, theta3) in degrees -> XYZ (mm), or None if the arms cannot meet at one point.
        Intersection of three spheres (one per forearm), solved in closed form."""
        f, e, rf, re = self.f, self.e, self.rf, self.re
        t = (f - e) * TAN30 / 2.0
        d2r = math.pi / 180.0
        t1, t2, t3 = t1 * d2r, t2 * d2r, t3 * d2r
        y1 = -(t + rf * math.cos(t1))
        z1 = -rf * math.sin(t1)
        y2 = (t + rf * math.cos(t2)) * SIN30
        x2 = y2 * TAN60
        z2 = -rf * math.sin(t2)
        y3 = (t + rf * math.cos(t3)) * SIN30
        x3 = -y3 * TAN60
        z3 = -rf * math.sin(t3)
        dnm = (y2 - y1) * x3 - (y3 - y1) * x2
        if abs(dnm) < 1e-12:
            return None
        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) ** 2 + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)
        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None
        z0 = -0.5 * (b + math.sqrt(disc)) / a
        return (a1 * z0 + b1) / dnm, (a2 * z0 + b2) / dnm, z0


@dataclass
class Workspace:
    """Axis-aligned box of allowed XYZ targets (mm). A move outside it is refused BEFORE kinematics even runs."""
    # same as the settings defaults (a box that is fully reachable with the default arm lengths)
    x: Tuple[float, float] = (-100.0, 100.0)
    y: Tuple[float, float] = (-100.0, 100.0)
    z: Tuple[float, float] = (-350.0, -210.0)

    def violation(self, x: float, y: float, z: float) -> Optional[str]:
        for name, v, (lo, hi) in (("X", x, self.x), ("Y", y, self.y), ("Z", z, self.z)):
            if not (lo <= v <= hi):
                return f"{name}={v:.1f} is outside the workspace box [{lo:.1f}, {hi:.1f}]"
        return None

    def contains(self, x: float, y: float, z: float) -> bool:
        return self.violation(x, y, z) is None


def angles_to_steps(angles: Sequence[float], offsets_deg: Sequence[float], invert: Sequence[bool],
                    steps_per_deg: float) -> Tuple[int, int, int]:
    """Arm angle (deg) -> absolute motor step target. step 0 is the homed pose, so the pose angle at home is -offset."""
    out = []
    for theta, off, inv in zip(angles, offsets_deg, invert):
        corrected = theta + off
        if inv:
            corrected = -corrected
        out.append(int(round(corrected * steps_per_deg)))
    return out[0], out[1], out[2]


def steps_to_angles(steps: Sequence[int], offsets_deg: Sequence[float], invert: Sequence[bool],
                    steps_per_deg: float) -> Triple:
    """Inverse of angles_to_steps."""
    out = []
    for s, off, inv in zip(steps, offsets_deg, invert):
        corrected = s / steps_per_deg
        if inv:
            corrected = -corrected
        out.append(corrected - off)
    return out[0], out[1], out[2]


def joint_limit_violation(angles: Sequence[float], theta_min: float, theta_max: float) -> Optional[str]:
    for i, a in enumerate(angles):
        if not (theta_min <= a <= theta_max):
            return f"arm {i + 1} would need {a:.1f} deg, outside its allowed range [{theta_min:.0f}, {theta_max:.0f}] deg"
    return None

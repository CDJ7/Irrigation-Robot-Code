"""
icons.py -- the small navigation icons, DRAWN with QPainter (no image files to ship, sharp at any screen scaling).
"""
import math

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap


def _canvas(size: int):
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    return pm, p


def make_icon(name: str, color: str = "#c9d1dc", size: int = 40) -> QIcon:
    pm, p = _canvas(size)
    c = QColor(color)
    pen = QPen(c, size * 0.075, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin)
    p.setPen(pen)
    p.setBrush(Qt.BrushStyle.NoBrush)
    m, s = size * 0.18, size
    cx = cy = size / 2
    if name == "operations":                       # four arrows: move in X/Y
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            p.drawLine(QPointF(cx, cy), QPointF(cx + dx * (cx - m), cy + dy * (cy - m)))
            tip = QPointF(cx + dx * (cx - m), cy + dy * (cy - m))
            p.drawLine(tip, QPointF(tip.x() - dx * size * .12 + dy * size * .1, tip.y() - dy * size * .12 + dx * size * .1))
            p.drawLine(tip, QPointF(tip.x() - dx * size * .12 - dy * size * .1, tip.y() - dy * size * .12 - dx * size * .1))
    elif name == "camera":
        p.drawRoundedRect(QRectF(m * .8, s * .30, s - 2 * m * .8, s * .46), 5, 5)
        p.drawEllipse(QPointF(cx, s * .53), s * .13, s * .13)
        p.drawLine(QPointF(s * .36, s * .30), QPointF(s * .42, s * .20)); p.drawLine(QPointF(s * .42, s * .20), QPointF(s * .58, s * .20))
        p.drawLine(QPointF(s * .58, s * .20), QPointF(s * .64, s * .30))
    elif name == "calibration":                    # crosshair
        p.drawEllipse(QPointF(cx, cy), s * .24, s * .24)
        p.drawLine(QPointF(cx, m * .6), QPointF(cx, s - m * .6)); p.drawLine(QPointF(m * .6, cy), QPointF(s - m * .6, cy))
    elif name == "harvest":                        # a berry with a stem
        p.drawEllipse(QPointF(cx, s * .58), s * .24, s * .26)
        p.drawLine(QPointF(cx, s * .32), QPointF(cx, s * .16)); p.drawLine(QPointF(cx, s * .24), QPointF(s * .66, s * .18))
        p.drawLine(QPointF(cx, s * .24), QPointF(s * .34, s * .18))
    elif name == "debug":                          # terminal prompt  >_
        p.drawLine(QPointF(s * .22, s * .30), QPointF(s * .44, s * .50)); p.drawLine(QPointF(s * .44, s * .50), QPointF(s * .22, s * .70))
        p.drawLine(QPointF(s * .52, s * .72), QPointF(s * .78, s * .72))
    elif name == "settings":                       # gear
        path = QPainterPath()
        teeth, r_out, r_in = 8, s * .40, s * .30
        for i in range(teeth * 2):
            a = math.pi * i / teeth
            r = r_out if i % 2 == 0 else r_in
            pt = QPointF(cx + r * math.cos(a), cy + r * math.sin(a))
            path.moveTo(pt) if i == 0 else path.lineTo(pt)
        path.closeSubpath()
        p.drawPath(path)
        p.drawEllipse(QPointF(cx, cy), s * .12, s * .12)
    p.end()
    return QIcon(pm)

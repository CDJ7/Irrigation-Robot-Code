"""
widgets.py -- small reusable pieces: status pills, a picture view that keeps its aspect ratio, a card helper.
"""
from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (QDoubleSpinBox, QFormLayout, QFrame, QGroupBox, QLabel, QSizePolicy, QSpinBox,
                             QVBoxLayout, QWidget)


def repolish(w: QWidget) -> None:
    """Re-apply the stylesheet after a dynamic property changed (Qt does not do this by itself)."""
    w.style().unpolish(w)
    w.style().polish(w)


class Pill(QLabel):
    """A rounded status label. level: ok (green) | warn (amber) | bad (red) | idle (grey)."""
    def __init__(self, text: str = "", level: str = "idle"):
        super().__init__(text)
        self.setObjectName("pill")
        self.set(text, level)

    def set(self, text: str, level: str) -> None:
        self.setText(text)
        self.setProperty("level", level)
        repolish(self)


class Badge(QLabel):
    def __init__(self, text: str = "", level: str = "ok"):
        super().__init__(text)
        self.setObjectName("badge")
        self.set(text, level)

    def set(self, text: str, level: str) -> None:
        self.setText(text)
        self.setProperty("level", level)
        repolish(self)


class CheckLabel(QLabel):
    """One-line verdict, green or red (used for 'is this target allowed?')."""
    def __init__(self):
        super().__init__("")
        self.setObjectName("check")
        self.setWordWrap(True)

    def set(self, text: str, ok: bool) -> None:
        self.setText(("✓  " if ok else "✗  ") + text)
        self.setProperty("level", "ok" if ok else "bad")
        repolish(self)


class ImageView(QLabel):
    """Shows a picture scaled to fit, keeping its shape; shows a message while there is no picture."""
    def __init__(self, placeholder: str = "No picture yet", min_w: int = 320, min_h: int = 240):
        super().__init__(placeholder)
        self.setObjectName("video")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(min_w, min_h)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._pix: Optional[QPixmap] = None
        self._placeholder = placeholder

    def set_image(self, img: Optional[QImage]) -> None:
        if img is None:
            self._pix = None
            self.setPixmap(QPixmap())
            self.setText(self._placeholder)
            return
        self._pix = QPixmap.fromImage(img)
        self._rescale()

    def _rescale(self) -> None:
        if self._pix is not None and self.width() > 4 and self.height() > 4:
            self.setText("")
            self.setPixmap(self._pix.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                            Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, ev) -> None:        # noqa: N802 (Qt naming)
        super().resizeEvent(ev)
        self._rescale()


def card(title: str) -> QGroupBox:
    return QGroupBox(title)


class NumberBox(QDoubleSpinBox):
    """A number field that (1) has no tiny up/down buttons (type the value; the Up/Down keys still work) and (2) ignores the mouse
    wheel unless it has the keyboard focus -- otherwise scrolling a page of settings would silently change values under the cursor."""
    def __init__(self):
        super().__init__()
        # (the arrow buttons are hidden by the stylesheet in theme.py; setButtonSymbols(NoButtons) would make the text vanish)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMaximumWidth(190)

    def wheelEvent(self, ev) -> None:        # noqa: N802
        if self.hasFocus():
            super().wheelEvent(ev)
        else:
            ev.ignore()


class IntBox(QSpinBox):
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMaximumWidth(190)
        self.setAlignment(Qt.AlignmentFlag.AlignRight)

    def wheelEvent(self, ev) -> None:        # noqa: N802
        if self.hasFocus():
            super().wheelEvent(ev)
        else:
            ev.ignore()


def spin(lo: float, hi: float, value: float, decimals: int = 1, suffix: str = " mm", step: float = 1.0) -> NumberBox:
    s = NumberBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setSuffix(suffix)
    s.setValue(value)
    s.setKeyboardTracking(False)
    s.setAlignment(Qt.AlignmentFlag.AlignRight)
    return s


def ibox(lo: int, hi: int, value: int = 0, suffix: str = "", step: int = 1) -> IntBox:
    s = IntBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setSuffix(suffix)
    s.setValue(value)
    s.setKeyboardTracking(False)
    return s


def separator() -> QFrame:
    f = QFrame()
    f.setObjectName("sep")
    f.setFixedHeight(1)
    return f

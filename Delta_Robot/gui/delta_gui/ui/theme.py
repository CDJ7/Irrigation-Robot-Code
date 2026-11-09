"""
theme.py -- the ONE place that defines how the program looks (colours, fonts, spacing). Every widget picks its look from the
stylesheet below, so changing a colour here changes it everywhere.
"""
PALETTE = {
    "bg": "#12151a",          # window background
    "panel": "#1a1e25",       # cards / panels
    "panel2": "#21262f",      # inputs, raised areas
    "border": "#2c333e",
    "text": "#e6e9ee",
    "muted": "#8d97a6",
    "accent": "#3b82f6",      # primary action
    "accent_hover": "#5b9bff",
    "ok": "#22c55e",
    "warn": "#f59e0b",
    "danger": "#ef4444",
    "danger_dark": "#b91c1c",
}


def stylesheet() -> str:
    p = PALETTE
    return f"""
* {{ font-family: "Segoe UI", "Inter", "Noto Sans", "DejaVu Sans", sans-serif; font-size: 13px; color: {p['text']}; }}
QMainWindow, QDialog, QWidget#page {{ background: {p['bg']}; }}
QToolTip {{ background: {p['panel2']}; color: {p['text']}; border: 1px solid {p['border']}; padding: 4px; }}

/* ---- cards ---- */
QGroupBox {{ background: {p['panel']}; border: 1px solid {p['border']}; border-radius: 10px; margin-top: 16px; padding: 14px 12px 10px 12px; }}
QGroupBox::title {{ subcontrol-origin: margin; subcontrol-position: top left; left: 12px; padding: 0 6px; color: {p['muted']};
                   font-weight: 600; font-size: 12px; text-transform: uppercase; }}
QLabel#muted {{ color: {p['muted']}; }}
QLabel#h1 {{ font-size: 20px; font-weight: 700; }}
QLabel#readout {{ font-family: "Cascadia Mono", "Consolas", "DejaVu Sans Mono", monospace; font-size: 26px; font-weight: 600; }}
QLabel#mono {{ font-family: "Cascadia Mono", "Consolas", "DejaVu Sans Mono", monospace; }}

/* ---- buttons ---- */
QPushButton {{ background: {p['panel2']}; border: 1px solid {p['border']}; border-radius: 7px; padding: 7px 14px; }}
QPushButton:hover {{ border-color: {p['accent']}; }}
QPushButton:pressed {{ background: {p['border']}; }}
QPushButton:disabled {{ color: #5a6472; background: {p['panel']}; border-color: {p['panel2']}; }}
QPushButton#primary {{ background: {p['accent']}; border-color: {p['accent']}; color: white; font-weight: 600; }}
QPushButton#primary:hover {{ background: {p['accent_hover']}; }}
QPushButton#primary:disabled {{ background: #26406b; color: #7f8da6; border-color: #26406b; }}
QPushButton#warning {{ background: #3b2e0e; border-color: {p['warn']}; color: {p['warn']}; }}

/* ---- the E-STOP: big, red, impossible to miss ---- */
QPushButton#estop {{ background: {p['danger']}; border: 3px solid #fecaca; border-radius: 12px; color: white;
                     font-size: 17px; font-weight: 800; padding: 14px 8px; }}
QPushButton#estop:hover {{ background: #f87171; }}
QPushButton#estop:pressed {{ background: {p['danger_dark']}; }}
QPushButton#estop[latched="true"] {{ background: #7f1d1d; border-color: {p['danger']}; }}

/* ---- sidebar ---- */
QWidget#sidebar {{ background: {p['panel']}; border-right: 1px solid {p['border']}; }}
QPushButton#nav {{ text-align: left; background: transparent; border: none; border-radius: 8px; padding: 11px 12px; font-size: 14px; }}
QPushButton#nav:hover {{ background: {p['panel2']}; }}
QPushButton#nav:checked {{ background: {p['panel2']}; color: {p['accent_hover']}; font-weight: 700;
                           border-left: 3px solid {p['accent']}; }}

/* ---- header + status bar ---- */
QWidget#header {{ background: {p['panel']}; border-bottom: 1px solid {p['border']}; }}
QStatusBar {{ background: {p['panel']}; border-top: 1px solid {p['border']}; }}
QStatusBar::item {{ border: none; }}
QLabel#pill {{ border-radius: 10px; padding: 3px 11px; background: {p['panel2']}; border: 1px solid {p['border']}; font-weight: 600; }}
QLabel#pill[level="ok"] {{ background: #0f2e1c; border-color: {p['ok']}; color: #86efac; }}
QLabel#pill[level="warn"] {{ background: #3b2e0e; border-color: {p['warn']}; color: #fcd34d; }}
QLabel#pill[level="bad"] {{ background: #3b1212; border-color: {p['danger']}; color: #fca5a5; }}
QLabel#pill[level="idle"] {{ color: {p['muted']}; }}
QLabel#badge[level="ok"] {{ color: {p['ok']}; font-weight: 700; }}
QLabel#badge[level="bad"] {{ color: {p['danger']}; font-weight: 700; }}
QLabel#badge[level="warn"] {{ color: {p['warn']}; font-weight: 700; }}
QLabel#check[level="ok"] {{ color: {p['ok']}; }}
QLabel#check[level="bad"] {{ color: {p['danger']}; }}

/* ---- inputs ---- */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background: {p['panel2']}; border: 1px solid {p['border']}; border-radius: 6px;
                                                   padding: 5px 8px; selection-background-color: {p['accent']}; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ min-height: 24px; }}
QPushButton {{ min-height: 20px; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {p['accent']}; }}
QComboBox QAbstractItemView {{ background: {p['panel2']}; selection-background-color: {p['accent']}; border: 1px solid {p['border']}; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{ width: 0px; height: 0px; border: none; margin: 0; padding: 0; }}
QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border-radius: 4px; border: 1px solid {p['border']}; background: {p['panel2']}; }}
QCheckBox::indicator:checked {{ background: {p['accent']}; border-color: {p['accent']}; }}

/* ---- tables, logs ---- */
QTableWidget {{ background: {p['panel']}; gridline-color: {p['border']}; border: 1px solid {p['border']}; border-radius: 8px;
                alternate-background-color: #1d222a; selection-background-color: #1e3a66; }}
QHeaderView::section {{ background: {p['panel2']}; border: none; border-bottom: 1px solid {p['border']}; padding: 6px; color: {p['muted']};
                        font-weight: 600; }}
QPlainTextEdit {{ background: #0e1014; border: 1px solid {p['border']}; border-radius: 8px; font-family: "Cascadia Mono", "Consolas",
                  "DejaVu Sans Mono", monospace; font-size: 12px; }}
QDockWidget {{ titlebar-close-icon: none; }}
QDockWidget::title {{ background: {p['panel']}; padding: 6px 10px; border-bottom: 1px solid {p['border']}; }}
QScrollBar:vertical {{ background: transparent; width: 10px; }}
QScrollBar::handle:vertical {{ background: {p['border']}; border-radius: 5px; min-height: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QLabel#video {{ background: #0b0d11; border: 1px solid {p['border']}; border-radius: 8px; color: {p['muted']}; }}
QFrame#sep {{ background: {p['border']}; max-height: 1px; }}
QScrollArea#pagescroll, QScrollArea#pagescroll QWidget#qt_scrollarea_viewport {{ background: {p['bg']}; border: none; }}
"""

"""Dark theme for the dashboard: one QSS stylesheet plus a small palette of
named colors so widgets stay visually consistent without hardcoding hex
values all over the place.
"""
from __future__ import annotations

# Semantic colors, reused by widgets for status dots/text (e.g. joint
# table stale cells, connection indicator, torque state banner).
COLOR_OK = '#3fb950'          # green: enabled / connected / good
COLOR_WARN = '#d29922'        # amber: transitioning / free move / caution
COLOR_ERROR = '#f85149'       # red: error / disconnected / stale
COLOR_INFO = '#58a6ff'        # blue: neutral informational
COLOR_MUTED = '#8b949e'       # gray: unknown / idle / disabled text

BACKGROUND = '#0d1117'
PANEL_BACKGROUND = '#161b22'
BORDER = '#30363d'
TEXT = '#e6edf3'
TEXT_MUTED = COLOR_MUTED

DARK_STYLESHEET = f"""
QWidget {{
    background-color: {BACKGROUND};
    color: {TEXT};
    font-family: "Segoe UI", "Ubuntu", sans-serif;
    font-size: 13px;
}}

QMainWindow, QDialog {{
    background-color: {BACKGROUND};
}}

QGroupBox {{
    background-color: {PANEL_BACKGROUND};
    border: 1px solid {BORDER};
    border-radius: 6px;
    margin-top: 12px;
    padding: 8px;
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: {TEXT_MUTED};
}}

QPushButton {{
    background-color: #21262d;
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 8px 14px;
    color: {TEXT};
}}
QPushButton:hover:!disabled {{
    background-color: #30363d;
}}
QPushButton:pressed:!disabled {{
    background-color: #388bfd26;
}}
QPushButton:disabled {{
    color: {TEXT_MUTED};
    border-color: #21262d;
}}

QPushButton#freeMoveButton {{
    background-color: {COLOR_WARN};
    color: #1a1a1a;
    font-weight: 700;
}}
QPushButton#freeMoveButton:hover:!disabled {{
    background-color: #e3a72e;
}}

QPushButton#holdButton {{
    background-color: {COLOR_OK};
    color: #05260f;
    font-weight: 700;
}}
QPushButton#holdButton:hover:!disabled {{
    background-color: #56d364;
}}

QPushButton#stopButton {{
    background-color: {COLOR_ERROR};
    color: #2b0b09;
    font-weight: 800;
    font-size: 15px;
    padding: 14px;
}}
QPushButton#stopButton:hover:!disabled {{
    background-color: #ff6a61;
}}

QTableWidget {{
    background-color: {PANEL_BACKGROUND};
    gridline-color: {BORDER};
    border: 1px solid {BORDER};
    selection-background-color: #388bfd40;
}}
QHeaderView::section {{
    background-color: #21262d;
    color: {TEXT_MUTED};
    padding: 4px;
    border: none;
    border-bottom: 1px solid {BORDER};
}}

QTabWidget::pane {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    top: -1px;
}}
QTabBar::tab {{
    background: {PANEL_BACKGROUND};
    padding: 8px 16px;
    border: 1px solid {BORDER};
    border-bottom: none;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background: {BACKGROUND};
    color: {COLOR_INFO};
}}

QLabel#bannerLabel {{
    font-size: 16px;
    font-weight: 700;
    padding: 6px;
    border-radius: 5px;
}}

QLabel#mockBanner {{
    background-color: {COLOR_WARN};
    color: #1a1a1a;
    font-weight: 800;
    padding: 4px 10px;
    border-radius: 4px;
}}
"""

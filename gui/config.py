"""Configuration parameters, color themes, and stylesheets for the GUI."""

from __future__ import annotations

import re

# Telemetry format regex
TELEMETRY_PATTERN = re.compile(
    r"L:(-?\d+)\s+d:(-?\d+)\s+RPM:(-?\d+)\s+PWM:(\d+)\s+\|\s+"
    r"R:(-?\d+)\s+d:(-?\d+)\s+RPM:(-?\d+)\s+PWM:(\d+)"
)

# Color theme
BG_DARK = "#0c1316"
BG_CANVAS = "#0e171a"
BG_PANEL = "#152024"
BG_CARD = "#1c2b30"
BG_INPUT = "#10181b"
BORDER = "#2b3d42"
TEXT_PRIMARY = "#e6edf0"
TEXT_MUTED = "#8fa3a8"
ACCENT_MINT = "#79e0b5"
ACCENT_ORANGE = "#ffbd69"
STOP_RED = "#e0534c"
STOP_BG = "#422120"
GRID_COLOR = "#233338"
AXIS_COLOR = "#3d575c"

# Maze visualization colors
MAZE_WALL_COLOR = "#ef4444"      # Competition red walls
MAZE_POST_COLOR = "#f8fafc"      # White corner posts
MAZE_FLOOR_BG = "#10181e"        # Maze floor area
MAZE_START_BG = "#133528"        # Subtle emerald green highlight for cell (0,0)
MAZE_GOAL_BG = "#2a1e38"         # Subtle indigo/violet highlight for center goal cells
MAZE_LABEL_COLOR = "#64748b"     # Coordinates / cell markers

# Default hardware & motion geometry
DEFAULT_LEFT_CPR = 815.70
DEFAULT_RIGHT_CPR = 815.55
DEFAULT_WHEEL_DIAMETER_MM = 33.0
DEFAULT_TRACK_WIDTH_MM = 112.0
DEFAULT_CELL_TICKS = 1060
DEFAULT_TURN_90_TICKS = 692
DEFAULT_TURN_180_TICKS = 1384
DEFAULT_CELL_SIZE_MM = 134.7
DEFAULT_BAUD = 115200


def get_app_stylesheet() -> str:
    """Returns the dark modern QSS theme for the application."""
    return f"""
        QMainWindow {{
            background-color: {BG_DARK};
        }}
        QWidget {{
            color: {TEXT_PRIMARY};
            font-family: 'Segoe UI', system-ui, sans-serif;
            font-size: 13px;
        }}
        QFrame#header {{
            background-color: {BG_DARK};
            border-bottom: 1px solid {BORDER};
        }}
        QScrollArea#sidebarScroll {{
            background-color: {BG_PANEL};
            border: none;
            border-right: 1px solid {BORDER};
        }}
        QWidget#sidebar {{
            background-color: {BG_PANEL};
        }}
        QScrollBar:vertical {{
            border: none;
            background: {BG_DARK};
            width: 6px;
            margin: 0px;
        }}
        QScrollBar::handle:vertical {{
            background: {BORDER};
            min-height: 25px;
            border-radius: 3px;
        }}
        QScrollBar::handle:vertical:hover {{
            background: {ACCENT_MINT};
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
            height: 0px;
        }}
        QFrame#footer {{
            background-color: {BG_PANEL};
            border-top: 1px solid {BORDER};
        }}
        QFrame#metricCard {{
            background-color: {BG_CARD};
            border: 1px solid {BORDER};
            border-radius: 6px;
        }}
        QFrame#stateBox {{
            background-color: {BG_CARD};
            border: 1px solid {BORDER};
            border-radius: 6px;
        }}
        QLabel#sectionHeader {{
            color: {ACCENT_MINT};
            font-weight: 600;
            font-size: 11px;
            letter-spacing: 0.5px;
        }}
        QLabel#metricTitle {{
            color: {TEXT_MUTED};
            font-size: 10px;
            font-weight: 600;
        }}
        QLabel#metricValue {{
            color: {TEXT_PRIMARY};
            font-size: 13px;
            font-weight: 600;
        }}
        QLineEdit, QComboBox {{
            background-color: {BG_INPUT};
            color: {TEXT_PRIMARY};
            border: 1px solid {BORDER};
            border-radius: 4px;
            padding: 3px 8px;
            min-height: 24px;
        }}
        QLineEdit:focus, QComboBox:focus {{
            border: 1px solid {ACCENT_MINT};
        }}
        QComboBox::drop-down {{
            border: none;
            width: 20px;
        }}
        QCheckBox {{
            spacing: 6px;
            color: {TEXT_PRIMARY};
            font-size: 12px;
        }}
        QCheckBox::indicator {{
            width: 15px;
            height: 15px;
            border: 1px solid {BORDER};
            border-radius: 3px;
            background-color: {BG_INPUT};
        }}
        QCheckBox::indicator:checked {{
            background-color: {ACCENT_MINT};
            border: 1px solid {ACCENT_MINT};
        }}
        QPushButton {{
            background-color: {BG_CARD};
            color: {TEXT_PRIMARY};
            border: 1px solid {BORDER};
            border-radius: 5px;
            font-weight: 600;
            padding: 5px 10px;
        }}
        QPushButton:hover {{
            background-color: #273a40;
            border-color: #3b5258;
        }}
        QPushButton:pressed {{
            background-color: #172428;
        }}
        QPushButton:disabled {{
            color: #55666a;
            background-color: #141c1e;
            border-color: #1f2a2d;
        }}
        QPushButton#actionBtn {{
            background-color: {ACCENT_MINT};
            color: #0b1a15;
            border: none;
        }}
        QPushButton#actionBtn:hover {{
            background-color: #9bf0ce;
        }}
        QPushButton#stopBtn {{
            background-color: {STOP_BG};
            color: #ffd8d4;
            border: 1px solid #6b2f2c;
        }}
        QPushButton#stopBtn:hover {{
            background-color: #5e2a27;
        }}
        QPushButton#dpadBtn {{
            font-size: 12px;
            padding: 6px 4px;
        }}
    """

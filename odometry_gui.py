"""Live differential-drive odometry viewer for the Blue Pill micromouse (PyQt6)."""

from __future__ import annotations

import math
import re
import sys
import threading
import time
from typing import Optional

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
import serial
from serial.tools import list_ports


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


class DifferentialDriveOdometry:
    """Differential-drive forward kinematics based on linear speed (v) and angular speed (w)."""

    def __init__(
        self,
        left_counts_per_revolution: float,
        right_counts_per_revolution: float,
        wheel_diameter_mm: float,
        track_width_mm: float,
    ) -> None:
        self.left_cpr = left_counts_per_revolution
        self.right_cpr = right_counts_per_revolution
        self.wheel_diameter_mm = wheel_diameter_mm
        self.wheel_circumference_mm = math.pi * wheel_diameter_mm
        self.track_width_mm = track_width_mm

        # Robot pose:
        # X: mm (East / Right is +X, West / Left is -X)
        # Y: mm (North / Forward is +Y, South / Backward is -Y)
        # Heading: rad CCW from +Y (0 = North, +pi/2 = West/Left, -pi/2 = East/Right)
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_rad = 0.0

        # Kinematic velocities
        self.linear_speed_mm_s = 0.0
        self.angular_speed_rad_s = 0.0

        self.path: list[tuple[float, float]] = [(0.0, 0.0)]
        self.last_counts: tuple[int, int] | None = None
        self.last_time: float | None = None

    def set_baseline(self, left_count: int, right_count: int) -> None:
        self.last_counts = (left_count, right_count)
        self.last_time = time.monotonic()

    def reset(self) -> None:
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_rad = 0.0
        self.linear_speed_mm_s = 0.0
        self.angular_speed_rad_s = 0.0
        self.path = [(0.0, 0.0)]
        self.last_counts = None
        self.last_time = None

    def update_counts(
        self, left_count: int, right_count: int, timestamp: float | None = None
    ) -> bool:
        if timestamp is None:
            timestamp = time.monotonic()

        current_counts = (left_count, right_count)
        if self.last_counts is None:
            self.last_counts = current_counts
            self.last_time = timestamp
            return False

        left_delta = left_count - self.last_counts[0]
        right_delta = right_count - self.last_counts[1]
        self.last_counts = current_counts

        dt = (timestamp - self.last_time) if self.last_time else 0.1
        self.last_time = timestamp
        dt = max(1e-4, dt)

        # 1. Wheel travel distances (mm)
        delta_left = (left_delta / self.left_cpr) * self.wheel_circumference_mm
        delta_right = (right_delta / self.right_cpr) * self.wheel_circumference_mm

        # 2. Differential Drive Kinematics:
        # Linear displacement: delta_s = (delta_right + delta_left) / 2
        # Angular displacement: delta_theta = (delta_right - delta_left) / L
        # In standard kinematics, counter-clockwise (turning LEFT) is positive:
        # when turning left, right wheel moves forward more than left wheel (delta_right > delta_left).
        delta_s = (delta_right + delta_left) * 0.5
        delta_theta = (delta_right - delta_left) / self.track_width_mm

        # Kinematic velocities
        self.linear_speed_mm_s = delta_s / dt
        self.angular_speed_rad_s = delta_theta / dt

        # 3. Kinematic Pose Integration:
        # Heading 0 is facing +Y (North).
        # Turning LEFT (delta_theta > 0) rotates toward -X (West).
        # Turning RIGHT (delta_theta < 0) rotates toward +X (East).
        if abs(delta_theta) < 1e-6:
            mid_heading = self.heading_rad + delta_theta * 0.5
            dx = -delta_s * math.sin(mid_heading)
            dy = delta_s * math.cos(mid_heading)
        else:
            # Exact arc integration with chord length
            half_dtheta = delta_theta * 0.5
            chord = delta_s * (math.sin(half_dtheta) / half_dtheta)
            mid_heading = self.heading_rad + half_dtheta
            dx = -chord * math.sin(mid_heading)
            dy = chord * math.cos(mid_heading)

        self.x_mm += dx
        self.y_mm += dy
        self.heading_rad = math.atan2(
            math.sin(self.heading_rad + delta_theta),
            math.cos(self.heading_rad + delta_theta),
        )

        self.path.append((self.x_mm, self.y_mm))
        return True

    def update_from_velocities(
        self, v_mm_s: float, omega_rad_s: float, dt: float
    ) -> bool:
        """Kinematic pose update directly using linear speed (v) and angular speed (omega)."""
        self.linear_speed_mm_s = v_mm_s
        self.angular_speed_rad_s = omega_rad_s

        delta_s = v_mm_s * dt
        delta_theta = omega_rad_s * dt

        if abs(delta_s) < 1e-9 and abs(delta_theta) < 1e-9:
            return False

        if abs(delta_theta) < 1e-6:
            mid_heading = self.heading_rad + delta_theta * 0.5
            dx = -delta_s * math.sin(mid_heading)
            dy = delta_s * math.cos(mid_heading)
        else:
            half_dtheta = delta_theta * 0.5
            chord = delta_s * (math.sin(half_dtheta) / half_dtheta)
            mid_heading = self.heading_rad + half_dtheta
            dx = -chord * math.sin(mid_heading)
            dy = chord * math.cos(mid_heading)

        self.x_mm += dx
        self.y_mm += dy
        self.heading_rad = math.atan2(
            math.sin(self.heading_rad + delta_theta),
            math.cos(self.heading_rad + delta_theta),
        )

        self.path.append((self.x_mm, self.y_mm))
        return True


class SerialReaderWorker(QThread):
    """Background worker for reading serial telemetry lines."""

    telemetry_received = pyqtSignal(str)
    connection_error = pyqtSignal(str)

    def __init__(self, connection: serial.Serial) -> None:
        super().__init__()
        self.connection = connection
        self._running = True

    def run(self) -> None:
        while self._running:
            try:
                line = self.connection.readline()
                if line:
                    decoded = line.decode("ascii", errors="replace").strip()
                    if decoded:
                        self.telemetry_received.emit(decoded)
            except (serial.SerialException, OSError) as exc:
                if self._running:
                    self.connection_error.emit(str(exc))
                break

    def stop(self) -> None:
        self._running = False


class OdometryCanvas(QWidget):
    """Smooth, hardware-accelerated 2D viewport for dead-reckoning trajectory."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)
        self.odometry: Optional[DifferentialDriveOdometry] = None
        self.cell_size_mm = 180.0

    def set_odometry(self, odometry: Optional[DifferentialDriveOdometry]) -> None:
        self.odometry = odometry
        self.update()

    def set_cell_size(self, cell_size_mm: float) -> None:
        self.cell_size_mm = max(10.0, cell_size_mm)
        self.update()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        self.setFocus()
        super().mousePressEvent(event)

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)

        width = self.width()
        height = self.height()

        painter.fillRect(0, 0, width, height, QColor(BG_CANVAS))

        if self.odometry is None or not self.odometry.path:
            self._draw_empty_state(painter, width, height)
            return

        points = self.odometry.path
        xs = [pt[0] for pt in points]
        ys = [pt[1] for pt in points]
        pad = self.cell_size_mm * 1.5
        min_x = min(xs) - pad
        max_x = max(xs) + pad
        min_y = min(ys) - pad
        max_y = max(ys) + pad

        world_w = max(max_x - min_x, self.cell_size_mm * 3.0)
        world_h = max(max_y - min_y, self.cell_size_mm * 3.0)

        scale = min((width - 48) / world_w, (height - 48) / world_h)
        scale = max(0.08, min(scale, 8.0))
        center_x = (min_x + max_x) * 0.5
        center_y = (min_y + max_y) * 0.5

        def screen(px: float, py: float) -> QPointF:
            return QPointF(
                width * 0.5 + (px - center_x) * scale,
                height * 0.5 - (py - center_y) * scale,
            )

        # Draw coordinate grid
        self._draw_grid(painter, width, height, scale, center_x, center_y, screen)

        # Draw path
        if len(points) > 1:
            path = QPainterPath()
            path.moveTo(screen(*points[0]))
            for pt in points[1:]:
                path.lineTo(screen(*pt))

            path_pen = QPen(QColor(ACCENT_MINT), 2.5)
            path_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            path_pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            painter.setPen(path_pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawPath(path)

        # Start position marker (origin)
        start_pt = screen(*points[0])
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(ACCENT_ORANGE))
        painter.drawEllipse(start_pt, 4.5, 4.5)

        # Robot heading marker
        rx, ry = self.odometry.x_mm, self.odometry.y_mm
        heading = self.odometry.heading_rad
        robot_screen = screen(rx, ry)
        self._draw_robot(painter, robot_screen.x(), robot_screen.y(), heading, scale)

        # HUD Overlay text
        painter.setPen(QColor(TEXT_MUTED))
        font = QFont("Segoe UI", 9, QFont.Weight.DemiBold)
        painter.setFont(font)
        painter.drawText(20, 26, f"PATH / CELL GRID ({self.cell_size_mm:.0f} mm)")

        # Display heading formatted (e.g. 0° North, L 30°, R 30°)
        deg = math.degrees(heading)
        if abs(deg) < 0.5:
            dir_str = "0.0° [N]"
        elif deg > 0:
            dir_str = f"L {deg:.1f}°"
        else:
            dir_str = f"R {abs(deg):.1f}°"

        painter.setPen(QColor(TEXT_PRIMARY))
        painter.drawText(
            int(robot_screen.x() + 18),
            int(robot_screen.y() + 14),
            dir_str,
        )

        samples_text = f"{len(points) - 1} samples"
        painter.setPen(QColor(TEXT_MUTED))
        painter.drawText(width - 120, 26, samples_text)

    def _draw_grid(
        self,
        painter: QPainter,
        width: int,
        height: int,
        scale: float,
        center_x: float,
        center_y: float,
        screen,
    ) -> None:
        cell_size = self.cell_size_mm
        spacing = cell_size * scale
        if spacing < 8.0:
            return

        left_world = center_x - width / (2.0 * scale)
        right_world = center_x + width / (2.0 * scale)
        bottom_world = center_y - height / (2.0 * scale)
        top_world = center_y + height / (2.0 * scale)

        first_x = math.floor(left_world / cell_size)
        last_x = math.ceil(right_world / cell_size)
        first_y = math.floor(bottom_world / cell_size)
        last_y = math.ceil(top_world / cell_size)

        grid_pen = QPen(QColor(GRID_COLOR), 1.0)
        axis_pen = QPen(QColor(AXIS_COLOR), 1.5)

        for gx in range(first_x, last_x + 1):
            sx = screen(gx * cell_size, center_y).x()
            painter.setPen(axis_pen if gx == 0 else grid_pen)
            painter.drawLine(int(sx), 0, int(sx), height)

        for gy in range(first_y, last_y + 1):
            sy = screen(center_x, gy * cell_size).y()
            painter.setPen(axis_pen if gy == 0 else grid_pen)
            painter.drawLine(0, int(sy), width, int(sy))

    def _draw_robot(
        self, painter: QPainter, x: float, y: float, heading: float, scale: float
    ) -> None:
        # Screen coordinates:
        # Facing +Y (North): screen dx = 0, screen dy = -1 (Up)
        # Turn Left (+heading): screen dx = -1 (Left), screen dy = 0
        # Turn Right (-heading): screen dx = +1 (Right), screen dy = 0
        fwd_x = -math.sin(heading)
        fwd_y = -math.cos(heading)
        side_x = math.cos(heading)
        side_y = -math.sin(heading)

        nose_len = max(18.0, min(32.0, scale * 26.0))
        rear_len = nose_len * 0.72
        half_w = nose_len * 0.55

        p_nose = QPointF(x + fwd_x * nose_len, y + fwd_y * nose_len)
        p_right = QPointF(
            x - fwd_x * rear_len + side_x * half_w,
            y - fwd_y * rear_len + side_y * half_w,
        )
        p_left = QPointF(
            x - fwd_x * rear_len - side_x * half_w,
            y - fwd_y * rear_len - side_y * half_w,
        )

        polygon = QPolygonF([p_nose, p_right, p_left])
        painter.setBrush(QColor(ACCENT_ORANGE))
        painter.setPen(QPen(QColor("#fff4e0"), 1.8))
        painter.drawPolygon(polygon)

    def _draw_empty_state(self, painter: QPainter, width: int, height: int) -> None:
        painter.setPen(QColor(TEXT_PRIMARY))
        font_main = QFont("Segoe UI", 14, QFont.Weight.DemiBold)
        painter.setFont(font_main)
        rect_main = QRectF(0, height * 0.5 - 35, width, 30)
        painter.drawText(
            rect_main, Qt.AlignmentFlag.AlignCenter, "Connect to the robot to view odometry"
        )

        painter.setPen(QColor(TEXT_MUTED))
        font_sub = QFont("Segoe UI", 10)
        painter.setFont(font_sub)
        rect_sub = QRectF(0, height * 0.5 + 5, width, 30)
        painter.drawText(
            rect_sub,
            Qt.AlignmentFlag.AlignCenter,
            "Default track: 112 mm  ·  Default diameter: 33 mm",
        )


class OdometryApp(QMainWindow):
    """Main application window using PyQt6."""

    MOVEMENT_KEY_MAP = {
        Qt.Key.Key_W: "w",
        Qt.Key.Key_Up: "w",
        Qt.Key.Key_S: "s",
        Qt.Key.Key_Down: "s",
        Qt.Key.Key_A: "a",
        Qt.Key.Key_Left: "a",
        Qt.Key.Key_D: "d",
        Qt.Key.Key_Right: "d",
    }

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Micromouse | Differential Drive Odometry (PyQt6)")
        self.resize(1260, 810)
        self.setMinimumSize(1020, 690)

        self.connection: Optional[serial.Serial] = None
        self.worker: Optional[SerialReaderWorker] = None
        self.write_lock = threading.Lock()
        self.odometry: Optional[DifferentialDriveOdometry] = None
        self.reset_stage: Optional[str] = None
        self.reset_deadline = 0.0

        # Multi-key tracking for simultaneous linear (W/S) and angular (A/D) velocities
        self.held_movement_keys: set[str] = set()
        self.current_v_cmd = 0.0
        self.current_w_cmd = 0.0
        self.telemetry = (0, 0, 0, 0, 0, 0, 0, 0)

        # Simulation & continuous kinematic update timer (for live response)
        self.kinematic_timer = QTimer(self)
        self.kinematic_timer.setInterval(33)  # ~30 Hz
        self.kinematic_timer.timeout.connect(self._kinematic_tick)
        self.kinematic_timer.start()

        self._setup_style()
        self._build_ui()
        self._refresh_ports()

    def _setup_style(self) -> None:
        self.setStyleSheet(f"""
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
            QFrame#sidebar {{
                background-color: {BG_PANEL};
                border-right: 1px solid {BORDER};
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
                padding: 5px 8px;
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
                padding: 6px 10px;
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
                padding: 8px 4px;
            }}
        """)

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # 1. Header
        header = QFrame(self)
        header.setObjectName("header")
        header.setFixedHeight(62)
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(20, 0, 20, 0)

        title = QLabel("MICROMOUSE / ODOMETRY", header)
        title.setFont(QFont("Segoe UI", 14, QFont.Weight.DemiBold))
        sub = QLabel("DIFFERENTIAL DRIVE KINEMATICS · LINEAR & ANGULAR SPEEDS", header)
        sub.setFont(QFont("Segoe UI", 9))
        sub.setStyleSheet(f"color: {TEXT_MUTED}; margin-left: 10px;")

        self.status_badge = QLabel("DISCONNECTED", header)
        self.status_badge.setFont(QFont("Segoe UI", 9, QFont.Weight.DemiBold))
        self.status_badge.setStyleSheet(f"color: {ACCENT_ORANGE};")

        h_layout.addWidget(title)
        h_layout.addWidget(sub)
        h_layout.addStretch()
        h_layout.addWidget(self.status_badge)
        root_layout.addWidget(header)

        # 2. Main Body (Sidebar + Viewport)
        body = QWidget(self)
        b_layout = QHBoxLayout(body)
        b_layout.setContentsMargins(0, 0, 0, 0)
        b_layout.setSpacing(0)

        sidebar = QFrame(body)
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(310)
        self._build_sidebar(sidebar)
        b_layout.addWidget(sidebar)

        right_panel = QWidget(body)
        rp_layout = QVBoxLayout(right_panel)
        rp_layout.setContentsMargins(0, 0, 0, 0)
        rp_layout.setSpacing(0)

        self.canvas = OdometryCanvas(right_panel)
        rp_layout.addWidget(self.canvas, stretch=1)

        footer = QFrame(right_panel)
        footer.setObjectName("footer")
        footer.setFixedHeight(72)
        self._build_footer(footer)
        rp_layout.addWidget(footer)

        b_layout.addWidget(right_panel, stretch=1)
        root_layout.addWidget(body, stretch=1)

    def _build_sidebar(self, sidebar: QFrame) -> None:
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        # --- SERIAL LINK ---
        sec1 = QLabel("SERIAL LINK", sidebar)
        sec1.setObjectName("sectionHeader")
        layout.addWidget(sec1)

        form1 = QGridLayout()
        form1.setVerticalSpacing(6)
        form1.setHorizontalSpacing(8)

        lbl_port = QLabel("COM port", sidebar)
        lbl_port.setStyleSheet(f"color: {TEXT_MUTED};")
        self.port_combo = QComboBox(sidebar)
        self.port_combo.setEditable(True)
        form1.addWidget(lbl_port, 0, 0)
        form1.addWidget(self.port_combo, 0, 1)

        lbl_baud = QLabel("Baud rate", sidebar)
        lbl_baud.setStyleSheet(f"color: {TEXT_MUTED};")
        self.baud_edit = QLineEdit("115200", sidebar)
        form1.addWidget(lbl_baud, 1, 0)
        form1.addWidget(self.baud_edit, 1, 1)
        layout.addLayout(form1)

        link_btns = QHBoxLayout()
        self.refresh_btn = QPushButton("Find ports", sidebar)
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QPushButton("Connect", sidebar)
        self.connect_btn.setObjectName("actionBtn")
        self.connect_btn.clicked.connect(self.toggle_connection)
        link_btns.addWidget(self.refresh_btn)
        link_btns.addWidget(self.connect_btn)
        layout.addLayout(link_btns)

        layout.addSpacing(4)

        # --- ROBOT GEOMETRY ---
        sec2 = QLabel("ROBOT GEOMETRY", sidebar)
        sec2.setObjectName("sectionHeader")
        layout.addWidget(sec2)

        form2 = QGridLayout()
        form2.setVerticalSpacing(5)
        form2.setHorizontalSpacing(8)

        self.left_cpr_edit = QLineEdit("815.70", sidebar)
        self.right_cpr_edit = QLineEdit("815.55", sidebar)
        self.wheel_diam_edit = QLineEdit("33", sidebar)  # Default 33 mm
        self.track_width_edit = QLineEdit("112", sidebar)  # Default 112 mm
        self.cell_size_edit = QLineEdit("180", sidebar)

        form2.addWidget(QLabel("Left CPR", sidebar), 0, 0)
        form2.addWidget(self.left_cpr_edit, 0, 1)
        form2.addWidget(QLabel("Right CPR", sidebar), 1, 0)
        form2.addWidget(self.right_cpr_edit, 1, 1)
        form2.addWidget(QLabel("Wheel dia / mm", sidebar), 2, 0)
        form2.addWidget(self.wheel_diam_edit, 2, 1)
        form2.addWidget(QLabel("Track width / mm", sidebar), 3, 0)
        form2.addWidget(self.track_width_edit, 3, 1)
        form2.addWidget(QLabel("Cell grid / mm", sidebar), 4, 0)
        form2.addWidget(self.cell_size_edit, 4, 1)

        for i in range(5):
            lbl = form2.itemAtPosition(i, 0).widget()
            if lbl:
                lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")

        layout.addLayout(form2)

        layout.addSpacing(4)

        # --- MOTION CONTROLS ---
        sec3 = QLabel("MOTION CONTROLS", sidebar)
        sec3.setObjectName("sectionHeader")
        layout.addWidget(sec3)

        state_card = QFrame(sidebar)
        state_card.setObjectName("stateBox")
        sc_layout = QVBoxLayout(state_card)
        sc_layout.setContentsMargins(10, 6, 10, 6)
        sc_layout.setSpacing(2)
        lbl_s_tag = QLabel("MOTION STATE", state_card)
        lbl_s_tag.setObjectName("metricTitle")
        self.motion_state_lbl = QLabel("IDLE / DISCONNECTED", state_card)
        self.motion_state_lbl.setFont(QFont("Segoe UI", 10, QFont.Weight.DemiBold))
        self.motion_state_lbl.setStyleSheet(f"color: {ACCENT_MINT};")
        sc_layout.addWidget(lbl_s_tag)
        sc_layout.addWidget(self.motion_state_lbl)
        layout.addWidget(state_card)

        # D-pad Grid
        dpad = QGridLayout()
        dpad.setSpacing(4)

        self.btn_fwd = QPushButton("▲ W", sidebar)
        self.btn_fwd.setObjectName("dpadBtn")
        self.btn_fwd.pressed.connect(lambda: self._handle_dpad_press("w"))
        self.btn_fwd.released.connect(lambda: self._handle_dpad_release("w"))

        self.btn_left = QPushButton("◀ A", sidebar)
        self.btn_left.setObjectName("dpadBtn")
        self.btn_left.pressed.connect(lambda: self._handle_dpad_press("a"))
        self.btn_left.released.connect(lambda: self._handle_dpad_release("a"))

        self.btn_stop = QPushButton("■ STOP", sidebar)
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.clicked.connect(self._manual_stop)

        self.btn_right = QPushButton("▶ D", sidebar)
        self.btn_right.setObjectName("dpadBtn")
        self.btn_right.pressed.connect(lambda: self._handle_dpad_press("d"))
        self.btn_right.released.connect(lambda: self._handle_dpad_release("d"))

        self.btn_rev = QPushButton("▼ S", sidebar)
        self.btn_rev.setObjectName("dpadBtn")
        self.btn_rev.pressed.connect(lambda: self._handle_dpad_press("s"))
        self.btn_rev.released.connect(lambda: self._handle_dpad_release("s"))

        dpad.addWidget(self.btn_fwd, 0, 1)
        dpad.addWidget(self.btn_left, 1, 0)
        dpad.addWidget(self.btn_stop, 1, 1)
        dpad.addWidget(self.btn_right, 1, 2)
        dpad.addWidget(self.btn_rev, 2, 1)
        layout.addLayout(dpad)

        # Cell and Reset
        aux_row = QHBoxLayout()
        self.btn_cell = QPushButton("1 Cell (F)", sidebar)
        self.btn_cell.clicked.connect(self._action_cell)
        self.btn_reset = QPushButton("Reset Pose (R)", sidebar)
        self.btn_reset.clicked.connect(self.reset_pose_and_encoders)
        aux_row.addWidget(self.btn_cell)
        aux_row.addWidget(self.btn_reset)
        layout.addLayout(aux_row)

        self.hold_to_drive_check = QCheckBox("Hold-to-run (stop on key release)", sidebar)
        self.hold_to_drive_check.setChecked(True)
        layout.addWidget(self.hold_to_drive_check)

        hint = QLabel(
            "Keys: W/S move · A=Left · D=Right · Space/X=Stop · F=Cell · R=Reset\n"
            "Lift key to stop instantly (teleop keyboard style).",
            sidebar,
        )
        hint.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px; line-height: 14px;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        layout.addStretch()

        self.link_note = QLabel("Connect to the board to begin odometry.", sidebar)
        self.link_note.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        self.link_note.setWordWrap(True)
        layout.addWidget(self.link_note)

        self._set_controls_enabled(False)

    def _build_footer(self, footer: QFrame) -> None:
        layout = QHBoxLayout(footer)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(10)

        self.pos_val = self._create_metric_card(layout, "POSITION / mm", "X 0.0   Y 0.0")
        self.heading_val = self._create_metric_card(layout, "HEADING", "0.0° [N]")
        self.v_val = self._create_metric_card(layout, "LINEAR SPEED (v)", "0.0 mm/s")
        self.w_val = self._create_metric_card(layout, "ANGULAR SPEED (ω)", "0.0 °/s")
        self.encoders_val = self._create_metric_card(layout, "ENCODERS (L / R)", "0 / 0")
        self.rpm_val = self._create_metric_card(layout, "MOTOR RPM (L / R)", "0 / 0")

    def _create_metric_card(self, parent_layout: QHBoxLayout, label: str, init: str) -> QLabel:
        card = QFrame(self)
        card.setObjectName("metricCard")
        c_layout = QVBoxLayout(card)
        c_layout.setContentsMargins(8, 6, 8, 6)
        c_layout.setSpacing(2)

        lbl = QLabel(label, card)
        lbl.setObjectName("metricTitle")
        val = QLabel(init, card)
        val.setObjectName("metricValue")
        c_layout.addWidget(lbl)
        c_layout.addWidget(val)
        parent_layout.addWidget(card, stretch=1)
        return val

    def _set_controls_enabled(self, enabled: bool) -> None:
        self.btn_fwd.setEnabled(enabled)
        self.btn_rev.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)
        self.btn_cell.setEnabled(enabled)
        self.btn_reset.setEnabled(enabled)

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = [port.device for port in list_ports.comports()]
        if ports:
            self.port_combo.addItems(ports)
            self.link_note.setText("Detected ports: " + ", ".join(ports))
        else:
            self.port_combo.addItem("COM5")
            self.link_note.setText("No active COM ports detected. Enter manually.")

    def _read_geometry(self) -> tuple[float, float, float, float, float]:
        try:
            left_cpr = float(self.left_cpr_edit.text().strip())
            right_cpr = float(self.right_cpr_edit.text().strip())
            wheel_dia = float(self.wheel_diam_edit.text().strip())
            track_w = float(self.track_width_edit.text().strip())
            cell_size = float(self.cell_size_edit.text().strip())
        except ValueError as exc:
            raise ValueError("All calibration parameters must be numeric values.") from exc

        if min(left_cpr, right_cpr, wheel_dia, track_w, cell_size) <= 0.0:
            raise ValueError("All calibration values must be strictly positive.")
        return left_cpr, right_cpr, wheel_dia, track_w, cell_size

    def toggle_connection(self) -> None:
        if self.connection is not None:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self) -> None:
        try:
            left_cpr, right_cpr, wheel_dia, track_w, cell_size = self._read_geometry()
            baud = int(self.baud_edit.text().strip())
            port = self.port_combo.currentText().strip()
            conn = serial.Serial(port, baud, timeout=0.1, write_timeout=0.5)
        except (ValueError, serial.SerialException) as exc:
            QMessageBox.critical(self, "Connection Failed", str(exc))
            return

        self.odometry = DifferentialDriveOdometry(left_cpr, right_cpr, wheel_dia, track_w)
        self.canvas.set_odometry(self.odometry)
        self.canvas.set_cell_size(cell_size)

        self.connection = conn
        self.worker = SerialReaderWorker(conn)
        self.worker.telemetry_received.connect(self._on_telemetry_line)
        self.worker.connection_error.connect(self._on_serial_error)
        self.worker.start()

        self.status_badge.setText(f"CONNECTED / {port}")
        self.status_badge.setStyleSheet(f"color: {ACCENT_MINT};")
        self.connect_btn.setText("Disconnect")
        self.connect_btn.setObjectName("stopBtn")
        self.connect_btn.setStyle(self.connect_btn.style())
        self.link_note.setText("Connected! Receiving telemetry...")
        self.motion_state_lbl.setText("READY / STOPPED [X]")

        self._set_controls_enabled(True)
        self.canvas.setFocus()

    def disconnect_serial(self, send_stop: bool = True) -> None:
        conn = self.connection
        self.connection = None
        self.held_movement_keys.clear()
        self.current_v_cmd = 0.0
        self.current_w_cmd = 0.0

        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(500)
            self.worker = None

        if conn is not None:
            if send_stop and conn.is_open:
                try:
                    with self.write_lock:
                        conn.write(b"x")
                except serial.SerialException:
                    pass
            try:
                conn.close()
            except serial.SerialException:
                pass

        self.status_badge.setText("DISCONNECTED")
        self.status_badge.setStyleSheet(f"color: {ACCENT_ORANGE};")
        self.connect_btn.setText("Connect")
        self.connect_btn.setObjectName("actionBtn")
        self.connect_btn.setStyle(self.connect_btn.style())
        self.motion_state_lbl.setText("IDLE / DISCONNECTED")
        self.link_note.setText("Disconnected. The last path remains visible.")

        self._set_controls_enabled(False)

    def send_command(self, cmd: str) -> bool:
        conn = self.connection
        if conn is None or not conn.is_open:
            return False
        try:
            with self.write_lock:
                conn.write(cmd.encode("ascii"))
            return True
        except serial.SerialException as exc:
            self._on_serial_error(str(exc))
            return False

    # --- Teleop & Simultaneous Multi-Key Handling (Differential Drive Kinematics) ---

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.isAutoRepeat():
            return

        focused = QApplication.focusWidget()
        if isinstance(focused, (QLineEdit, QComboBox)):
            if event.key() in (Qt.Key.Key_Escape, Qt.Key.Key_Return, Qt.Key.Key_Enter):
                self.canvas.setFocus()
            super().keyPressEvent(event)
            return

        key = event.key()

        if key in self.MOVEMENT_KEY_MAP:
            cmd = self.MOVEMENT_KEY_MAP[key]
            self.held_movement_keys.add(cmd)
            self._process_movement_keys()

        elif key in (Qt.Key.Key_Space, Qt.Key.Key_X, Qt.Key.Key_Escape):
            self._manual_stop()

        elif key == Qt.Key.Key_F:
            self._action_cell()

        elif key == Qt.Key.Key_R:
            self.reset_pose_and_encoders()

        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.isAutoRepeat():
            return

        focused = QApplication.focusWidget()
        if isinstance(focused, (QLineEdit, QComboBox)):
            super().keyReleaseEvent(event)
            return

        key = event.key()

        if key in self.MOVEMENT_KEY_MAP:
            cmd = self.MOVEMENT_KEY_MAP[key]
            self.held_movement_keys.discard(cmd)
            if self.hold_to_drive_check.isChecked() or not self.held_movement_keys:
                self._process_movement_keys()
        else:
            super().keyReleaseEvent(event)

    def _process_movement_keys(self) -> None:
        """Derive simultaneous linear velocity (v) and angular velocity (w) from active keys."""
        keys = self.held_movement_keys

        if not self.hold_to_drive_check.isChecked() and not keys:
            return

        has_w = "w" in keys
        has_s = "s" in keys
        has_a = "a" in keys
        has_d = "d" in keys

        if has_w and not has_s:
            v_sign = 1.0
        elif has_s and not has_w:
            v_sign = -1.0
        else:
            v_sign = 0.0

        if has_a and not has_d:
            w_sign = 1.0  # Turn Left (CCW)
        elif has_d and not has_a:
            w_sign = -1.0 # Turn Right (CW)
        else:
            w_sign = 0.0

        track = 112.0
        wheel_dia = 33.0
        if self.odometry is not None:
            track = self.odometry.track_width_mm
            wheel_dia = self.odometry.wheel_diameter_mm

        # Full speed: 150 RPM * pi * D / 60
        # Curve slow inner wheel: 60 RPM * pi * D / 60
        v_fast = (150.0 / 60.0) * math.pi * wheel_dia
        v_slow = (60.0 / 60.0) * math.pi * wheel_dia

        if v_sign != 0.0 and w_sign != 0.0:
            # BOTH linear (W/S) and angular (A/D) active: SMOOTH ARC CURVE
            # Outer wheel = 150 RPM, Inner wheel = 60 RPM
            v_cmd = v_sign * ((v_fast + v_slow) * 0.5)
            w_cmd = w_sign * ((v_fast - v_slow) / track)

            if v_sign > 0 and w_sign > 0:
                serial_cmd = "q"
                label = "FORWARD + TURN LEFT [W + A]"
            elif v_sign > 0 and w_sign < 0:
                serial_cmd = "e"
                label = "FORWARD + TURN RIGHT [W + D]"
            elif v_sign < 0 and w_sign > 0:
                serial_cmd = "z"
                label = "BACKWARD + TURN LEFT [S + A]"
            else:
                serial_cmd = "c"
                label = "BACKWARD + TURN RIGHT [S + D]"
        elif v_sign != 0.0:
            # Pure linear
            v_cmd = v_sign * v_fast
            w_cmd = 0.0
            serial_cmd = "w" if v_sign > 0 else "s"
            label = "DRIVING FORWARD [W]" if v_sign > 0 else "DRIVING BACKWARD [S]"
        elif w_sign != 0.0:
            # Pure angular (in-place spin: 150 RPM in opposite directions)
            v_cmd = 0.0
            w_cmd = w_sign * (2.0 * v_fast / track)
            serial_cmd = "a" if w_sign > 0 else "d"
            label = "SPINNING LEFT [A]" if w_sign > 0 else "SPINNING RIGHT [D]"
        else:
            # All released or conflicting
            v_cmd = 0.0
            w_cmd = 0.0
            serial_cmd = "x"
            label = "STOPPED [Space/X]"

        self.current_v_cmd = v_cmd
        self.current_w_cmd = w_cmd
        self.send_command(serial_cmd)
        self.motion_state_lbl.setText(label)
        self._highlight_active_buttons(v_sign, w_sign)

        # Update real-time metric cards with commanded speeds
        if self.connection is None or not self.connection.is_open:
            self.v_val.setText(f"{v_cmd:.1f} mm/s")
            self.w_val.setText(f"{math.degrees(w_cmd):.1f} °/s")

    def _highlight_active_buttons(self, v_sign: float, w_sign: float) -> None:
        active_style = f"background-color: {ACCENT_MINT}; color: #0b1a15;"
        self.btn_fwd.setStyleSheet(active_style if v_sign > 0 else "")
        self.btn_rev.setStyleSheet(active_style if v_sign < 0 else "")
        self.btn_left.setStyleSheet(active_style if w_sign > 0 else "")
        self.btn_right.setStyleSheet(active_style if w_sign < 0 else "")

    def _kinematic_tick(self) -> None:
        """Live kinematic update when testing/driving without active serial feedback."""
        if self.connection is not None and self.connection.is_open:
            return
        if self.odometry is None:
            return
        if abs(self.current_v_cmd) > 1e-3 or abs(self.current_w_cmd) > 1e-3:
            moved = self.odometry.update_from_velocities(
                self.current_v_cmd, self.current_w_cmd, dt=0.033
            )
            if moved:
                self._update_pose_labels()
                self.canvas.update()

    # Mouse D-Pad actions
    def _handle_dpad_press(self, cmd: str) -> None:
        self.canvas.setFocus()
        self.held_movement_keys.add(cmd)
        self._process_movement_keys()

    def _handle_dpad_release(self, cmd: str) -> None:
        self.held_movement_keys.discard(cmd)
        self._process_movement_keys()

    def _manual_stop(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()

    def _action_cell(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()
        self.send_command("f")
        self.motion_state_lbl.setText("CELL MOVE [F]")

    def reset_pose_and_encoders(self) -> None:
        self.held_movement_keys.clear()
        self.current_v_cmd = 0.0
        self.current_w_cmd = 0.0
        self._highlight_active_buttons(0, 0)
        self.send_command("x")
        if self.odometry is not None:
            self.odometry.reset()
        self.reset_stage = "stopping"
        self.reset_deadline = time.monotonic() + 3.0
        self.motion_state_lbl.setText("RESETTING...")
        self.link_note.setText("Stopping, then resetting encoder totals...")
        QTimer.singleShot(180, self._send_encoder_reset)
        self.canvas.update()
        self._update_pose_labels()

    def _send_encoder_reset(self) -> None:
        if self.connection is None:
            self.reset_stage = None
            return
        if self.reset_stage == "stopping":
            if self.send_command("r"):
                self.reset_stage = "baseline"
                self.link_note.setText("Waiting for reset counts from firmware...")
            else:
                self.reset_stage = None

    # --- Telemetry processing ---

    def _on_serial_error(self, message: str) -> None:
        self.disconnect_serial(send_stop=False)
        self.link_note.setText(f"Serial Error: {message}")

    def _on_telemetry_line(self, line: str) -> None:
        match = TELEMETRY_PATTERN.search(line)
        if match is None:
            return

        parsed = tuple(int(v) for v in match.groups())
        self.telemetry = parsed
        (
            left_count,
            _l_delta,
            left_rpm,
            left_pwm,
            right_count,
            _r_delta,
            right_rpm,
            right_pwm,
        ) = parsed

        self.encoders_val.setText(f"{left_count} / {right_count}")
        self.rpm_val.setText(f"{left_rpm} / {right_rpm} RPM")

        if self.odometry is None:
            return

        if self.reset_stage == "stopping":
            return

        if self.reset_stage == "baseline":
            if abs(left_count) <= 120 and abs(right_count) <= 120:
                self.odometry.set_baseline(left_count, right_count)
                self.reset_stage = None
                self.link_note.setText("Pose and firmware encoder totals reset.")
                self.motion_state_lbl.setText("STOPPED [Space/X]")
                self._update_pose_labels()
                self.canvas.update()
            elif time.monotonic() >= self.reset_deadline:
                self.odometry.set_baseline(left_count, right_count)
                self.reset_stage = None
                self.link_note.setText("Reset timed out; baseline restarted.")
                self.motion_state_lbl.setText("STOPPED [Space/X]")
                self._update_pose_labels()
                self.canvas.update()
            return

        moved = self.odometry.update_counts(left_count, right_count)
        self._update_pose_labels()

        # Compute kinematic linear and angular speeds from filtered RPM
        circumference = self.odometry.wheel_circumference_mm
        track = self.odometry.track_width_mm
        v_l = (left_rpm / 60.0) * circumference
        v_r = (right_rpm / 60.0) * circumference
        v_linear = (v_r + v_l) * 0.5
        w_rad = (v_r - v_l) / track
        w_deg = math.degrees(w_rad)

        self.v_val.setText(f"{v_linear:.1f} mm/s")
        self.w_val.setText(f"{w_deg:.1f} °/s")

        if moved:
            self.canvas.update()

    def _update_pose_labels(self) -> None:
        if self.odometry is None:
            return

        # Position
        self.pos_val.setText(f"X {self.odometry.x_mm:.1f}   Y {self.odometry.y_mm:.1f}")

        # Heading with direction label
        deg = math.degrees(self.odometry.heading_rad)
        if abs(deg) < 0.5:
            head_str = "0.0° [N]"
        elif deg > 0:
            head_str = f"L {deg:.1f}°"
        else:
            head_str = f"R {abs(deg):.1f}°"
        self.heading_val.setText(head_str)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.disconnect_serial()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    window = OdometryApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

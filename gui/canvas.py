"""Hardware-accelerated 2D viewport for dead-reckoning trajectory and cell grid."""

from __future__ import annotations

import math
from typing import Optional

from PyQt6 import QtGui
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import QWidget

from gui.config import (
    ACCENT_MINT,
    ACCENT_ORANGE,
    AXIS_COLOR,
    BG_CANVAS,
    DEFAULT_CELL_SIZE_MM,
    GRID_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
)
from gui.kinematics import DifferentialDriveOdometry


class OdometryCanvas(QWidget):
    """Smooth, hardware-accelerated 2D viewport for dead-reckoning trajectory."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)
        self.odometry: Optional[DifferentialDriveOdometry] = None
        self.cell_size_mm = DEFAULT_CELL_SIZE_MM

    def set_odometry(self, odometry: Optional[DifferentialDriveOdometry]) -> None:
        """Sets the active odometry kinematic model."""
        self.odometry = odometry
        self.update()

    def set_cell_size(self, cell_size_mm: float) -> None:
        """Updates grid cell size in millimeters."""
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

        # 1. Draw coordinate grid
        self._draw_grid(painter, width, height, scale, center_x, center_y, screen)

        # 2. Draw trajectory path
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

        # 3. Origin start marker
        start_pt = screen(*points[0])
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(ACCENT_ORANGE))
        painter.drawEllipse(start_pt, 4.5, 4.5)

        # 4. Robot heading marker
        rx, ry = self.odometry.x_mm, self.odometry.y_mm
        heading = self.odometry.heading_rad
        robot_screen = screen(rx, ry)
        self._draw_robot(painter, robot_screen.x(), robot_screen.y(), heading, scale)

        # 5. Top HUD Overlay
        painter.setPen(QColor(TEXT_MUTED))
        font = QFont("Segoe UI", 9, QFont.Weight.DemiBold)
        painter.setFont(font)
        painter.drawText(20, 26, f"PATH / CELL GRID ({self.cell_size_mm:.1f} mm = 1 Cell)")

        # Formatted heading
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

"""Sidebar controls panel containing serial connection, geometry, and motion triggers."""

from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from gui.config import (
    ACCENT_MINT,
    DEFAULT_BAUD,
    DEFAULT_CELL_SIZE_MM,
    DEFAULT_LEFT_CPR,
    DEFAULT_RIGHT_CPR,
    DEFAULT_TRACK_WIDTH_MM,
    DEFAULT_WHEEL_DIAMETER_MM,
    TEXT_MUTED,
)


class SidebarWidget(QScrollArea):
    """Scrollable sidebar container for connection, calibration, and robot motion controls."""

    # UI Event Signals
    connect_requested = pyqtSignal()
    disconnect_requested = pyqtSignal()
    refresh_ports_requested = pyqtSignal()
    recalculate_cell_requested = pyqtSignal()

    dpad_pressed = pyqtSignal(str)
    dpad_released = pyqtSignal(str)
    stop_requested = pyqtSignal()

    cell_fwd_requested = pyqtSignal()
    cell_back_requested = pyqtSignal()
    turn_90_l_requested = pyqtSignal()
    turn_90_r_requested = pyqtSignal()
    turn_180_requested = pyqtSignal()
    reset_pose_requested = pyqtSignal()

    # Maze signals
    load_maze_requested = pyqtSignal(str)
    place_robot_origin_requested = pyqtSignal()
    clear_maze_requested = pyqtSignal()

    cell_size_changed = pyqtSignal(float)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("sidebarScroll")
        self.setWidgetResizable(True)
        self.setFixedWidth(330)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setFrameShape(QFrame.Shape.NoFrame)

        content = QWidget()
        content.setObjectName("sidebar")
        self._build_content(content)
        self.setWidget(content)

    def _build_content(self, sidebar: QWidget) -> None:
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        # 1. SERIAL LINK
        sec1 = QLabel("SERIAL LINK", sidebar)
        sec1.setObjectName("sectionHeader")
        layout.addWidget(sec1)

        form1 = QGridLayout()
        form1.setVerticalSpacing(6)
        form1.setHorizontalSpacing(8)

        lbl_port = QLabel("COM port", sidebar)
        lbl_port.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        self.port_combo = QComboBox(sidebar)
        self.port_combo.setEditable(True)
        self.port_combo.setFixedHeight(28)
        form1.addWidget(lbl_port, 0, 0)
        form1.addWidget(self.port_combo, 0, 1)

        lbl_baud = QLabel("Baud rate", sidebar)
        lbl_baud.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")
        self.baud_edit = QLineEdit(str(DEFAULT_BAUD), sidebar)
        self.baud_edit.setFixedHeight(28)
        form1.addWidget(lbl_baud, 1, 0)
        form1.addWidget(self.baud_edit, 1, 1)
        layout.addLayout(form1)

        link_btns = QHBoxLayout()
        self.refresh_btn = QPushButton("Find ports", sidebar)
        self.refresh_btn.setFixedHeight(28)
        self.refresh_btn.clicked.connect(self.refresh_ports_requested.emit)
        self.connect_btn = QPushButton("Connect", sidebar)
        self.connect_btn.setFixedHeight(28)
        self.connect_btn.setObjectName("actionBtn")
        self.connect_btn.clicked.connect(self._on_connect_toggle)
        link_btns.addWidget(self.refresh_btn)
        link_btns.addWidget(self.connect_btn)
        layout.addLayout(link_btns)

        layout.addSpacing(2)

        # 2. ROBOT GEOMETRY
        sec2 = QLabel("ROBOT GEOMETRY", sidebar)
        sec2.setObjectName("sectionHeader")
        layout.addWidget(sec2)

        form2 = QGridLayout()
        form2.setVerticalSpacing(6)
        form2.setHorizontalSpacing(8)
        form2.setColumnStretch(0, 0)
        form2.setColumnStretch(1, 1)

        self.left_cpr_edit = QLineEdit(f"{DEFAULT_LEFT_CPR:.2f}", sidebar)
        self.left_cpr_edit.setFixedHeight(28)
        self.right_cpr_edit = QLineEdit(f"{DEFAULT_RIGHT_CPR:.2f}", sidebar)
        self.right_cpr_edit.setFixedHeight(28)
        self.wheel_diam_edit = QLineEdit(f"{DEFAULT_WHEEL_DIAMETER_MM:.0f}", sidebar)
        self.wheel_diam_edit.setFixedHeight(28)
        self.track_width_edit = QLineEdit(f"{DEFAULT_TRACK_WIDTH_MM:.0f}", sidebar)
        self.track_width_edit.setFixedHeight(28)

        cell_box = QHBoxLayout()
        cell_box.setSpacing(6)
        cell_box.setContentsMargins(0, 0, 0, 0)
        self.cell_size_edit = QLineEdit(f"{DEFAULT_CELL_SIZE_MM:.1f}", sidebar)
        self.cell_size_edit.setFixedHeight(28)
        self.cell_size_edit.textChanged.connect(self._on_cell_text_changed)
        btn_calc_cell = QPushButton("Calc (F)", sidebar)
        btn_calc_cell.setFixedHeight(28)
        btn_calc_cell.setToolTip("Calculate 1 cell distance (1060 ticks) from CPR & wheel diameter")
        btn_calc_cell.setStyleSheet("padding: 3px 6px; font-size: 11px;")
        btn_calc_cell.clicked.connect(self.recalculate_cell_size)
        cell_box.addWidget(self.cell_size_edit, stretch=1)
        cell_box.addWidget(btn_calc_cell)

        lbl_lcpr = QLabel("Left CPR:", sidebar)
        lbl_rcpr = QLabel("Right CPR:", sidebar)
        lbl_wdia = QLabel("Wheel dia (mm):", sidebar)
        lbl_tw = QLabel("Track (mm):", sidebar)
        lbl_cg = QLabel("Cell grid (mm):", sidebar)

        for lbl in (lbl_lcpr, lbl_rcpr, lbl_wdia, lbl_tw, lbl_cg):
            lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 12px;")

        form2.addWidget(lbl_lcpr, 0, 0)
        form2.addWidget(self.left_cpr_edit, 0, 1)
        form2.addWidget(lbl_rcpr, 1, 0)
        form2.addWidget(self.right_cpr_edit, 1, 1)
        form2.addWidget(lbl_wdia, 2, 0)
        form2.addWidget(self.wheel_diam_edit, 2, 1)
        form2.addWidget(lbl_tw, 3, 0)
        form2.addWidget(self.track_width_edit, 3, 1)
        form2.addWidget(lbl_cg, 4, 0)
        form2.addLayout(cell_box, 4, 1)

        layout.addLayout(form2)

        layout.addSpacing(2)

        # 3. MAZE ENVIRONMENT
        sec_maze = QLabel("MAZE ENVIRONMENT", sidebar)
        sec_maze.setObjectName("sectionHeader")
        layout.addWidget(sec_maze)

        form_maze = QVBoxLayout()
        form_maze.setSpacing(6)

        self.maze_combo = QComboBox(sidebar)
        self.maze_combo.setFixedHeight(28)
        self.maze_combo.addItems([
            "6x6 Test Maze",
            "Empty 6x6 Maze",
            "10x10 Test Maze",
            "16x16 Competition Maze",
        ])
        form_maze.addWidget(self.maze_combo)

        maze_btn_box = QHBoxLayout()
        maze_btn_box.setSpacing(6)
        self.btn_load_maze = QPushButton("Load Maze", sidebar)
        self.btn_load_maze.setFixedHeight(28)
        self.btn_load_maze.setObjectName("actionBtn")
        self.btn_load_maze.clicked.connect(self._on_load_maze_clicked)

        self.btn_place_start = QPushButton("Place at (0,0)", sidebar)
        self.btn_place_start.setFixedHeight(28)
        self.btn_place_start.setToolTip("Places robot at the center of cell (0,0) facing North")
        self.btn_place_start.clicked.connect(self.place_robot_origin_requested.emit)

        maze_btn_box.addWidget(self.btn_load_maze)
        maze_btn_box.addWidget(self.btn_place_start)
        form_maze.addLayout(maze_btn_box)

        self.maze_status_lbl = QLabel("Maze: Not loaded", sidebar)
        self.maze_status_lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        form_maze.addWidget(self.maze_status_lbl)

        layout.addLayout(form_maze)

        layout.addSpacing(2)

        # 4. CONTINUOUS DRIVE (WASD)
        sec3 = QLabel("CONTINUOUS DRIVE (WASD)", sidebar)
        sec3.setObjectName("sectionHeader")
        layout.addWidget(sec3)

        state_card = QFrame(sidebar)
        state_card.setObjectName("stateBox")
        sc_layout = QVBoxLayout(state_card)
        sc_layout.setContentsMargins(10, 6, 10, 6)
        sc_layout.setSpacing(2)
        lbl_s_tag = QLabel("MOTION STATE", state_card)
        lbl_s_tag.setObjectName("metricTitle")
        self.motion_state_lbl = QLabel("READY / STOPPED [X]", state_card)
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
        self.btn_fwd.setFixedHeight(30)
        self.btn_fwd.pressed.connect(lambda: self.dpad_pressed.emit("w"))
        self.btn_fwd.released.connect(lambda: self.dpad_released.emit("w"))

        self.btn_left = QPushButton("◀ A", sidebar)
        self.btn_left.setObjectName("dpadBtn")
        self.btn_left.setFixedHeight(30)
        self.btn_left.pressed.connect(lambda: self.dpad_pressed.emit("a"))
        self.btn_left.released.connect(lambda: self.dpad_released.emit("a"))

        self.btn_stop = QPushButton("■ STOP", sidebar)
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.setFixedHeight(30)
        self.btn_stop.clicked.connect(self.stop_requested.emit)

        self.btn_right = QPushButton("▶ D", sidebar)
        self.btn_right.setObjectName("dpadBtn")
        self.btn_right.setFixedHeight(30)
        self.btn_right.pressed.connect(lambda: self.dpad_pressed.emit("d"))
        self.btn_right.released.connect(lambda: self.dpad_released.emit("d"))

        self.btn_rev = QPushButton("▼ S", sidebar)
        self.btn_rev.setObjectName("dpadBtn")
        self.btn_rev.setFixedHeight(30)
        self.btn_rev.pressed.connect(lambda: self.dpad_pressed.emit("s"))
        self.btn_rev.released.connect(lambda: self.dpad_released.emit("s"))

        dpad.addWidget(self.btn_fwd, 0, 1)
        dpad.addWidget(self.btn_left, 1, 0)
        dpad.addWidget(self.btn_stop, 1, 1)
        dpad.addWidget(self.btn_right, 1, 2)
        dpad.addWidget(self.btn_rev, 2, 1)
        layout.addLayout(dpad)

        self.hold_to_drive_check = QCheckBox("Hold-to-run (stop on key release)", sidebar)
        self.hold_to_drive_check.setChecked(True)
        layout.addWidget(self.hold_to_drive_check)

        layout.addSpacing(2)

        # 4. DISCRETE MOTIONS
        sec4 = QLabel("DISCRETE MOTIONS", sidebar)
        sec4.setObjectName("sectionHeader")
        layout.addWidget(sec4)

        disc_grid = QGridLayout()
        disc_grid.setSpacing(4)

        self.btn_cell_fwd = QPushButton("▲ 1 Cell Fwd (F)", sidebar)
        self.btn_cell_fwd.setFixedHeight(30)
        self.btn_cell_fwd.clicked.connect(self.cell_fwd_requested.emit)

        self.btn_cell_back = QPushButton("▼ 1 Cell Back (B)", sidebar)
        self.btn_cell_back.setFixedHeight(30)
        self.btn_cell_back.clicked.connect(self.cell_back_requested.emit)

        self.btn_turn_90_l = QPushButton("↶ 90° Left (J)", sidebar)
        self.btn_turn_90_l.setFixedHeight(30)
        self.btn_turn_90_l.clicked.connect(self.turn_90_l_requested.emit)

        self.btn_turn_90_r = QPushButton("↷ 90° Right (K)", sidebar)
        self.btn_turn_90_r.setFixedHeight(30)
        self.btn_turn_90_r.clicked.connect(self.turn_90_r_requested.emit)

        self.btn_turn_180 = QPushButton("↺ Turn 180° (U)", sidebar)
        self.btn_turn_180.setFixedHeight(30)
        self.btn_turn_180.clicked.connect(self.turn_180_requested.emit)

        self.btn_reset = QPushButton("⟲ Reset (R)", sidebar)
        self.btn_reset.setFixedHeight(30)
        self.btn_reset.clicked.connect(self.reset_pose_requested.emit)

        disc_grid.addWidget(self.btn_cell_fwd, 0, 0)
        disc_grid.addWidget(self.btn_cell_back, 0, 1)
        disc_grid.addWidget(self.btn_turn_90_l, 1, 0)
        disc_grid.addWidget(self.btn_turn_90_r, 1, 1)
        disc_grid.addWidget(self.btn_turn_180, 2, 0)
        disc_grid.addWidget(self.btn_reset, 2, 1)
        layout.addLayout(disc_grid)

        hint = QLabel(
            "Controls:\n"
            "• W/S/A/D: teleop drive (lift key to stop)\n"
            "• F: 1 Cell Fwd · B: 1 Cell Back\n"
            "• J: Turn 90° Left · K: Turn 90° Right · U: Turn 180°\n"
            "• Space/X: Stop · R: Reset origin",
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

    # Calibration & Cell Size Recalculation
    def compute_cell_distance_mm(self) -> float:
        """Calculates 1 cell distance (1060 ticks) based on current CPR and wheel diameter."""
        try:
            lcpr = float(self.left_cpr_edit.text().strip())
            rcpr = float(self.right_cpr_edit.text().strip())
            wdia = float(self.wheel_diam_edit.text().strip())
            avg_cpr = (lcpr + rcpr) * 0.5
            if avg_cpr > 0:
                return (1060.0 / avg_cpr) * math.pi * wdia
        except (ValueError, ZeroDivisionError):
            pass
        return DEFAULT_CELL_SIZE_MM

    def recalculate_cell_size(self) -> None:
        """Computes and updates cell size input field."""
        dist = self.compute_cell_distance_mm()
        self.cell_size_edit.setText(f"{dist:.1f}")
        self.cell_size_changed.emit(dist)

    def _on_cell_text_changed(self, text: str) -> None:
        try:
            val = float(text.strip())
            if val > 1.0:
                self.cell_size_changed.emit(val)
        except ValueError:
            pass

    def get_geometry(self) -> tuple[float, float, float, float, float]:
        """Reads and validates calibration values."""
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

    def get_cell_distance_mm(self) -> float:
        """Returns the currently configured cell size in mm."""
        try:
            return float(self.cell_size_edit.text().strip())
        except ValueError:
            return DEFAULT_CELL_SIZE_MM

    def _on_connect_toggle(self) -> None:
        if self.connect_btn.text() == "Connect":
            self.connect_requested.emit()
        else:
            self.disconnect_requested.emit()

    def set_connected(self, is_connected: bool, port: str = "") -> None:
        """Updates button label and appearance based on serial state."""
        if is_connected:
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setObjectName("stopBtn")
            self.connect_btn.setStyle(self.connect_btn.style())
            self.link_note.setText("Connected! Receiving telemetry...")
            self.motion_state_lbl.setText("READY / STOPPED [X]")
        else:
            self.connect_btn.setText("Connect")
            self.connect_btn.setObjectName("actionBtn")
            self.connect_btn.setStyle(self.connect_btn.style())
            self.motion_state_lbl.setText("READY / STOPPED [X]")
            self.link_note.setText("Disconnected. The last path remains visible.")

    def set_motion_label(self, text: str) -> None:
        """Sets the active motion state description."""
        self.motion_state_lbl.setText(text)

    def set_note(self, text: str) -> None:
        """Sets the lower status note."""
        self.link_note.setText(text)

    def highlight_dpad(self, v_sign: float, w_sign: float) -> None:
        """Visual feedback on D-Pad buttons when active."""
        active_style = f"background-color: {ACCENT_MINT}; color: #0b1a15;"
        self.btn_fwd.setStyleSheet(active_style if v_sign > 0 else "")
        self.btn_rev.setStyleSheet(active_style if v_sign < 0 else "")
        self.btn_left.setStyleSheet(active_style if w_sign > 0 else "")
        self.btn_right.setStyleSheet(active_style if w_sign < 0 else "")

    def _on_load_maze_clicked(self) -> None:
        self.load_maze_requested.emit(self.maze_combo.currentText())

    def set_maze_status(self, text: str) -> None:
        """Updates the maze status label."""
        self.maze_status_lbl.setText(text)

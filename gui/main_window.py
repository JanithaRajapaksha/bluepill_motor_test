"""Main application window coordinating UI modules, serial telemetry, and kinematics."""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import Optional

from PyQt6 import QtGui
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QVBoxLayout,
    QWidget,
)
import serial
from serial.tools import list_ports

from gui.canvas import OdometryCanvas
from gui.config import (
    ACCENT_MINT,
    ACCENT_ORANGE,
    DEFAULT_BAUD,
    DEFAULT_CELL_SIZE_MM,
    DEFAULT_LEFT_CPR,
    DEFAULT_RIGHT_CPR,
    DEFAULT_TRACK_WIDTH_MM,
    DEFAULT_WHEEL_DIAMETER_MM,
    FLOODFILL_STEP_PATTERN,
    TELEMETRY_PATTERN,
    get_app_stylesheet,
)
from gui.footer import FooterWidget
from gui.kinematics import DifferentialDriveOdometry
from gui.serial_worker import SerialReaderWorker
from gui.sidebar import SidebarWidget
from navigation_engine import NavigationEngine, query_cell_relative_walls


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

        # Initialize default odometry so the canvas is active immediately
        self.odometry = DifferentialDriveOdometry(
            DEFAULT_LEFT_CPR,
            DEFAULT_RIGHT_CPR,
            DEFAULT_WHEEL_DIAMETER_MM,
            DEFAULT_TRACK_WIDTH_MM,
        )

        # Unified Navigation Engine (shared directly with verify_stm32_floodfill.py)
        self.nav_engine = NavigationEngine()
        self.nav_engine.on_wall_request = self._on_engine_wall_request
        self.nav_engine.on_cell_arrival = self._on_engine_cell_arrival
        self.nav_engine.on_step = self._on_engine_step
        self.nav_engine.on_goal = self._on_engine_goal
        self.nav_engine.on_status = self._on_engine_status
        self.nav_engine.on_telemetry = self._on_engine_telemetry

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
        self._on_load_maze("6x6 Test Maze")

    def _setup_style(self) -> None:
        self.setStyleSheet(get_app_stylesheet())

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # 1. Header Bar
        header = QFrame(central)
        header.setObjectName("header")
        header.setFixedHeight(54)
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(18, 0, 18, 0)

        title = QLabel("MICROMOUSE ODOMETRY & CONTROL", header)
        title.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        sub = QLabel("Differential-Drive Kinematics Viewer", header)
        sub.setFont(QFont("Segoe UI", 10))
        sub.setStyleSheet(f"color: {ACCENT_MINT}; margin-left: 8px;")

        self.status_badge = QLabel("DISCONNECTED", header)
        self.status_badge.setFont(QFont("Segoe UI", 10, QFont.Weight.DemiBold))
        self.status_badge.setStyleSheet(f"color: {ACCENT_ORANGE};")

        h_layout.addWidget(title)
        h_layout.addWidget(sub)
        h_layout.addStretch()
        h_layout.addWidget(self.status_badge)
        root_layout.addWidget(header)

        # 2. Body (Sidebar + Viewport + Footer)
        body = QWidget(self)
        b_layout = QHBoxLayout(body)
        b_layout.setContentsMargins(0, 0, 0, 0)
        b_layout.setSpacing(0)

        # Sidebar component
        self.sidebar = SidebarWidget(body)
        b_layout.addWidget(self.sidebar)

        # Center Canvas & Footer
        right_panel = QWidget(body)
        rp_layout = QVBoxLayout(right_panel)
        rp_layout.setContentsMargins(0, 0, 0, 0)
        rp_layout.setSpacing(0)

        self.canvas = OdometryCanvas(right_panel)
        self.canvas.set_odometry(self.odometry)
        self.canvas.set_cell_size(DEFAULT_CELL_SIZE_MM)
        rp_layout.addWidget(self.canvas, stretch=1)

        self.footer = FooterWidget(right_panel)
        rp_layout.addWidget(self.footer)

        self._connect_sidebar_signals()

        b_layout.addWidget(right_panel, stretch=1)
        root_layout.addWidget(body, stretch=1)

    def _connect_sidebar_signals(self) -> None:
        sb = self.sidebar
        sb.connect_requested.connect(self.connect_serial)
        sb.disconnect_requested.connect(self.disconnect_serial)
        sb.refresh_ports_requested.connect(self._refresh_ports)
        sb.cell_size_changed.connect(self.canvas.set_cell_size)

        sb.dpad_pressed.connect(self._handle_dpad_press)
        sb.dpad_released.connect(self._handle_dpad_release)
        sb.stop_requested.connect(self._manual_stop)

        sb.cell_fwd_requested.connect(self._action_cell_forward)
        sb.cell_back_requested.connect(self._action_cell_backward)
        sb.turn_90_l_requested.connect(self._action_turn_90_left)
        sb.turn_90_r_requested.connect(self._action_turn_90_right)
        sb.turn_180_requested.connect(self._action_turn_180)
        sb.reset_pose_requested.connect(self.reset_pose_and_encoders)

        sb.load_maze_requested.connect(self._on_load_maze)
        sb.place_robot_origin_requested.connect(self.place_robot_at_cell_0_0)
        sb.clear_maze_requested.connect(self._on_clear_maze)
        sb.solve_maze_requested.connect(self._on_solve_maze_clicked)

    def _refresh_ports(self) -> None:
        self.sidebar.port_combo.clear()
        ports = [port.device for port in list_ports.comports()]
        if ports:
            self.sidebar.port_combo.addItems(ports)
            self.sidebar.set_note("Detected ports: " + ", ".join(ports))
        else:
            self.sidebar.port_combo.addItem("COM5")
            self.sidebar.set_note("No active COM ports detected. Enter manually.")

    def connect_serial(self) -> None:
        try:
            left_cpr, right_cpr, wheel_dia, track_w, cell_size = (
                self.sidebar.get_geometry()
            )
            baud = int(self.sidebar.baud_edit.text().strip())
            port = self.sidebar.port_combo.currentText().strip()
            conn = serial.Serial(port, baud, timeout=0.1, write_timeout=0.5)
        except (ValueError, serial.SerialException) as exc:
            QMessageBox.critical(self, "Connection Failed", str(exc))
            return

        self.odometry = DifferentialDriveOdometry(
            left_cpr, right_cpr, wheel_dia, track_w
        )
        if self.canvas.maze is not None:
            self.odometry.set_pose(cell_size * 0.5, cell_size * 0.5, 0.0)
        self.canvas.set_odometry(self.odometry)
        self.canvas.set_cell_size(cell_size)

        self.connection = conn
        self.worker = SerialReaderWorker(conn)
        self.worker.telemetry_received.connect(self._on_telemetry_line)
        self.worker.connection_error.connect(self._on_serial_error)
        self.worker.start()

        self.status_badge.setText(f"CONNECTED / {port}")
        self.status_badge.setStyleSheet(f"color: {ACCENT_MINT};")
        self.sidebar.set_connected(True, port)
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
        self.sidebar.set_connected(False)

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

    # Keyboard & Teleop Handlers
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
            self._action_cell_forward()

        elif key == Qt.Key.Key_B:
            self._action_cell_backward()

        elif key in (Qt.Key.Key_J, Qt.Key.Key_BracketLeft):
            self._action_turn_90_left()

        elif key in (Qt.Key.Key_K, Qt.Key.Key_BracketRight):
            self._action_turn_90_right()

        elif key == Qt.Key.Key_U:
            self._action_turn_180()

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
            if (
                self.sidebar.hold_to_drive_check.isChecked()
                or not self.held_movement_keys
            ):
                self._process_movement_keys()
        else:
            super().keyReleaseEvent(event)

    def _process_movement_keys(self) -> None:
        """Derive simultaneous linear velocity (v) and angular velocity (w) from active keys."""
        keys = self.held_movement_keys
        if not self.sidebar.hold_to_drive_check.isChecked() and not keys:
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
            w_sign = -1.0  # Turn Right (CW)
        else:
            w_sign = 0.0

        track = self.odometry.track_width_mm
        wheel_dia = self.odometry.wheel_diameter_mm

        v_fast = (150.0 / 60.0) * math.pi * wheel_dia
        v_slow = (60.0 / 60.0) * math.pi * wheel_dia

        if v_sign != 0.0 and w_sign != 0.0:
            # Smooth arc curve
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
            # Pure angular (in-place spin)
            v_cmd = 0.0
            w_cmd = w_sign * (2.0 * v_fast / track)
            serial_cmd = "a" if w_sign > 0 else "d"
            label = "SPINNING LEFT [A]" if w_sign > 0 else "SPINNING RIGHT [D]"
        else:
            # Stopped
            v_cmd = 0.0
            w_cmd = 0.0
            serial_cmd = "x"
            label = "READY / STOPPED [X]"

        self.current_v_cmd = v_cmd
        self.current_w_cmd = w_cmd
        self.send_command(serial_cmd)
        self.sidebar.set_motion_label(label)
        self.sidebar.highlight_dpad(v_sign, w_sign)

        # Update HUD metric cards directly when disconnected
        if self.connection is None or not self.connection.is_open:
            self.footer.update_velocities(v_cmd, math.degrees(w_cmd))

    def _kinematic_tick(self) -> None:
        """Live kinematic simulation tick when testing without serial connection."""
        if self.connection is not None and self.connection.is_open:
            return
        if abs(self.current_v_cmd) > 1e-3 or abs(self.current_w_cmd) > 1e-3:
            moved = self.odometry.update_from_velocities(
                self.current_v_cmd, self.current_w_cmd, dt=0.033
            )
            if moved:
                self.footer.update_pose(
                    self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
                )
                self.canvas.update()

    # D-Pad Button Handlers
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

    # Discrete Action Triggers
    def _action_cell_forward(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()
        self.send_command("f")
        self.sidebar.set_motion_label("1 CELL FORWARD [F]")
        if self.connection is None or not self.connection.is_open:
            d = self.sidebar.get_cell_distance_mm()
            dx = -d * math.sin(self.odometry.heading_rad)
            dy = d * math.cos(self.odometry.heading_rad)
            self.odometry.displace(dx, dy)
            self.footer.update_pose(
                self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
            )
            self.canvas.update()

    def _action_cell_backward(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()
        self.send_command("b")
        self.sidebar.set_motion_label("1 CELL BACKWARD [B]")
        if self.connection is None or not self.connection.is_open:
            d = self.sidebar.get_cell_distance_mm()
            dx = -d * math.sin(self.odometry.heading_rad)
            dy = d * math.cos(self.odometry.heading_rad)
            self.odometry.displace(-dx, -dy)
            self.footer.update_pose(
                self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
            )
            self.canvas.update()

    def _action_turn_90_left(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()
        self.send_command("j")
        self.sidebar.set_motion_label("TURN 90° LEFT [J]")
        if self.connection is None or not self.connection.is_open:
            self.odometry.rotate(math.pi / 2.0)
            self.footer.update_pose(
                self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
            )
            self.canvas.update()

    def _action_turn_90_right(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()
        self.send_command("k")
        self.sidebar.set_motion_label("TURN 90° RIGHT [K]")
        if self.connection is None or not self.connection.is_open:
            self.odometry.rotate(-math.pi / 2.0)
            self.footer.update_pose(
                self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
            )
            self.canvas.update()

    def _action_turn_180(self) -> None:
        self.held_movement_keys.clear()
        self._process_movement_keys()
        self.send_command("u")
        self.sidebar.set_motion_label("TURN 180° [U]")
        if self.connection is None or not self.connection.is_open:
            self.odometry.rotate(math.pi)
            self.footer.update_pose(
                self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
            )
            self.canvas.update()

    def reset_pose_and_encoders(self) -> None:
        self.held_movement_keys.clear()
        self.current_v_cmd = 0.0
        self.current_w_cmd = 0.0
        self.sidebar.highlight_dpad(0, 0)
        self.send_command("x")
        self.canvas.reset_discovered()
        self.canvas.mark_cell_discovered(0, 0)
        if self.canvas.maze is not None:
            cell_size = self.sidebar.get_cell_distance_mm()
            self.odometry.reset(cell_size * 0.5, cell_size * 0.5, 0.0)
        else:
            self.odometry.reset(0.0, 0.0, 0.0)
        self.reset_stage = "stopping"
        self.reset_deadline = time.monotonic() + 3.0
        self.sidebar.set_motion_label("RESETTING...")
        self.sidebar.set_note("Stopping, then resetting encoder totals...")
        QTimer.singleShot(180, self._send_encoder_reset)
        self.canvas.update()
        self.footer.update_pose(
            self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
        )

    def _on_load_maze(self, preset_name: str = "6x6 Test Maze") -> None:
        """Loads a maze layout from the FloodFill-Micromouse submodule and places robot at (0,0)."""
        try:
            from gui import maze_loader

            maze = maze_loader.load_preset(preset_name)
            self.canvas.set_maze(maze)
            self.nav_engine.set_maze(maze)
            self.sidebar.set_maze_status(f"Active: {preset_name} ({maze.size}x{maze.size})")
            self.place_robot_at_cell_0_0()
        except Exception as exc:
            self.sidebar.set_maze_status(f"Error: {exc}")
            QMessageBox.warning(self, "Failed to Load Maze", str(exc))

    def _on_clear_maze(self) -> None:
        """Clears the active maze from the canvas."""
        self.canvas.set_maze(None)
        self.nav_engine.set_maze(None)
        self.sidebar.set_maze_status("Maze: None")
        self.canvas.update()

    def place_robot_at_cell_0_0(self) -> None:
        """Positions the robot at the center of cell (0, 0) facing North (0.0 rad)."""
        cell_size = self.sidebar.get_cell_distance_mm()
        x0 = cell_size * 0.5
        y0 = cell_size * 0.5
        self.odometry.reset(x0, y0, heading_rad=0.0)
        self.nav_engine.reset()
        self.footer.update_pose(x0, y0, 0.0)
        self.sidebar.set_motion_label("PLACED AT CELL (0,0)")
        self.canvas.reset_discovered()
        self.canvas.mark_cell_discovered(0, 0)
        self.canvas.update()

    def _on_solve_maze_clicked(self) -> None:
        """Triggers embedded blind flood-fill solver on STM32."""
        if self.connection is None or not self.connection.is_open:
            QMessageBox.information(
                self,
                "Not Connected",
                "Please connect to the STM32 Blue Pill on COM port (e.g. COM5) before solving.",
            )
            return

        if self.canvas.maze is None:
            self._on_load_maze("6x6 Test Maze")

        self.place_robot_at_cell_0_0()
        drive_motors = self.sidebar.chk_drive_motors.isChecked()
        mode_str = "DRIVING MOTORS" if drive_motors else "SIMULATION"
        self.sidebar.set_motion_label(f"BLIND SOLVE [{mode_str}]...")
        self.sidebar.set_maze_status(f"Blind cell-by-cell navigation ({mode_str.lower()})...")

        # Run solve in a quick background thread to avoid blocking GUI
        threading.Thread(target=self._upload_and_solve_worker, args=(drive_motors,), daemon=True).start()

    def _upload_and_solve_worker(self, drive_motors: bool = False) -> None:
        try:
            maze = self.canvas.maze
            if maze is None or self.connection is None:
                return

            cell_size = self.sidebar.get_cell_distance_mm()
            x0 = cell_size * 0.5
            y0 = cell_size * 0.5

            # Reset odometry state and baseline at cell (0, 0)
            self.odometry.reset(x0, y0, heading_rad=0.0)

            # If driving motors, reset firmware encoder counter so hardware & software start at 0
            if drive_motors:
                with self.write_lock:
                    if self.connection and self.connection.is_open:
                        self.connection.write(b"r\r\n")
                time.sleep(0.08)

            # Unified Engine generates the blind solve command
            cmd = self.nav_engine.start_solve(maze, with_motors=drive_motors)
            with self.write_lock:
                if self.connection and self.connection.is_open:
                    self.connection.write(cmd.encode("ascii"))

        except Exception as exc:
            print("Solve worker error:", exc)

    def _send_encoder_reset(self) -> None:
        if self.connection is None:
            self.reset_stage = None
            return
        if self.reset_stage == "stopping":
            if self.send_command("r"):
                self.reset_stage = "baseline"
                self.sidebar.set_note("Waiting for reset counts from firmware...")
            else:
                self.reset_stage = None

    # ------------------------------------------------------------------
    # Unified Navigation Engine Event Callbacks
    # ------------------------------------------------------------------
    def _on_engine_wall_request(
        self, rx: int, ry: int, r_dir: str, f: int, l: int, r: int
    ) -> None:
        self.canvas.mark_cell_discovered(rx, ry)
        self.sidebar.set_note(f"Oracle: Supplied walls for ({rx},{ry}) {r_dir} -> F:{f} L:{l} R:{r}")

    def _on_engine_cell_arrival(self, rx: int, ry: int, r_dir: str) -> None:
        self.canvas.mark_cell_discovered(rx, ry)

        cardinal_angles = {"N": 0.0, "E": -math.pi / 2.0, "S": math.pi, "W": math.pi / 2.0}
        target_heading = cardinal_angles.get(r_dir.upper(), 0.0)

        if not self.nav_engine.with_motors:
            # Simulation mode: no wheel encoder counts occur, so advance kinematic pose
            # cell-by-cell along the path so the canvas draws the true maze path.
            cell_size = self.sidebar.get_cell_distance_mm()
            target_x = (rx + 0.5) * cell_size
            target_y = (ry + 0.5) * cell_size
            self.odometry.set_pose(target_x, target_y, target_heading, keep_path=True)
            self.odometry.imu.reset(target_heading)
        else:
            # Live motors mode: wheel encoders continuously track physical movement.
            # On cell arrival, align simulated IMU reference to the cardinal heading
            # and fuse it through EKF to correct yaw drift without artificial position snapping.
            self.odometry.imu.set_cardinal_reference(r_dir)
            imu_yaw = self.odometry.imu.get_yaw_measurement()
            self.odometry.update_imu_yaw(imu_yaw)

        # Update HUD cards and UI labels
        self.footer.update_pose(
            self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
        )
        self.sidebar.set_motion_label(f"ARRIVED AT ({rx}, {ry}) [{r_dir}]")
        self.sidebar.set_note(
            f"Pose: ({self.odometry.x_mm:.1f}, {self.odometry.y_mm:.1f}) Yaw: {math.degrees(self.odometry.heading_rad):.1f}° [EKF Fused]"
        )
        self.canvas.update()

    def _on_engine_step(self, step_dict: dict[str, Any]) -> None:
        self.sidebar.set_motion_label(
            f"STEP {step_dict['step_num']}: {step_dict['action']} -> ({step_dict['col']}, {step_dict['row']}) [{step_dict['dir']}]"
        )

    def _on_engine_motor_action(self, line: str) -> None:
        self.sidebar.set_note(line)

    def _on_engine_goal(self, coords: tuple[int, int]) -> None:
        self.sidebar.set_motion_label("GOAL REACHED! [SUCCESS]")
        self.sidebar.set_maze_status("Goal reached successfully!")
        self.canvas.update()

    def _on_engine_status(self, success: bool, msg: str) -> None:
        status_text = "SOLVE SUCCESS" if success else "SOLVE FAILED"
        self.sidebar.set_motion_label(status_text)
        self.sidebar.set_maze_status(f"Maze solve status: {msg}")
        self.canvas.update()

    def _on_engine_telemetry(self, parsed: tuple[int, ...]) -> None:
        self.telemetry = parsed
        (
            left_count,
            _l_delta,
            left_rpm,
            _left_pwm,
            right_count,
            _r_delta,
            right_rpm,
            _right_pwm,
        ) = parsed

        self.footer.update_hardware(left_count, right_count, left_rpm, right_rpm)

        if self.reset_stage == "stopping":
            return

        if self.reset_stage == "baseline":
            if abs(left_count) <= 120 and abs(right_count) <= 120:
                self.odometry.set_baseline(left_count, right_count)
                self.reset_stage = None
                self.sidebar.set_note("Pose and firmware encoder totals reset.")
                self.sidebar.set_motion_label("READY / STOPPED [X]")
                self.footer.update_pose(
                    self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
                )
                self.canvas.update()
            elif time.monotonic() >= self.reset_deadline:
                self.odometry.set_baseline(left_count, right_count)
                self.reset_stage = None
                self.sidebar.set_note("Reset timed out; baseline restarted.")
                self.sidebar.set_motion_label("READY / STOPPED [X]")
                self.footer.update_pose(
                    self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
                )
                self.canvas.update()
            return

        moved = self.odometry.update_counts(left_count, right_count)
        self.footer.update_pose(
            self.odometry.x_mm, self.odometry.y_mm, self.odometry.heading_rad
        )

        circumference = self.odometry.wheel_circumference_mm
        track = self.odometry.track_width_mm
        v_l = (left_rpm / 60.0) * circumference
        v_r = (right_rpm / 60.0) * circumference
        v_linear = (v_r + v_l) * 0.5
        w_rad = (v_r - v_l) / track

        self.footer.update_velocities(v_linear, math.degrees(w_rad))

        if moved:
            if self.canvas.maze is not None:
                cell_size = self.sidebar.get_cell_distance_mm()
                cx = int(math.floor(self.odometry.x_mm / cell_size))
                cy = int(math.floor(self.odometry.y_mm / cell_size))
                if 0 <= cx < self.canvas.maze.size and 0 <= cy < self.canvas.maze.size:
                    self.canvas.mark_cell_discovered(cx, cy)
            self.canvas.update()

    # Telemetry Line Receiver
    def _on_serial_error(self, message: str) -> None:
        self.disconnect_serial(send_stop=False)
        self.sidebar.set_note(f"Serial Error: {message}")

    def _on_telemetry_line(self, line: str) -> None:
        """Processes line through the unified NavigationEngine and dispatches responses."""
        response = self.nav_engine.process_line(line)
        if response is not None:
            with self.write_lock:
                if self.connection and self.connection.is_open:
                    self.connection.write(response.encode("ascii"))

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.disconnect_serial()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    window = OdometryApp()
    window.show()
    sys.exit(app.exec())

"""Differential drive forward kinematics and dead-reckoning pose tracking."""

from __future__ import annotations

import math
import time


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
        """Sets the baseline encoder readings without updating position."""
        self.last_counts = (left_count, right_count)
        self.last_time = time.monotonic()

    def reset(self, x_mm: float = 0.0, y_mm: float = 0.0, heading_rad: float = 0.0) -> None:
        """Resets the robot pose back to (x_mm, y_mm, heading_rad) and clears path history."""
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.heading_rad = heading_rad
        self.linear_speed_mm_s = 0.0
        self.angular_speed_rad_s = 0.0
        self.path = [(x_mm, y_mm)]
        self.last_counts = None
        self.last_time = None

    def set_pose(self, x_mm: float, y_mm: float, heading_rad: float = 0.0) -> None:
        """Sets the current pose and starts a new path."""
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.heading_rad = heading_rad
        self.linear_speed_mm_s = 0.0
        self.angular_speed_rad_s = 0.0
        self.path = [(x_mm, y_mm)]

    def update_counts(
        self, left_count: int, right_count: int, timestamp: float | None = None
    ) -> bool:
        """Updates robot pose from raw hardware encoder counters."""
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

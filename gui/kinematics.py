"""Differential drive forward kinematics, Extended Kalman Filter (EKF), and simulated IMU."""

from __future__ import annotations

import math
import random
import time


def normalize_angle(angle_rad: float) -> float:
    """Normalizes angle to the range (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class SimulatedIMU:
    """
    Simulates a 6-DOF/9-DOF inertial measurement unit (gyroscope / magnetometer).
    Provides yaw (heading) referenced to the start point as 0.0 radians (0, 0, 0).
    Allows simulating physical sensor noise now, and cleanly swapping in hardware IMU telemetry later.
    """

    def __init__(self, initial_yaw_rad: float = 0.0, noise_std_rad: float = 0.003) -> None:
        self.current_yaw_rad: float = normalize_angle(initial_yaw_rad)
        self.noise_std_rad: float = noise_std_rad
        self._target_cardinal_yaw: float = self.current_yaw_rad

    def reset(self, yaw_rad: float = 0.0) -> None:
        """Resets the simulated IMU to the start reference (0.0 rad = North)."""
        self.current_yaw_rad = normalize_angle(yaw_rad)
        self._target_cardinal_yaw = self.current_yaw_rad

    def set_cardinal_reference(self, cardinal_dir: str) -> None:
        """
        Aligns target heading from cardinal direction:
        N = 0.0 rad (0°), E = -pi/2 (-90°), S = pi (180°), W = +pi/2 (+90°).
        """
        cardinal_map = {
            "N": 0.0,
            "E": -math.pi / 2.0,
            "S": math.pi,
            "W": math.pi / 2.0,
        }
        target = cardinal_map.get(cardinal_dir.upper(), self.current_yaw_rad)
        self._target_cardinal_yaw = target
        self.current_yaw_rad = target

    def update_from_turning(self, delta_theta_rad: float) -> None:
        """Updates continuous yaw during turning motions."""
        self.current_yaw_rad = normalize_angle(self.current_yaw_rad + delta_theta_rad)

    def get_yaw_measurement(self, add_noise: bool = True) -> float:
        """
        Returns simulated IMU yaw measurement referenced to the start point (0, 0, 0).
        Includes Gaussian gyro sensor noise if add_noise is True.
        """
        noise = random.gauss(0.0, self.noise_std_rad) if add_noise and self.noise_std_rad > 0 else 0.0
        return normalize_angle(self.current_yaw_rad + noise)


class RobotEKF:
    """
    Extended Kalman Filter (EKF) for 2D differential-drive robot pose estimation.
    State vector: x = [x (mm), y (mm), theta (rad)]^T relative to start point (0, 0, 0).
    Fuses wheel encoder odometry (prediction) with IMU yaw measurements (correction).
    """

    def __init__(
        self,
        x_init_mm: float = 0.0,
        y_init_mm: float = 0.0,
        theta_init_rad: float = 0.0,
        sigma_pos_init: float = 0.5,
        sigma_theta_init: float = 0.005,
    ) -> None:
        self.x = x_init_mm
        self.y = y_init_mm
        self.theta = normalize_angle(theta_init_rad)

        # 3x3 State Covariance Matrix P
        self.P = [
            [sigma_pos_init ** 2, 0.0, 0.0],
            [0.0, sigma_pos_init ** 2, 0.0],
            [0.0, 0.0, sigma_theta_init ** 2],
        ]

        # Base process noise parameters
        self.k_pos = 0.02          # Process noise scaling with linear distance
        self.k_rot = 0.01          # Process noise scaling with angular rotation
        self.base_q_pos = 0.05     # mm^2
        self.base_q_theta = 0.0001 # rad^2

        # Default measurement noise for IMU yaw (~0.17 deg std)
        self.default_r_yaw = 0.003 ** 2

    def reset(
        self,
        x_mm: float = 0.0,
        y_mm: float = 0.0,
        theta_rad: float = 0.0,
        sigma_pos: float = 0.5,
        sigma_theta: float = 0.005,
    ) -> None:
        """Resets the EKF state and covariance back to initial conditions."""
        self.x = x_mm
        self.y = y_mm
        self.theta = normalize_angle(theta_rad)
        self.P = [
            [sigma_pos ** 2, 0.0, 0.0],
            [0.0, sigma_pos ** 2, 0.0],
            [0.0, 0.0, sigma_theta ** 2],
        ]

    def predict(
        self,
        delta_left_mm: float,
        delta_right_mm: float,
        track_width_mm: float,
    ) -> None:
        """
        EKF Prediction Step:
        Propagates state and covariance using differential-drive kinematics.
        """
        delta_s = (delta_right_mm + delta_left_mm) * 0.5
        delta_theta = (delta_right_mm - delta_left_mm) / track_width_mm

        # Mid-step heading for circular arc integration
        mid_theta = self.theta + delta_theta * 0.5

        # 1. Non-linear state propagation
        self.x += -delta_s * math.sin(mid_theta)
        self.y += delta_s * math.cos(mid_theta)
        self.theta = normalize_angle(self.theta + delta_theta)

        # 2. Jacobian of motion model w.r.t state F = df/dx
        a = -delta_s * math.cos(mid_theta)
        b = -delta_s * math.sin(mid_theta)

        # 3. Process noise covariance Q
        q11 = self.base_q_pos + self.k_pos * abs(delta_s)
        q22 = self.base_q_pos + self.k_pos * abs(delta_s)
        q33 = self.base_q_theta + self.k_rot * abs(delta_theta)

        # 4. Covariance propagation P_new = F * P * F^T + Q
        p = self.P
        p11, p12, p13 = p[0][0], p[0][1], p[0][2]
        p21, p22, p23 = p[1][0], p[1][1], p[1][2]
        p31, p32, p33 = p[2][0], p[2][1], p[2][2]

        new_p11 = p11 + 2.0 * a * p13 + (a ** 2) * p33 + q11
        new_p12 = p12 + a * p23 + b * p13 + a * b * p33
        new_p13 = p13 + a * p33
        new_p22 = p22 + 2.0 * b * p23 + (b ** 2) * p33 + q22
        new_p23 = p23 + b * p33
        new_p33 = p33 + q33

        self.P = [
            [new_p11, new_p12, new_p13],
            [new_p12, new_p22, new_p23],
            [new_p13, new_p23, new_p33],
        ]

    def update_yaw(self, yaw_measurement_rad: float, r_yaw: float | None = None) -> float:
        """
        EKF Measurement Update Step:
        Fuses IMU yaw observation z = [theta_imu].
        Measurement model: z = H * x where H = [0, 0, 1].
        Returns innovation (measurement residual).
        """
        if r_yaw is None:
            r_yaw = self.default_r_yaw

        # 1. Innovation (measurement residual with angle wrapping)
        innovation = normalize_angle(yaw_measurement_rad - self.theta)

        # 2. Innovation covariance: S = H * P * H^T + R = P33 + R
        s = self.P[2][2] + r_yaw
        if s <= 1e-9:
            return innovation

        # 3. Kalman Gain for heading: K_theta = P33 / S
        # Decouple heading from position to avoid unobserved lateral jumps
        k3 = self.P[2][2] / s

        # 4. State correction (heading only)
        self.theta = normalize_angle(self.theta + k3 * innovation)

        # 5. Covariance update (Joseph stabilized form for heading)
        p = self.P
        new_p33 = max(1e-6, p[2][2] * (1.0 - k3))
        new_p13 = p[0][2] * (1.0 - k3)
        new_p23 = p[1][2] * (1.0 - k3)

        self.P = [
            [p[0][0], p[0][1], new_p13],
            [p[1][0], p[1][1], new_p23],
            [new_p13, new_p23, new_p33],
        ]

        return innovation


class DifferentialDriveOdometry:
    """
    Differential-drive robot pose tracker powered by an Extended Kalman Filter (EKF).
    Fuses wheel encoder counts (prediction) with Simulated IMU yaw (correction),
    referenced to the start point as (0, 0, 0).
    """

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

        # Start point origin offset in canvas coordinates (e.g. center of start cell (0, 0))
        self.origin_x_mm: float = 0.0
        self.origin_y_mm: float = 0.0

        # EKF state estimator and simulated IMU (start point referenced as 0, 0, 0)
        self.ekf = RobotEKF(0.0, 0.0, 0.0)
        self.imu = SimulatedIMU(0.0)

        # Kinematic velocities
        self.linear_speed_mm_s = 0.0
        self.angular_speed_rad_s = 0.0

        self.path: list[tuple[float, float]] = [(0.0, 0.0)]
        self.last_counts: tuple[int, int] | None = None
        self.last_time: float | None = None

    @property
    def x_mm(self) -> float:
        """Global X position in canvas coordinates."""
        return self.origin_x_mm + self.ekf.x

    @x_mm.setter
    def x_mm(self, val: float) -> None:
        self.ekf.x = val - self.origin_x_mm

    @property
    def y_mm(self) -> float:
        """Global Y position in canvas coordinates."""
        return self.origin_y_mm + self.ekf.y

    @y_mm.setter
    def y_mm(self, val: float) -> None:
        self.ekf.y = val - self.origin_y_mm

    @property
    def heading_rad(self) -> float:
        """Estimated heading angle in radians (North = 0.0 rad)."""
        return self.ekf.theta

    @heading_rad.setter
    def heading_rad(self, val: float) -> None:
        self.ekf.theta = normalize_angle(val)
        self.imu.reset(self.ekf.theta)

    @property
    def relative_x_mm(self) -> float:
        """X position in mm referenced to start point as (0, 0, 0)."""
        return self.ekf.x

    @property
    def relative_y_mm(self) -> float:
        """Y position in mm referenced to start point as (0, 0, 0)."""
        return self.ekf.y

    def displace(self, dx_mm: float, dy_mm: float) -> None:
        """Displaces robot position by (dx_mm, dy_mm) and appends to path."""
        self.ekf.x += dx_mm
        self.ekf.y += dy_mm
        self.path.append((self.x_mm, self.y_mm))

    def rotate(self, delta_rad: float) -> None:
        """Rotates robot heading by delta_rad."""
        new_heading = normalize_angle(self.ekf.theta + delta_rad)
        self.ekf.theta = new_heading
        self.imu.reset(new_heading)

    def set_baseline(self, left_count: int, right_count: int) -> None:
        """Sets the baseline encoder readings without updating position."""
        self.last_counts = (left_count, right_count)
        self.last_time = time.monotonic()

    def reset(self, x_mm: float = 0.0, y_mm: float = 0.0, heading_rad: float = 0.0) -> None:
        """
        Resets the robot pose. Sets start point origin to (x_mm, y_mm),
        resets EKF to (0, 0, 0) relative to start point, and resets IMU to heading_rad.
        """
        self.origin_x_mm = x_mm
        self.origin_y_mm = y_mm
        self.ekf.reset(0.0, 0.0, heading_rad)
        self.imu.reset(heading_rad)
        self.linear_speed_mm_s = 0.0
        self.angular_speed_rad_s = 0.0
        self.path = [(self.x_mm, self.y_mm)]
        self.last_counts = None
        self.last_time = None

    def set_pose(
        self,
        x_mm: float,
        y_mm: float,
        heading_rad: float = 0.0,
        keep_path: bool = False,
    ) -> None:
        """Sets the current pose (re-calibrates origin or advances kinematic pose)."""
        if keep_path:
            self.ekf.x = x_mm - self.origin_x_mm
            self.ekf.y = y_mm - self.origin_y_mm
            self.ekf.theta = normalize_angle(heading_rad)
            self.imu.reset(heading_rad)
            self.linear_speed_mm_s = 0.0
            self.angular_speed_rad_s = 0.0
            if not self.path or abs(self.path[-1][0] - x_mm) > 0.5 or abs(self.path[-1][1] - y_mm) > 0.5:
                self.path.append((x_mm, y_mm))
        else:
            self.origin_x_mm = x_mm
            self.origin_y_mm = y_mm
            self.ekf.reset(0.0, 0.0, heading_rad)
            self.imu.reset(heading_rad)
            self.linear_speed_mm_s = 0.0
            self.angular_speed_rad_s = 0.0
            self.path = [(x_mm, y_mm)]

    def update_imu_yaw(self, yaw_measurement_rad: float) -> None:
        """
        Directly updates EKF state with an external IMU yaw observation
        (e.g. from Python oracle or future hardware IMU).
        """
        self.ekf.update_yaw(yaw_measurement_rad)
        if not self.path or abs(self.path[-1][0] - self.x_mm) > 0.5 or abs(self.path[-1][1] - self.y_mm) > 0.5:
            self.path.append((self.x_mm, self.y_mm))

    def update_counts(
        self,
        left_count: int,
        right_count: int,
        timestamp: float | None = None,
        imu_yaw: float | None = None,
    ) -> bool:
        """
        Updates robot pose using the EKF:
        1. Prediction step from raw hardware encoder counters.
        2. Measurement update step from IMU yaw.
        """
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

        # Reject corrupt or unaligned single-frame encoder deltas (> 1500 counts in 100 ms)
        if abs(left_delta) > 1500 or abs(right_delta) > 1500:
            return False

        dt = (timestamp - self.last_time) if self.last_time else 0.1
        self.last_time = timestamp
        dt = max(1e-4, dt)

        # 1. Wheel travel distances in mm
        delta_left = (left_delta / self.left_cpr) * self.wheel_circumference_mm
        delta_right = (right_delta / self.right_cpr) * self.wheel_circumference_mm

        # 2. Kinematic velocities
        delta_s = (delta_right + delta_left) * 0.5
        delta_theta = (delta_right - delta_left) / self.track_width_mm
        self.linear_speed_mm_s = delta_s / dt
        self.angular_speed_rad_s = delta_theta / dt

        if abs(left_delta) == 0 and abs(right_delta) == 0:
            return False

        # 3. EKF Prediction Step (motion model from wheel odometry)
        self.ekf.predict(delta_left, delta_right, self.track_width_mm)

        # 4. Simulated IMU yaw measurement & EKF Update Step
        if imu_yaw is None:
            self.imu.update_from_turning(delta_theta)
            imu_yaw = self.imu.get_yaw_measurement()

        self.ekf.update_yaw(imu_yaw)

        # 5. Append estimated pose to trajectory path
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

        # Equivalent left and right wheel travel
        delta_left = delta_s - (omega_rad_s * self.track_width_mm * dt * 0.5)
        delta_right = delta_s + (omega_rad_s * self.track_width_mm * dt * 0.5)

        self.ekf.predict(delta_left, delta_right, self.track_width_mm)
        self.imu.update_from_turning(delta_theta)
        imu_yaw = self.imu.get_yaw_measurement()
        self.ekf.update_yaw(imu_yaw)

        self.path.append((self.x_mm, self.y_mm))
        return True


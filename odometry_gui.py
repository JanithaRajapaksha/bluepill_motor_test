"""Live differential-drive odometry viewer for the Blue Pill micromouse (PyQt6).

Modularized architecture:
- gui.config: Themes, color palettes, default geometry constants, regex patterns
- gui.kinematics: Pure mathematical DifferentialDriveOdometry model
- gui.serial_worker: Background QThread for serial telemetry parsing
- gui.canvas: High-performance QPainter 2D viewport
- gui.sidebar: Scrollable sidebar with serial, calibration, WASD, and discrete controls
- gui.footer: Live telemetry metrics cards dashboard
- gui.main_window: Main application window coordinating all components
"""

from gui.main_window import OdometryApp, main

if __name__ == "__main__":
    main()

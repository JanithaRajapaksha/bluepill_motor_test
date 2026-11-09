"""Bottom metrics HUD footer displaying live robot kinematics and hardware state."""

from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class FooterWidget(QFrame):
    """Bottom telemetry dashboard card container."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("footer")
        self.setFixedHeight(72)
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(10)

        self.pos_val = self._create_card(layout, "POSITION / mm", "X 0.0   Y 0.0")
        self.heading_val = self._create_card(layout, "HEADING", "0.0° [N]")
        self.v_val = self._create_card(layout, "LINEAR SPEED (v)", "0.0 mm/s")
        self.w_val = self._create_card(layout, "ANGULAR SPEED (ω)", "0.0 °/s")
        self.encoders_val = self._create_card(layout, "ENCODERS (L / R)", "0 / 0")
        self.rpm_val = self._create_card(layout, "MOTOR RPM (L / R)", "0 / 0")

    def _create_card(self, parent_layout: QHBoxLayout, label: str, init: str) -> QLabel:
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

    def update_pose(self, x_mm: float, y_mm: float, heading_rad: float) -> None:
        """Updates displayed position and heading."""
        self.pos_val.setText(f"X {x_mm:.1f}   Y {y_mm:.1f}")

        deg = math.degrees(heading_rad)
        if abs(deg) < 0.5:
            head_str = "0.0° [N]"
        elif deg > 0:
            head_str = f"L {deg:.1f}°"
        else:
            head_str = f"R {abs(deg):.1f}°"
        self.heading_val.setText(head_str)

    def update_velocities(self, v_mm_s: float, omega_deg_s: float) -> None:
        """Updates linear and angular speed readouts."""
        self.v_val.setText(f"{v_mm_s:.1f} mm/s")
        self.w_val.setText(f"{omega_deg_s:.1f} °/s")

    def update_hardware(
        self, left_count: int, right_count: int, left_rpm: int, right_rpm: int
    ) -> None:
        """Updates raw hardware encoder and RPM readings."""
        self.encoders_val.setText(f"{left_count} / {right_count}")
        self.rpm_val.setText(f"{left_rpm} / {right_rpm} RPM")

"""Core data models for the UAV TUI dashboard."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Tuple

Vector3 = Tuple[float, float, float]


@dataclass(slots=True)
class UAVStatus:
    """Represents the current UAV telemetry snapshot consumed by the UI layer."""

    position: Vector3 = (0.0, 0.0, 0.0)
    orientation: Vector3 = (0.0, 0.0, 0.0)
    battery_level: float = 100.0
    voltage: float = 12.5

    @classmethod
    def empty(cls) -> "UAVStatus":
        """Return a default-initialised status instance."""
        return cls()

    def with_position(self, position: Vector3) -> "UAVStatus":
        """Return a copy with an updated position."""
        return replace(self, position=position)

    def with_orientation(self, orientation: Vector3) -> "UAVStatus":
        """Return a copy with an updated orientation."""
        return replace(self, orientation=orientation)

    def with_battery(self, battery_level: float, voltage: float) -> "UAVStatus":
        """Return a copy with updated battery metrics."""
        return replace(self, battery_level=battery_level, voltage=voltage)


__all__ = ["UAVStatus", "Vector3"]

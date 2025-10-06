"""Core data models for the UAV TUI dashboard."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from datetime import datetime
from typing import Optional, Tuple

Vector3 = Tuple[float, float, float]


@dataclass(slots=True)
class VehicleState:
    """Represents high-level vehicle health and mode information."""

    valid: bool = False
    timestamp: Optional[datetime] = None
    arming_state: Optional[int] = None
    nav_state: Optional[int] = None
    failsafe: bool = False
    pre_flight_checks_pass: Optional[bool] = None


@dataclass(slots=True)
class UAVStatus:
    """Represents the current UAV telemetry snapshot consumed by the UI layer."""

    position: Vector3 = (0.0, 0.0, 0.0)
    orientation: Vector3 = (0.0, 0.0, 0.0)
    battery_level: float = 100.0
    voltage: float = 12.5
    vehicle_state: VehicleState = field(default_factory=VehicleState)

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

    def with_vehicle_state(self, vehicle_state: VehicleState) -> "UAVStatus":
        """Return a copy with updated vehicle health/state data."""
        return replace(self, vehicle_state=vehicle_state)


__all__ = ["UAVStatus", "Vector3", "VehicleState"]

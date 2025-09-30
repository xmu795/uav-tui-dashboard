"""Data source abstractions for the UAV TUI dashboard."""

from __future__ import annotations

import abc
import asyncio
import logging
import random
from typing import Optional

from .models import UAVStatus, Vector3

logger = logging.getLogger(__name__)


class DataSource(abc.ABC):
    """Provides UAV telemetry snapshots for the UI layer."""

    @abc.abstractmethod
    async def start(self) -> None:
        """Initialise resources before polling begins."""

    @abc.abstractmethod
    async def stop(self) -> None:
        """Release any resources when polling stops."""

    @abc.abstractmethod
    async def fetch(self) -> UAVStatus:
        """Return the latest UAV status."""


class SimDataSource(DataSource):
    """Generates pseudo-random telemetry data for demonstration purposes."""

    def __init__(
        self,
        *,
        position_delta: float = 0.1,
        altitude_delta: float = 0.05,
        orientation_delta: float = 0.01,
        battery_drain: float = 0.5,
        min_voltage: float = 10.0,
        voltage_delta: float = 0.1,
        seed: Optional[int] = None,
    ) -> None:
        self._status = UAVStatus.empty()
        self._position_delta = position_delta
        self._altitude_delta = altitude_delta
        self._orientation_delta = orientation_delta
        self._battery_drain = battery_drain
        self._min_voltage = min_voltage
        self._voltage_delta = voltage_delta
        self._rng = random.Random(seed)
        self._lock = asyncio.Lock()
        self._running = False

    async def start(self) -> None:
        logger.debug("Starting SimDataSource")
        self._running = True

    async def stop(self) -> None:
        logger.debug("Stopping SimDataSource")
        self._running = False

    async def fetch(self) -> UAVStatus:
        if not self._running:
            logger.debug("Fetch requested while data source stopped; starting lazily")
            await self.start()

        async with self._lock:
            pos = self._status.position
            ori = self._status.orientation
            new_position: Vector3 = (
                pos[0] + self._rng.uniform(-self._position_delta, self._position_delta),
                pos[1] + self._rng.uniform(-self._position_delta, self._position_delta),
                pos[2] + self._rng.uniform(-self._altitude_delta, self._altitude_delta),
            )
            new_orientation: Vector3 = (
                ori[0] + self._rng.uniform(-self._orientation_delta, self._orientation_delta),
                ori[1] + self._rng.uniform(-self._orientation_delta, self._orientation_delta),
                ori[2] + self._rng.uniform(-self._orientation_delta, self._orientation_delta),
            )
            drained_battery = max(0.0, self._status.battery_level - self._rng.uniform(0, self._battery_drain))
            voltage = max(self._min_voltage, self._status.voltage + self._rng.uniform(-self._voltage_delta, self._voltage_delta))

            self._status = (
                self._status
                .with_position(new_position)
                .with_orientation(new_orientation)
                .with_battery(drained_battery, voltage)
            )

            logger.debug("Simulated status: %s", self._status)
            return self._status


class Ros2DataSource(DataSource):
    """Placeholder for ROS2 integration."""

    def __init__(self, *_, **__) -> None:  # pragma: no cover - stub implementation
        raise NotImplementedError("ROS2 data source is not yet implemented")

    async def start(self) -> None:  # pragma: no cover - stub implementation
        raise NotImplementedError

    async def stop(self) -> None:  # pragma: no cover - stub implementation
        raise NotImplementedError

    async def fetch(self) -> UAVStatus:  # pragma: no cover - stub implementation
        raise NotImplementedError


__all__ = ["DataSource", "SimDataSource", "Ros2DataSource"]

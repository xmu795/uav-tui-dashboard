"""Textual UI application for the UAV dashboard."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from textual.app import App, ComposeResult
from textual.timer import Timer
from textual.widgets import DataTable, Footer, Header

from ..core.datasource import DataSource
from ..core.models import UAVStatus

logger = logging.getLogger(__name__)


class UAVDashboardApp(App):
    """Textual application that renders UAV telemetry."""

    CSS_PATH = Path(__file__).with_name("dashboard.css")
    FIELD_COLUMN_KEY = "field"
    VALUE_COLUMN_KEY = "value"

    def __init__(self, data_source: DataSource, *, poll_interval: float = 1.0) -> None:
        super().__init__()
        self._data_source = data_source
        self._poll_interval = poll_interval
        self._status = UAVStatus.empty()
        self._status_timer: Optional[Timer] = None

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield DataTable(id="status_table")
        yield Footer()

    async def on_mount(self) -> None:
        await self._data_source.start()
        table = self.query_one("#status_table", DataTable)
        table.add_column("Field", key=self.FIELD_COLUMN_KEY)
        table.add_column("Value", key=self.VALUE_COLUMN_KEY)
        self._add_rows(table)
        self._status_timer = self.set_interval(self._poll_interval, self._tick, name="status_poll")

    async def on_unmount(self) -> None:
        if self._status_timer is not None:
            self._status_timer.stop()
        await self._data_source.stop()

    def _add_rows(self, table: DataTable) -> None:
        table.add_row("Position X", "0.00", key="position_x")
        table.add_row("Position Y", "0.00", key="position_y")
        table.add_row("Position Z", "0.00", key="position_z")
        table.add_row("Roll", "0.00", key="roll")
        table.add_row("Pitch", "0.00", key="pitch")
        table.add_row("Yaw", "0.00", key="yaw")
        table.add_row("Battery Level", "100.0%", key="battery_level")
        table.add_row("Voltage", "12.50V", key="voltage")

    async def _tick(self) -> None:
        try:
            status = await self._data_source.fetch()
        except Exception:  # pragma: no cover - defensive path
            logger.exception("Failed to fetch UAV status")
            return

        self._status = status
        self._update_table(status)

    def _update_table(self, status: UAVStatus) -> None:
        table = self.query_one("#status_table", DataTable)
        table.update_cell("position_x", self.VALUE_COLUMN_KEY, f"{status.position[0]:.2f}")
        table.update_cell("position_y", self.VALUE_COLUMN_KEY, f"{status.position[1]:.2f}")
        table.update_cell("position_z", self.VALUE_COLUMN_KEY, f"{status.position[2]:.2f}")
        table.update_cell("roll", self.VALUE_COLUMN_KEY, f"{status.orientation[0]:.2f}")
        table.update_cell("pitch", self.VALUE_COLUMN_KEY, f"{status.orientation[1]:.2f}")
        table.update_cell("yaw", self.VALUE_COLUMN_KEY, f"{status.orientation[2]:.2f}")
        table.update_cell("battery_level", self.VALUE_COLUMN_KEY, f"{status.battery_level:.1f}%")
        table.update_cell("voltage", self.VALUE_COLUMN_KEY, f"{status.voltage:.2f}V")


__all__ = ["UAVDashboardApp"]

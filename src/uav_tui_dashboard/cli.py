"""Command line interface wiring the data source and UI layers."""

from __future__ import annotations

import argparse
import logging
from typing import Optional, Sequence

from .core import DataSource, SimDataSource
from .ui import UAVDashboardApp


def _configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s - %(message)s",
    )


def _create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="UAV TUI dashboard")
    parser.add_argument(
        "--mode",
        choices=("sim", "ros2"),
        default="sim",
        help="Select the telemetry source. 'ros2' is a placeholder for future work.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=1.0,
        help="Polling interval (seconds) for fetching telemetry data.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Python logging level (e.g. INFO, DEBUG).",
    )
    return parser


def _make_data_source(mode: str) -> DataSource:
    if mode == "sim":
        return SimDataSource()
    if mode == "ros2":
        raise NotImplementedError("ROS2 mode is not yet implemented")
    raise ValueError(f"Unknown mode: {mode}")


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = _create_parser()
    args = parser.parse_args(argv)
    _configure_logging(args.log_level)
    data_source = _make_data_source(args.mode)
    app = UAVDashboardApp(data_source=data_source, poll_interval=args.poll_interval)
    app.run()


__all__ = ["main"]

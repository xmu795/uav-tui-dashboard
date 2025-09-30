"""Command line interface wiring the data source and UI layers."""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
from pathlib import Path
from typing import Any, Callable, Coroutine, Optional, Sequence

from .core import DataSource, SimDataSource
from .logging_config import LoggingSetupResult, configure_logging
from .shutdown import GracefulShutdown
from .ui import UAVDashboardApp

def _run_coroutine(factory: Callable[[], Coroutine[Any, Any, Any]]) -> None:
    try:
        asyncio.run(factory())
    except RuntimeError:
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(factory())
        finally:
            loop.close()


def _ensure_data_source_stopped(data_source: DataSource) -> None:
    async def _stop() -> None:
        await data_source.stop()

    _run_coroutine(_stop)


def _request_app_exit(app: UAVDashboardApp) -> None:
    try:
        if getattr(app, "is_running", False):
            app.call_from_thread(app.exit)
            return
    except RuntimeError:
        logger.debug("调用 call_from_thread 失败，尝试直接退出", exc_info=True)

    app.exit()

logger = logging.getLogger(__name__)

LOG_CONFIG_ENV_VAR = "UAV_TUI_LOG_CONFIG"
LOG_DIR_ENV_VAR = "UAV_TUI_LOG_DIR"


def _resolve_optional_path(value: Optional[Path | str]) -> Optional[Path]:
    if value is None or value == "":
        return None
    path = Path(value).expanduser()
    return path


def _configure_logging_from_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> LoggingSetupResult:
    config_path = _resolve_optional_path(args.log_config)
    if config_path is None:
        env_config = os.getenv(LOG_CONFIG_ENV_VAR)
        config_path = _resolve_optional_path(env_config)

    log_dir = _resolve_optional_path(args.log_dir)
    if log_dir is None:
        env_dir = os.getenv(LOG_DIR_ENV_VAR)
        log_dir = _resolve_optional_path(env_dir)

    try:
        result = configure_logging(
            level=args.log_level,
            console=not args.no_log_console,
            log_dir=log_dir,
            config_path=config_path,
        )
    except FileNotFoundError as exc:
        parser.error(f"指定的日志配置文件不存在: {exc}")
    except ValueError as exc:
        parser.error(f"日志配置文件解析失败: {exc}")
    except OSError as exc:  # pragma: no cover - defensive path
        parser.error(f"初始化日志系统失败: {exc}")

    logger.info("日志已初始化，输出目录：%s", result.directory)
    for name, file_path in result.handler_files.items():
        logger.debug("日志处理器 %s 写入文件 %s", name, file_path)

    return result


def _install_global_exception_hook() -> None:
    original_hook = sys.excepthook

    def _handle(exc_type: type[BaseException], exc_value: BaseException, exc_traceback) -> None:
        if issubclass(exc_type, KeyboardInterrupt):
            original_hook(exc_type, exc_value, exc_traceback)
            return
        logger.critical("未处理异常导致应用退出", exc_info=(exc_type, exc_value, exc_traceback))
        original_hook(exc_type, exc_value, exc_traceback)

    sys.excepthook = _handle


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
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="日志文件输出目录，默认使用系统日志目录。",
    )
    parser.add_argument(
        "--log-config",
        type=Path,
        help="日志配置文件（TOML/JSON），覆盖默认配置。",
    )
    parser.add_argument(
        "--no-log-console",
        action="store_true",
        help="关闭控制台日志输出，仅写入文件。",
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
    _configure_logging_from_args(parser, args)
    _install_global_exception_hook()
    data_source = _make_data_source(args.mode)
    app = UAVDashboardApp(data_source=data_source, poll_interval=args.poll_interval)

    with GracefulShutdown() as shutdown:
        shutdown.register_callback(lambda: _request_app_exit(app))

        try:
            app.run()
        except KeyboardInterrupt:
            logger.info("检测到用户中断，正在退出…")
            shutdown.request_shutdown("用户中断 (Ctrl+C)")
            return
        finally:
            if not shutdown.triggered:
                shutdown.request_shutdown("主循环退出")
            shutdown.wait(timeout=2.0)
            _ensure_data_source_stopped(data_source)


__all__ = ["main"]

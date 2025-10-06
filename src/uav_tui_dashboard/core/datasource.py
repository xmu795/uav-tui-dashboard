"""Data source abstractions for the UAV TUI dashboard."""

from __future__ import annotations

import abc
import asyncio
import contextlib
import importlib
import logging
import math
import random
import threading
from typing import Any, Callable, Optional

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
    """ROS2-backed telemetry provider.

    The data source subscribes to odometry and battery topics to populate
    :class:`UAVStatus`. The default expectations are:

    * ``nav_msgs/msg/Odometry`` messages published on ``/uav/odometry``
      providing position and orientation quaternions.
    * ``sensor_msgs/msg/BatteryState`` messages on ``/uav/battery`` providing
      battery percentage and voltage information.

    Both topics and message parsing strategies can be customised via the
    constructor to adapt to different vehicle stacks.
    """

    def __init__(
        self,
        *,
        node_name: str = "uav_dashboard_listener",
        namespace: str | None = None,
        odometry_topic: str = "/uav/odometry",
        battery_topic: str | None = "/uav/battery",
        odometry_msg_type: str = "nav_msgs.msg.Odometry",
        battery_msg_type: str = "sensor_msgs.msg.BatteryState",
        odometry_parser: Optional[Callable[[Any], tuple[Vector3, Vector3]]] = None,
        battery_parser: Optional[Callable[[Any, UAVStatus], tuple[float, float]]] = None,
        qos_depth: int = 10,
        ros_args: Optional[list[str]] = None,
        spin_timeout: float = 0.1,
    ) -> None:
        self._node_name = node_name
        self._namespace = namespace
        self._odometry_topic = odometry_topic
        self._battery_topic = battery_topic
        self._odometry_msg_type = odometry_msg_type
        self._battery_msg_type = battery_msg_type
        self._odometry_parser = odometry_parser or _parse_odometry_message
        self._battery_parser = battery_parser or _parse_battery_message
        self._qos_depth = qos_depth
        self._ros_args = ros_args
        self._spin_timeout = spin_timeout

        self._status = UAVStatus.empty()
        self._status_lock = threading.Lock()
        self._running = False

        # Lazy-imported ROS2 components
        self._rclpy: Any = None
        self._context: Any = None
        self._executor: Any = None
        self._node: Any = None
        self._subscriptions: list[Any] = []
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._odometry_msg_cls: Optional[type[Any]] = None
        self._battery_msg_cls: Optional[type[Any]] = None

    async def start(self) -> None:
        if self._running:
            logger.debug("ROS2 数据源已启动，无需重复初始化")
            return

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._start_blocking)

    def _start_blocking(self) -> None:
        try:
            self._initialise_ros_components()
        except Exception:
            logger.exception("初始化 ROS2 数据源失败")
            raise

        self._stop_event.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_executor,
            name="uav-tui-ros2-spin",
            daemon=True,
        )
        self._spin_thread.start()
        self._running = True
        logger.info("ROS2 数据源已启动，订阅主题: %s, %s", self._odometry_topic, self._battery_topic)

    def _initialise_ros_components(self) -> None:
        try:
            self._rclpy = importlib.import_module("rclpy")
            context_module = importlib.import_module("rclpy.context")
            executors_module = importlib.import_module("rclpy.executors")
            qos_module = importlib.import_module("rclpy.qos")
            node_module = importlib.import_module("rclpy.node")
        except ImportError as exc:  # pragma: no cover - depends on ROS env
            raise RuntimeError(
                "ROS2 模式需要安装 rclpy 以及对应的 ROS2 运行时，请在 ROS2 环境中运行或安装相关依赖。"
            ) from exc

        QoSProfile = getattr(qos_module, "QoSProfile")
        Node = getattr(node_module, "Node")
        SingleThreadedExecutor = getattr(executors_module, "SingleThreadedExecutor")

        self._odometry_msg_cls = _resolve_message_type(self._odometry_msg_type)
        if self._battery_topic is not None:
            self._battery_msg_cls = _resolve_message_type(self._battery_msg_type)

        self._context = context_module.Context()
        self._rclpy.init(args=self._ros_args, context=self._context)
        self._node = Node(
            self._node_name,
            namespace=self._namespace,
            context=self._context,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        qos_profile = QoSProfile(depth=self._qos_depth)

        self._subscriptions = []
        self._subscriptions.append(
            self._node.create_subscription(
                self._odometry_msg_cls,
                self._odometry_topic,
                self._handle_odometry,
                qos_profile,
            )
        )

        if self._battery_topic and self._battery_msg_cls is not None:
            self._subscriptions.append(
                self._node.create_subscription(
                    self._battery_msg_cls,
                    self._battery_topic,
                    self._handle_battery,
                    qos_profile,
                )
            )

        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)

    def _spin_executor(self) -> None:
        assert self._executor is not None
        try:
            while not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=self._spin_timeout)
        except Exception:  # pragma: no cover - defensive logging
            logger.exception("ROS2 executor spin loop terminated unexpectedly")

    async def stop(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._stop_blocking)

    def _stop_blocking(self) -> None:
        self._stop_event.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():  # pragma: no cover - defensive path
                logger.warning("ROS2 executor 线程未在超时内退出，将继续后台运行")
        self._spin_thread = None

        if self._executor is not None:
            with contextlib.suppress(Exception):
                self._executor.remove_node(self._node)
            self._executor.shutdown()
        self._executor = None

        if self._node is not None:
            for subscription in self._subscriptions:
                with contextlib.suppress(Exception):
                    self._node.destroy_subscription(subscription)
            self._subscriptions.clear()
            with contextlib.suppress(Exception):
                self._node.destroy_node()
        self._node = None

        if self._rclpy is not None and self._context is not None:
            try:
                if self._rclpy.ok(context=self._context):
                    self._rclpy.shutdown(context=self._context)
            except Exception:  # pragma: no cover - defensive path
                logger.exception("关闭 ROS2 上下文失败")

        self._context = None
        self._running = False
        logger.info("ROS2 数据源已停止")

    async def fetch(self) -> UAVStatus:
        if not self._running:
            logger.debug("ROS2 数据源尚未启动，返回最近一次缓存的状态")

        with self._status_lock:
            return self._status

    def _handle_odometry(self, message: Any) -> None:
        try:
            position, orientation = self._odometry_parser(message)
        except Exception:
            logger.exception("解析里程计消息失败: %s", type(message).__name__)
            return

        with self._status_lock:
            self._status = (
                self._status.with_position(position).with_orientation(orientation)
            )

    def _handle_battery(self, message: Any) -> None:
        try:
            battery_level, voltage = self._battery_parser(message, self._status)
        except Exception:
            logger.exception("解析电池消息失败: %s", type(message).__name__)
            return

        with self._status_lock:
            self._status = self._status.with_battery(battery_level, voltage)


def _resolve_message_type(dotted_path: str) -> type[Any]:
    try:
        module_name, attr = dotted_path.rsplit(".", 1)
    except ValueError as exc:  # pragma: no cover - defensive path
        raise ValueError(f"无效的消息类型路径: {dotted_path}") from exc

    module = importlib.import_module(module_name)
    try:
        msg_cls = getattr(module, attr)
    except AttributeError as exc:
        raise ValueError(f"模块 {module_name!r} 中不存在消息类型 {attr!r}") from exc

    return msg_cls


def _parse_odometry_message(message: Any) -> tuple[Vector3, Vector3]:
    """Extract position/orientation vectors from a ROS odometry-like message."""

    if hasattr(message, "pose") and hasattr(message.pose, "pose"):
        pose = message.pose.pose
        position_msg = pose.position
        orientation_msg = pose.orientation
        position = (
            float(getattr(position_msg, "x", 0.0)),
            float(getattr(position_msg, "y", 0.0)),
            float(getattr(position_msg, "z", 0.0)),
        )
        orientation = _quaternion_to_euler(
            float(getattr(orientation_msg, "x", 0.0)),
            float(getattr(orientation_msg, "y", 0.0)),
            float(getattr(orientation_msg, "z", 0.0)),
            float(getattr(orientation_msg, "w", 1.0)),
        )
        return position, orientation

    if hasattr(message, "position") and hasattr(message, "q"):
        pos = getattr(message, "position")
        quat = getattr(message, "q")
        position = (
            float(pos[0]) if len(pos) > 0 else 0.0,
            float(pos[1]) if len(pos) > 1 else 0.0,
            float(pos[2]) if len(pos) > 2 else 0.0,
        )
        orientation = _quaternion_to_euler(
            float(quat[0]) if len(quat) > 0 else 0.0,
            float(quat[1]) if len(quat) > 1 else 0.0,
            float(quat[2]) if len(quat) > 2 else 0.0,
            float(quat[3]) if len(quat) > 3 else 1.0,
        )
        return position, orientation

    if hasattr(message, "position") and hasattr(message, "orientation"):
        position_msg = getattr(message, "position")
        orientation_msg = getattr(message, "orientation")
        position = (
            float(getattr(position_msg, "x", 0.0)),
            float(getattr(position_msg, "y", 0.0)),
            float(getattr(position_msg, "z", 0.0)),
        )
        orientation = _quaternion_to_euler(
            float(getattr(orientation_msg, "x", 0.0)),
            float(getattr(orientation_msg, "y", 0.0)),
            float(getattr(orientation_msg, "z", 0.0)),
            float(getattr(orientation_msg, "w", 1.0)),
        )
        return position, orientation

    raise TypeError(
        "无法从消息中解析位置和姿态信息，" f"收到类型: {type(message).__name__}"
    )


def _parse_battery_message(message: Any, current_status: UAVStatus) -> tuple[float, float]:
    """Extract battery percentage (0-100) and voltage from message."""

    battery_level = current_status.battery_level
    voltage = current_status.voltage

    percentage = getattr(message, "percentage", None)
    if percentage is not None:
        try:
            percentage_value = float(percentage)
        except (TypeError, ValueError):
            logger.debug("忽略无法解析的电量百分比值: %s", percentage)
        else:
            if not math.isnan(percentage_value):
                # ROS BatteryState.percentage is 0-1, but may already be 0-100 in some stacks.
                if percentage_value <= 1.0:
                    battery_level = max(0.0, min(100.0, percentage_value * 100.0))
                else:
                    battery_level = max(0.0, min(100.0, percentage_value))

    power_supply_status = getattr(message, "power_supply_status", None)
    if power_supply_status is not None:
        logger.debug("Battery power supply status: %s", power_supply_status)

    msg_voltage = getattr(message, "voltage", None)
    if msg_voltage is not None:
        try:
            voltage_value = float(msg_voltage)
        except (TypeError, ValueError):
            logger.debug("忽略无法解析的电压值: %s", msg_voltage)
        else:
            if not math.isnan(voltage_value) and voltage_value > 0.0:
                voltage = voltage_value
    if voltage == current_status.voltage:
        cell_voltages = getattr(message, "cell_voltage", None)
        if cell_voltages:
            values: list[float] = []
            for cell in cell_voltages:
                try:
                    cell_value = float(cell)
                except (TypeError, ValueError):
                    continue
                if not math.isnan(cell_value) and cell_value > 0.0:
                    values.append(cell_value)
            if values:
                voltage = sum(values) / len(values)

    return battery_level, voltage


def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> Vector3:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


__all__ = ["DataSource", "SimDataSource", "Ros2DataSource"]

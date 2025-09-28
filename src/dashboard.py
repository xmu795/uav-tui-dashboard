"""
UAV TUI Dashboard - 基于 Textual 的无人机状态监控终端界面

功能：
- 实时显示无人机坐标位姿和电量数据
- 简洁的 Linux 风格布局
- 自动数据更新（当前为模拟）

作者：GitHub Copilot
日期：2025-09-28
"""

from textual.app import App, ComposeResult
from textual.widgets import Header, DataTable, Footer
from textual.coordinate import Coordinate
import asyncio
from dataclasses import dataclass
import random

@dataclass
class UAVStatus:
    """
    无人机状态数据类

    包含坐标位姿和电量信息，具有良好的可扩展性。
    未来可添加速度、传感器数据等字段。
    """
    # 坐标位姿
    position: tuple[float, float, float]  # (x, y, z) 位置坐标
    orientation: tuple[float, float, float]  # (roll, pitch, yaw) 姿态角度
    
    # 电量
    battery_level: float  # 电池电量百分比 (0-100)
    voltage: float  # 电压 (V)
    
    # 可扩展字段 (目前为空，未来可添加速度、传感器数据等)
    # velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # sensors: dict[str, float] = None

class UAVDashboard(App):
    """
    UAV Dashboard 主应用类

    继承自 Textual App，实现 TUI 界面。
    显示无人机状态数据，支持实时更新。
    """
    CSS_PATH = "dashboard.css"  # 样式文件路径

    def __init__(self):
        """初始化应用，设置初始状态"""
        super().__init__()
        self.current_status = UAVStatus(
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0),
            battery_level=100.0,
            voltage=12.5
        )

    def compose(self) -> ComposeResult:
        """定义界面组件布局"""
        yield Header(show_clock=True)  # 顶部标题栏，显示时钟
        yield DataTable(id="status_table")  # 数据显示表格
        yield Footer()  # 底部状态栏

    async def on_mount(self):
        """应用挂载时初始化表格和启动更新任务"""
        table = self.query_one("#status_table", DataTable)
        table.add_columns("Field", "Value")  # 添加表头
        # 添加固定行，初始值设为0
        table.add_row("Position X", "0.00")
        table.add_row("Position Y", "0.00")
        table.add_row("Position Z", "0.00")
        table.add_row("Roll", "0.00")
        table.add_row("Pitch", "0.00")
        table.add_row("Yaw", "0.00")
        table.add_row("Battery Level", "100.0%")
        table.add_row("Voltage", "12.50V")
        self.update_task = asyncio.create_task(self.update_data())  # 启动数据更新任务

    async def on_unmount(self):
        """应用卸载时取消更新任务并优雅等待任务结束"""
        if hasattr(self, 'update_task'):
            self.update_task.cancel()
            try:
                await self.update_task
            except asyncio.CancelledError:
                pass

    async def update_data(self):
        """异步数据更新循环，每秒模拟数据变化；支持优雅取消"""
        try:
            while True:
                await asyncio.sleep(1)  # 等待1秒
                # 模拟数据变化：小幅随机改变各字段
                self.current_status.position = (
                    self.current_status.position[0] + random.uniform(-0.1, 0.1),
                    self.current_status.position[1] + random.uniform(-0.1, 0.1),
                    self.current_status.position[2] + random.uniform(-0.05, 0.05)
                )
                self.current_status.orientation = (
                    self.current_status.orientation[0] + random.uniform(-0.01, 0.01),
                    self.current_status.orientation[1] + random.uniform(-0.01, 0.01),
                    self.current_status.orientation[2] + random.uniform(-0.01, 0.01)
                )
                self.current_status.battery_level = max(0, self.current_status.battery_level - random.uniform(0, 0.5))
                self.current_status.voltage = max(10, self.current_status.voltage + random.uniform(-0.1, 0.1))
                self.update_table(self.current_status)  # 更新表格显示
        except asyncio.CancelledError:
            # 任务被取消时正常退出
            return

    def update_table(self, status: UAVStatus):
        """更新数据表格显示"""
        table = self.query_one("#status_table", DataTable)
        # 更新各行数据，不清空表格
        table.update_cell_at(Coordinate(0, 1), f"{status.position[0]:.2f}")
        table.update_cell_at(Coordinate(1, 1), f"{status.position[1]:.2f}")
        table.update_cell_at(Coordinate(2, 1), f"{status.position[2]:.2f}")
        table.update_cell_at(Coordinate(3, 1), f"{status.orientation[0]:.2f}")
        table.update_cell_at(Coordinate(4, 1), f"{status.orientation[1]:.2f}")
        table.update_cell_at(Coordinate(5, 1), f"{status.orientation[2]:.2f}")
        table.update_cell_at(Coordinate(6, 1), f"{status.battery_level:.1f}%")
        table.update_cell_at(Coordinate(7, 1), f"{status.voltage:.2f}V")

if __name__ == "__main__":
    """主入口，运行应用"""
    app = UAVDashboard()
    app.run()

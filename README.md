# UAV TUI Dashboard

基于 Textual 的无人机状态监控终端用户界面 (TUI)。

## 功能

- 实时显示无人机位置、姿态、电池电量和电压
- 模拟数据源用于演示
- 可配置轮询间隔和日志级别
- 优雅关闭和日志记录

## 安装

### 系统要求

- Python 3.8+
- Linux/macOS/Windows

### 安装步骤

1. 克隆仓库：
   ```bash
   git clone https://github.com/xmu795/uav-tui-dashboard.git
   cd uav-tui-dashboard
   ```

2. 创建虚拟环境：
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # Linux/macOS
   # 或 venv\Scripts\activate  # Windows
   ```

3. 安装依赖：
   ```bash
   pip install -e .
   ```

4. （可选）启用 ROS2 支持：
   - 确保已安装 ROS 2 Humble 及以上版本，并在相同终端中 source 对应的环境（如 `source /opt/ros/humble/setup.bash`）。
   - 通过 pip 安装额外依赖：
     ```bash
     pip install -e ".[ros2]"
     ```
     或者直接使用系统包管理器安装 `rclpy`（例如 `sudo apt install ros-humble-rclpy`）。

## 运行

### 模拟模式（默认）

```bash
uav-dashboard
```

或：

```bash
python -m uav_tui_dashboard
```

### ROS2 模式

```bash
uav-dashboard --mode ros2
```

默认情况下，仪表盘会订阅下列主题：

- `/uav/odometry`，消息类型 `nav_msgs/msg/Odometry`
- `/uav/battery`，消息类型 `sensor_msgs/msg/BatteryState`

可使用以下命令行参数进行自定义：

- `--ros-namespace`：节点命名空间
- `--ros-odometry-topic` 与 `--ros-odometry-type`：里程计主题与消息类型
- `--ros-battery-topic` 与 `--ros-battery-type`：电池主题与消息类型（留空主题可禁用订阅）
- `--ros-arg`：传递给 `rclpy.init()` 的额外参数，可多次使用

> 📘 **深入了解数据流与扩展方式**：请阅读 [`docs/ros2_integration.md`](docs/ros2_integration.md)，了解订阅结构、消息解析策略、缓存机制及常见问题解答。

### 命令行参数

- `--mode`: 数据源模式（sim 或 ros2，默认 sim）
- `--poll-interval`: 数据轮询间隔（秒，默认 1.0）
- `--log-level`: 日志级别（默认 INFO）
- `--log-dir`: 日志目录
- `--log-config`: 日志配置文件路径
- `--no-log-console`: 禁用控制台日志输出

### 环境变量

- `UAV_TUI_LOG_DIR`: 日志目录
- `UAV_TUI_LOG_CONFIG`: 日志配置文件路径

## 开发

### 运行测试

```bash
pip install -e ".[dev]"
pytest
```

### 项目结构

```
src/uav_tui_dashboard/
├── cli.py          # 命令行接口
├── core/
│   ├── datasource.py  # 数据源（模拟和 ROS2）
│   └── models.py      # 数据模型
├── ui/
│   ├── app.py         # Textual 应用
│   └── dashboard.css  # 样式
└── logging_config.py  # 日志配置
```

## 许可证

MIT

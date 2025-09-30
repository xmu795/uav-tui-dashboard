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

## 运行

### 模拟模式（默认）

```bash
uav-dashboard
```

或：

```bash
python -m uav_tui_dashboard
```

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

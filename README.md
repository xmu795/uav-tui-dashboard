# UAV TUI Dashboard

一个基于 Textual 的终端用户界面 (TUI) 无人机状态监控仪表板。

## 功能

- **实时数据显示**：展示无人机的坐标位姿（位置、姿态）和电量信息。
- **自动更新**：每秒模拟更新数据，显示动态变化。
- **可扩展性**：数据模型支持未来添加更多字段，如速度、传感器数据。

## 安装

### 系统要求

- Python 3.8+
- Linux/macOS/Windows (推荐 Linux)

### 开发环境设置

1. 克隆仓库：
   ```bash
   git clone https://github.com/yourusername/uav-tui-dashboard.git
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

直接运行：
```bash
uav-dashboard
```

或：
```bash
python -m uav_tui_dashboard
```

### ROS2 模式（未来）

当 ROS2 支持完成后：
```bash
uav-dashboard --ros2
```

## 开发者指南

### 项目结构

```
src/
└── uav_tui_dashboard/
   ├── __main__.py        # 支持 `python -m uav_tui_dashboard`
   ├── cli.py             # 命令行入口 (main 函数)
   ├── __init__.py        # 软件包信息与公共接口
   ├── core/
   │   ├── datasource.py  # 数据源抽象 (SimDataSource, Ros2DataSource)
   │   └── models.py      # 数据模型 (UAVStatus)
   └── ui/
      ├── app.py         # Textual 应用
      └── dashboard.css  # 样式
```

### 开发

1. 安装开发依赖：
   ```bash
   pip install -e ".[dev]"
   ```

2. 运行测试：
   ```bash
   pytest
   ```

3. 代码检查：
   ```bash
   ruff check .
   mypy .
   ```

### 发布

1. 更新版本号 in `pyproject.toml`
2. 创建标签：
   ```bash
   git tag v0.1.0
   git push --tags
   ```
3. 构建发布：
   ```bash
   python -m build
   twine upload dist/*
   ```

## 常见问题

### ROS2 环境变量

如果 ROS2 安装在非标准位置，确保设置：
```bash
export ROS_DISTRO=humble
export AMENT_PREFIX_PATH=/opt/ros/humble
source /opt/ros/humble/setup.bash
```

### 虚拟环境

项目使用本地虚拟环境，不要提交 `venv/` 目录。

### 依赖锁定

生产环境建议使用 `pip-tools` 锁定依赖版本。

## 文件结构

- `src/uav_tui_dashboard/cli.py`：命令行入口与参数解析。
- `src/uav_tui_dashboard/ui/app.py`：Textual 应用。
- `src/uav_tui_dashboard/ui/dashboard.css`：样式文件。
- `src/uav_tui_dashboard/core/datasource.py`：数据源抽象。
- `src/uav_tui_dashboard/core/models.py`：数据模型。
- `pyproject.toml`：项目配置和依赖。
- `requirements.txt`：兼容性依赖列表。
- `docs/`：文档和审查记录。

## 审查文档

- 最新一次提交审查与行动计划：`docs/REVIEW_2025-09-28.md`

## 后续计划

- 接入 ROS2：替换模拟数据为实际 ROS2 节点通信。
- 添加更多状态字段：速度、传感器数据等。
- 增强UI：添加图表、日志面板等。
- CI/CD：GitHub Actions 自动化测试和发布。

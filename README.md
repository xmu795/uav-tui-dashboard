# UAV TUI Dashboard

一个基于 Textual 的终端用户界面 (TUI) 无人机状态监控仪表板。

## 功能

- **实时数据显示**：展示无人机的坐标位姿（位置、姿态）和电量信息。
- **简洁布局**：遵循 Linux 工具设计美学，暗色主题，绿色边框，聚焦数据。
- **自动更新**：每秒模拟更新数据，显示动态变化。
- **可扩展性**：数据模型支持未来添加更多字段，如速度、传感器数据。

## 安装

1. 激活虚拟环境：
   ```bash
   source somewhere/UAV_TUI/bin/activate
   ```

2. 安装依赖（已预装 Textual 等）。

## 运行

```bash
python src/dashboard.py
```

使用 `q` 退出应用。

## 文件结构

- `src/dashboard.py`：主应用代码。
- `src/dashboard.css`：样式文件。
- `UAV_TUI/`：虚拟环境。

## 审查文档

- 最新一次提交审查与行动计划：`docs/REVIEW_2025-09-28.md`

## 后续计划

- 接入 ROS2：替换模拟数据为实际 ROS2 节点通信。
- 添加更多状态字段：速度、传感器数据等。
- 增强UI：添加图表、日志面板等。

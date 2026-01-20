# ROS2 Viser

ROS2 机器人可视化工具，使用 Viser 进行 3D 可视化。

## 功能特性

- 自动从 ROS2 话题订阅机器人描述（URDF）
- 实时可视化机器人关节状态
- 基于 Viser 的现代化 3D 可视化界面
- **完全依赖 `ros2_robot_interface`**：所有 ROS2 通信都通过 `ros2_robot_interface` 完成
- 支持 FSM（有限状态机）控制面板
- 支持夹爪控制面板

## 安装

### 从源码安装

```bash
cd ros2_viser
pip install -e .
```

## 依赖

- `ros2-robot-interface` - **必需**，ROS2 机器人接口（所有 ROS2 通信都通过此接口完成）
- `viser` - 3D 可视化库
- `yourdfpy` - URDF 解析库
- `rclpy` - ROS2 Python 客户端库（通过 `ros2_robot_interface` 间接使用）

## 使用方法

**重要说明**：`ros2_viser` 完全依赖 `ros2_robot_interface`。所有 ROS2 通信（包括关节状态订阅）都通过 `ros2_robot_interface` 完成。`ros2_viser` 会自动创建内部的 `ROS2RobotInterface` 实例。

### 基本使用

```python
from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig
import time

# 创建配置（会自动创建内部的 ROS2RobotInterface）
config = ROS2ViserConfig(
    robot_description_topic="/robot_description",
    joint_states_topic="/joint_states",
    root_node_name="/robot",
    update_rate=30.0
)

# 创建并启动可视化器
visualizer = ROS2ViserVisualizer(config)
visualizer.start()

# 保持运行
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    visualizer.stop()
```

### 使用上下文管理器

```python
from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig
import time

config = ROS2ViserConfig(
    robot_description_topic="/robot_description",
    joint_states_topic="/joint_states"
)

with ROS2ViserVisualizer(config) as visualizer:
    # 可视化器自动启动
    time.sleep(60)  # 运行 60 秒
# 自动清理资源
```

## 配置选项

### ROS2ViserConfig

- `robot_description_topic` (str): 机器人描述话题（URDF），默认 `/robot_description`
- `joint_states_topic` (str): 关节状态话题，默认 `/joint_states`，用于创建内部的 `ROS2RobotInterface`
- `root_node_name` (str): Viser 场景中的根节点名称，默认 `/robot`
- `update_rate` (float): 可视化更新频率（Hz），默认 30.0
- `auto_connect` (bool): 是否自动连接 ROS2 接口，默认 True
- `enable_fsm_panel` (bool): 是否启用 FSM（有限状态机）控制面板，默认 True
- `enable_gripper_panel` (bool): 是否启用夹爪控制面板，默认 True

**注意**：
- 所有 ROS2 通信都通过内部自动创建的 `ros2_robot_interface` 完成，包括关节状态订阅

## 工作原理

1. **初始化 ROS2 接口**：根据 `joint_states_topic` 自动创建 `ROS2RobotInterface` 实例
   
2. **订阅机器人描述**：通过 `ros2_robot_interface` 的 ROS2 节点订阅 `/robot_description` 话题获取 URDF 字符串

3. **解析 URDF**：使用 `yourdfpy` 解析 URDF 并提取关节信息

4. **初始化 Viser**：创建 Viser 服务器并加载 URDF 模型到 3D 场景

5. **获取关节状态**：**通过 `ros2_robot_interface.get_joint_state()` 获取实时关节状态**（所有关节状态都通过接口获取，不直接订阅话题）

6. **更新可视化**：根据关节状态实时更新机器人模型姿态

7. **控制面板**（可选）：
   - FSM 面板：通过 `ros2_robot_interface.send_fsm_command()` 发送状态切换命令
   - 夹爪面板：通过 `ros2_robot_interface` 的夹爪处理器控制夹爪

## 注意事项

- **必须安装 `ros2_robot_interface`**：所有 ROS2 通信都通过此接口完成
- 确保 ROS2 环境中已发布 `/robot_description` 话题（URDF）
- 确保 `/joint_states` 话题正在发布关节状态
- 可视化器会自动等待最多 30 秒来接收机器人描述
- 关节名称必须与 URDF 中的关节名称匹配
- FSM 和夹爪控制面板需要 `ros2_robot_interface` 支持相应的功能

## 示例

完整示例请参考 `examples/` 目录。

## 许可证

Apache-2.0

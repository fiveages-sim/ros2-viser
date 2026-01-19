# ROS2 Viser

ROS2 机器人可视化工具，使用 Viser 进行 3D 可视化。

## 功能特性

- 自动从 ROS2 话题订阅机器人描述（URDF）
- 实时可视化机器人关节状态
- 基于 Viser 的现代化 3D 可视化界面
- 与 `ros2_robot_interface` 集成，复用现有的 ROS2 接口

## 安装

### 从源码安装

```bash
cd ros2_viser
pip install -e .
```

## 依赖

- `ros2-robot-interface` - ROS2 机器人接口
- `viser` - 3D 可视化库
- `yourdfpy` - URDF 解析库
- `rclpy` - ROS2 Python 客户端库

## 使用方法

### 基本使用

```python
from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig
import time

# 创建配置
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

config = ROS2ViserConfig()
with ROS2ViserVisualizer(config) as visualizer:
    # 可视化器自动启动
    time.sleep(60)  # 运行 60 秒
# 自动清理资源
```

### 使用现有的 ROS2RobotInterface

如果你已经有一个 `ROS2RobotInterface` 实例，可以复用：

```python
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_viser import ROS2ViserVisualizer, ROS2ViserConfig

# 创建 ROS2 接口
interface_config = ROS2RobotInterfaceConfig(
    joint_states_topic="/joint_states",
    end_effector_pose_topic="/left_current_pose",
    end_effector_target_topic="/left_target"
)
interface = ROS2RobotInterface(interface_config)
interface.connect()

# 创建可视化器，复用现有接口
viser_config = ROS2ViserConfig(
    ros2_interface=interface,  # 复用现有接口
    robot_description_topic="/robot_description"
)
visualizer = ROS2ViserVisualizer(viser_config)
visualizer.start()
```

## 配置选项

### ROS2ViserConfig

- `robot_description_topic` (str): 机器人描述话题，默认 `/robot_description`
- `joint_states_topic` (str): 关节状态话题，默认 `/joint_states`
- `root_node_name` (str): Viser 场景中的根节点名称，默认 `/robot`
- `update_rate` (float): 更新频率（Hz），默认 30.0
- `auto_connect` (bool): 是否自动连接 ROS2 接口，默认 True
- `ros2_interface` (Optional): 可选的现有 ROS2RobotInterface 实例

## 工作原理

1. **订阅机器人描述**：从 `/robot_description` 话题获取 URDF 字符串
2. **解析 URDF**：使用 `yourdfpy` 解析 URDF 并提取关节信息
3. **初始化 Viser**：创建 Viser 服务器并加载 URDF 模型
4. **订阅关节状态**：通过 `ros2_robot_interface` 获取实时关节状态
5. **更新可视化**：根据关节状态实时更新机器人模型姿态

## 注意事项

- 确保 ROS2 环境中已发布 `/robot_description` 话题
- 确保 `/joint_states` 话题正在发布关节状态
- 可视化器会自动等待最多 30 秒来接收机器人描述
- 关节名称必须与 URDF 中的关节名称匹配

## 示例

完整示例请参考 `examples/` 目录。

## 许可证

Apache-2.0

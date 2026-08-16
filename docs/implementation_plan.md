# ROS 2 工作空间、Package、话题发布订阅及 Launch 启动全流程教程与实现方案

本方案旨在指导并完成从零搭建 ROS 2 (Jazzy) 工作空间、编写 Python 节点实现 Topic 消息发布与订阅、配置 Launch 启动文件，并通过 `colcon` 进行编译与运行验证的全过程。

## 目标与任务分解

1. **创建 ROS 2 工作空间 (Workspace)**：创建标准的 ROS 2 工作空间结构 `demo_ws/src`。
2. **创建 ROS 2 功能包 (Package)**：基于 Python (`ament_python`) 创建功能包 `demo_topic_pkg`，依赖 `rclpy` 和 `std_msgs`。
3. **实现 Topic 发布与订阅节点**：
   - **发布者节点 (`demo_publisher.py`)**：定时向 `/chatter` 话题发布结构化字符串消息。
   - **订阅者节点 (`demo_subscriber.py`)**：接收 `/chatter` 话题消息并在日志中打印。
4. **配置入口点与安装路径**：更新 `setup.py` 注册可执行文件入口点，确保组件可被 `ros2 run` 和 launch 检索。
5. **编写 Launch 启动文件 (`demo_launch.py`)**：使用 ROS 2 Python Launch 系统一次性启动发布者和订阅者节点。
6. **编译与运行验证**：使用 `colcon build` 编译项目，刷新环境变量并测试节点通信与 Launch 启动。
7. **教学与指导**：在 `walkthrough.md` 中详细阐述每个关键文件的作用、核心 API 讲解及使用命令。

---

## User Review Required

> [!NOTE]
> 当前环境检测到系统已安装 **ROS 2 Jazzy**。我们将使用 Python (`rclpy`) 作为开发语言，采用标准的 `ament_python` 构建类型。

---

## Open Questions

无。

---

## Proposed Changes

结构设计如下：

```
/home/wang/Ros2/demo_ws/
└── src/
    └── demo_topic_pkg/
        ├── demo_topic_pkg/
        │   ├── __init__.py
        │   ├── demo_publisher.py
        │   └── demo_subscriber.py
        ├── launch/
        │   └── demo_launch.py
        ├── resource/
        │   └── demo_topic_pkg
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

### [ROS 2 Workspace & Package Initial Setup]

#### [NEW] [demo_ws/src/demo_topic_pkg](file:///home/wang/Ros2/demo_ws/src/demo_topic_pkg)
使用 `ros2 pkg create --build-type ament_python demo_topic_pkg --dependencies rclpy std_msgs` 创建功能包基础设施。

### [Nodes & Launch Implementation]

#### [NEW] [demo_publisher.py](file:///home/wang/Ros2/demo_ws/src/demo_topic_pkg/demo_topic_pkg/demo_publisher.py)
- 创建 `TalkerNode` 类，继承自 `rclpy.node.Node`。
- 初始化发布者 `create_publisher(String, 'chatter', 10)`。
- 创建定时器 `create_timer(1.0, timer_callback)` 周期发送消息。

#### [NEW] [demo_subscriber.py](file:///home/wang/Ros2/demo_ws/src/demo_topic_pkg/demo_topic_pkg/demo_subscriber.py)
- 创建 `ListenerNode` 类，继承自 `rclpy.node.Node`。
- 初始化订阅者 `create_subscription(String, 'chatter', listener_callback, 10)`。
- 收到消息时打印日志 `get_logger().info(...)`。

#### [NEW] [demo_launch.py](file:///home/wang/Ros2/demo_ws/src/demo_topic_pkg/launch/demo_launch.py)
- 使用 `launch.LaunchDescription` 和 `launch_ros.actions.Node` 组织启动描述。
- 同时启动 `publisher_node` 和 `subscriber_node`。

#### [MODIFY] [setup.py](file:///home/wang/Ros2/demo_ws/src/demo_topic_pkg/setup.py)
- 包含 `launch` 目录文件安装配置 `(os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))`。
- 配置 `entry_points` 定义 `publisher_node` 和 `subscriber_node` 的可执行指令。

---

## Verification Plan

### Automated Tests
- 使用 `colcon build --symlink-install` 编译功能包，确保无语法与构建错误。
- 刷新环境变量 `source install/setup.bash` 后运行 `ros2 pkg list | grep demo_topic_pkg` 验证包注册。

### Manual Verification
- **Launch 启动测试**：运行 `ros2 launch demo_topic_pkg demo_launch.py`，观察控制台输出，确认发布者持续发送消息且订阅者正常接收并打印。
- **Topic CLI 工具测试**：在背景任务运行节点时，使用 `ros2 topic list` 及 `ros2 topic echo /chatter` 验证 ROS 2 网络中话题正常广播。

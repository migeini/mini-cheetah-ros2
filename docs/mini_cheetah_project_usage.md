# ROS 2 Mini Cheetah 四足机器人项目快速使用与 AI 复用指南

> **文档标识**：`KB-MINI-CHEETAH-PROJECT-USAGE-2026`  
> **适用对象**：AI 助手 (Antigravity Agent) 及开发者后续复用、维护与二次开发。  
> **核心用途**：记录项目架构、路径约定、节点接口契约、编译启动指令及常见踩坑经验，保证后续对话与新任务中可零门槛快速唤醒与调起。

---

## 1. 项目结构与路径约定

- **ROS 2 工作空间根目录**：`/home/wang/Ros2/demo_ws`
- **核心功能包路径**：`/home/wang/Ros2/demo_ws/src/mini_cheetah_demo`
- **文档保存目录**：`/home/wang/Ros2/demo_ws/docs`

```text
/home/wang/Ros2/demo_ws/
├── docs/
│   ├── mini_cheetah_project_usage.md    # [本指南] 项目快速使用与 AI 复用指南
│   ├── mini_cheetah_ik_teleop_guide.md   # IK 逆运动学与键盘遥控操作指南
│   ├── quadruped_gait_cycle_analysis.md # 12 关节运动周期与多关节协调推导
│   └── mini_cheetah_rviz_simulation.md  # URDF 建模与 RViz 仿真实战
└── src/
    └── mini_cheetah_demo/
        ├── config/
        │   └── mini_cheetah.rviz        # RViz 3D 视图与轨迹渲染配置 (Fixed Frame: odom)
        ├── launch/
        │   ├── mini_cheetah_ik_launch.py # [主启动] IK 逆运动学 + 真实平移 + RViz Launch
        │   └── mini_cheetah_launch.py    # 基础正弦波运动 Launch
        ├── mini_cheetah_demo/
        │   ├── __init__.py
        │   ├── ik_gait_controller.py    # [主节点] 3D 摆线轨迹 + 单腿 IK 解算 + 里程计航位推算 + /odom /robot_path 发布
        │   ├── teleop_keyboard.py       # [控制节点] 键盘按键捕获与 /cmd_vel 发布
        │   └── sine_joint_publisher.py  # 基础 50Hz 正弦波角度发布节点
        ├── urdf/
        │   └── mini_cheetah.urdf.xacro  # 12 自由度四足 URDF/Xacro 3D 模型
        ├── package.xml
        └── setup.py                     # ROS 2 可执行入口与安装规则
```

---

## 2. 节点与话题接口契约 (ROS 2 Interfaces)

| 节点名称 (`executable`) | 核心功能 | 订阅话题 (Subscribe) | 发布话题 (Publish) |
| :--- | :--- | :--- | :--- |
| **`ik_gait_controller`** | IK 逆运动学解算、摆线轨迹规划、里程计航位推算 | `/cmd_vel` (`geometry_msgs/msg/Twist`) | `/joint_states` (`sensor_msgs/msg/JointState`) <br> `/odom` (`nav_msgs/msg/Odometry`) <br> `/robot_path` (`nav_msgs/msg/Path`) <br> `TF: odom -> base_link` |
| **`teleop_keyboard`** | 终端键盘交互遥控 | 无 | `/cmd_vel` (`geometry_msgs/msg/Twist`) |
| **`sine_joint_publisher`** | 基础 12 关节正弦波演示 | 无 | `/joint_states` (`sensor_msgs/msg/JointState`) |
| **`robot_state_publisher`** | ROS 2 官方 TF 树正运动学计算 | `/joint_states` <br> `/robot_description` | `TF: base_link -> 12 连杆` |

---

## 3. 标准编译与运行指令 (AI 快捷指令)

### 3.1 编译与刷新环境
```bash
cd /home/wang/Ros2/demo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mini_cheetah_demo
source install/setup.bash
```

### 3.2 启动核心 IK 仿真系统 (后端 + RViz)
```bash
source /opt/ros/jazzy/setup.bash
source /home/wang/Ros2/demo_ws/install/setup.bash
ros2 launch mini_cheetah_demo mini_cheetah_ik_launch.py
```

### 3.3 启动 Gazebo 真实物理引擎仿真 (Gazebo Harmonic + 重力/碰撞/摩擦力 + RViz)
```bash
source /opt/ros/jazzy/setup.bash
source /home/wang/Ros2/demo_ws/install/setup.bash
ros2 launch mini_cheetah_demo mini_cheetah_gazebo_launch.py
```

### 3.3 启动键盘遥控节点
```bash
source /opt/ros/jazzy/setup.bash
source /home/wang/Ros2/demo_ws/install/setup.bash
ros2 run mini_cheetah_demo teleop_keyboard
```

### 3.4 命令行直接控制机器人速度 (无键盘环境替代方案)
```bash
# 1. 向前奔跑 (0.8 m/s)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# 2. 原地左转弯 (0.6 rad/s)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.6}}"

# 3. 实时查看精准位置坐标 (X, Y, Z)
ros2 topic echo /odom
```

---

## 4. 核心避坑指南与开发最佳实践 (Best Practices)

1. **`launch_ros` 解析 Xacro 字符串错误**：
   在 ROS 2 Jazzy 中使用 `Command(['xacro ', xacro_file])` 时，**必须**使用 `ParameterValue(..., value_type=str)` 包装，防止其被误当作 YAML 结构解析。
2. **RViz2 OpenGL 与环境变量丢失**：
   在 Launch 文件中给 `rviz2` 传环境变量时，必须继承当前环境变量 `rviz_env = dict(os.environ)` 并设置 `rviz_env['QT_QPA_PLATFORM'] = 'xcb'`，否则会导致缺失 `libOgreMain.so` 或崩溃。
3. **`teleop_keyboard` 健全性校验**：
   节点内包含 `sys.stdin.isatty()` 和 `if rclpy.ok():` 判断，防止在非交互终端或退出 Ctrl+C 时发生 `RCLError: publisher's context is invalid`。
4. **代码修改即时生效**：
   编译时务必使用 `--symlink-install` 标记，后续修改 Python 代码后无需重新执行 `colcon build` 即可直接生效。

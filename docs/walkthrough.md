# Mini Cheetah 四足机器人 12 关节正弦运动 RViz 仿真教程

本教程已在 `/home/wang/Ros2/demo_ws/src/mini_cheetah_demo` 中全自动完成了简化版 Mini Cheetah 四足机器人的 3D 模型定义、12 关节 TF 树广播、正弦波动态控制节点编写以及 RViz2 仿真可视化。

---

## 1. 任务完成指标校验

| 验收项目 | 状态 | 详细说明 |
| :--- | :---: | :--- |
| **Xacro 模型定义** | ✅ 已完成 | [mini_cheetah.urdf.xacro](file:///home/wang/Ros2/demo_ws/src/mini_cheetah_demo/urdf/mini_cheetah.urdf.xacro) 包含 `base_link` 躯干及 FL/FR/RL/RR 4 条腿 |
| **12 关节结构定义** | ✅ 已完成 | 每条腿包含 `hip_joint`(Roll), `thigh_joint`(Pitch), `calf_joint`(Pitch)，共 12 个转动关节 |
| **`/joint_states` 发布**| ✅ 已完成 | [sine_joint_publisher.py](file:///home/wang/Ros2/demo_ws/src/mini_cheetah_demo/mini_cheetah_demo/sine_joint_publisher.py) 以 50Hz 持续发布 12 关节运动数据 |
| **`robot_state_publisher`**| ✅ 已完成 | 读取 URDF 解析 TF 树，持续广播 3D 空间变换信息 |
| **正弦步态运动** | ✅ 已完成 | 正弦波模拟 Trot 对角对步运动（FL/RR 与 FR/RL 异相摆动） |
| **RViz2 3D 可视化** | ✅ 已完成 | [mini_cheetah.rviz](file:///home/wang/Ros2/demo_ws/src/mini_cheetah_demo/config/mini_cheetah.rviz) 自动加载并显示坐标系与机械臂动态 |

---

## 2. 项目核心代码文件结构

```text
mini_cheetah_demo/
├── config/
│   └── mini_cheetah.rviz            # RViz 显示配置文件
├── launch/
│   └── mini_cheetah_launch.py       # 一键启动机器人状态发布者、正弦节点与 RViz2
├── mini_cheetah_demo/
│   ├── __init__.py
│   └── sine_joint_publisher.py      # 12 关节正弦运动计算与发布节点
├── urdf/
│   └── mini_cheetah.urdf.xacro      # 简化版 12 自由度四足机器人 URDF 宏定义
├── package.xml
└── setup.py                         # 资源与可执行程序入口注册
```

---

## 3. 核心节点与运动学算子讲解

### (1) 12 关节正弦逻辑 `sine_joint_publisher.py`

发布者周期性（50Hz）发布下述 12 个关节的绝对弧度位置：

```python
# 12 个关节名称清单
self.joint_names = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
]

# 对角足相位映射
phase_fl_rr = math.sin(freq * t)
phase_fr_rl = math.sin(freq * t + math.pi)

# 关节运动正弦算子 (Roll + Pitch)
fl_hip   = 0.15 * math.sin(freq * t)
fl_thigh = 0.4 + 0.4 * phase_fl_rr
fl_calf  = -1.0 - 0.5 * phase_fl_rr
```

### (2) 整体启动描述 `mini_cheetah_launch.py`

同时调度 `robot_state_publisher` (解析 Xacro 生成 `/robot_description`)、正弦发布节点以及 `rviz2` 可视化软件：

```python
robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

return LaunchDescription([
    Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description]),
    Node(package='mini_cheetah_demo', executable='sine_joint_publisher'),
    Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_file])
])
```

---

## 4. 命令行手动运行指南

如需在新的终端窗口中重新运行本 Demo：

```bash
# 1. 进入工作空间
cd ~/Ros2/demo_ws

# 2. 编译 mini_cheetah_demo 功能包
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mini_cheetah_demo

# 3. 刷新环境变量并一键 Launch 启动
source install/setup.bash
ros2 launch mini_cheetah_demo mini_cheetah_launch.py
```

### 调试与验证命令：

- **验证 TF 树完整性**：
  ```bash
  ros2 run tf2_tools view_frames
  ```
- **查看关节状态实时数据**：
  ```bash
  ros2 topic echo /joint_states
  ```

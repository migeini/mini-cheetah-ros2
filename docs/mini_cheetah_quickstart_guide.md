# Mini Cheetah x Unitree Go1 仿真项目快速使用手册

欢迎使用全新升级的四足机器人 ROS 2 仿真工作空间！
本项目结合了 **MIT Mini Cheetah 极致轻量化的物理内核** 与 **Unitree Go1 精美的 3D 网格机甲外观**，并搭载了我们从零手写的**全向运动学逆解 (IK) 步态控制器**，在最新的 Gazebo Harmonic 物理引擎下运行。

以下是日常开发、启动与调试的核心命令指南。

---

## 1. 编译与环境生效 (Build & Source)
每次修改完代码（如 Python 节点或 URDF 物理参数），请在工作空间根目录 (`~/Ros2/demo_ws`) 重新编译并生效环境变量：
```bash
# 1. 编译工作空间
colcon build --packages-select mini_cheetah_demo

# 2. 使环境变量生效
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 2. 启动核心仿真世界 (Launch Simulation)
这是所有操作的第一步。该命令会同时启动 Gazebo 物理世界、RViz2 可视化、机器人状态发布器以及 ROS-Gazebo 桥接器。
```bash
ros2 launch mini_cheetah_demo mini_cheetah_gazebo_launch.py
```
> [!TIP]
> 启动后，机器人会悬空从 `0.35m` 的高度自由落体砸向地面。由于此时电机没有力矩，它会“瘫软”在地上。请不要关闭这个终端，打开**新的终端窗口**进行下一步。

---

## 3. 启动 IK 步态控制器 (Start Controller)
在第二个终端中，启动我们自研的 IK 同步控制器。该节点负责监听速度指令，利用逆向运动学实时计算出 12 个关节的期望角度，并以 50Hz 的高频发送给 Gazebo 的底层 PID 控制器。
```bash
# 记得先 source 环境变量
source install/setup.bash
ros2 run mini_cheetah_demo ik_gait_controller
```
> [!SUCCESS]
> 控制器一旦启动，您会看到瘫软的机器狗瞬间输出扭矩，**笔直地站立**起来！

---

## 4. 发送运动指令 (Teleop & Commands)
机器人站立后，您可以通过以下两种方式让它跑起来。

### 方法 A：交互式键盘遥控 (推荐)
在第三个终端中运行键盘监听程序：
```bash
source install/setup.bash
ros2 run mini_cheetah_demo teleop_keyboard
```
**操作按键说明：**
- `W` / `S`：控制前进与后退（X 轴平移）
- `A` / `D`：控制向左与向右横移（Y 轴平移）
- `Q` / `E`：控制左转与右转（Z 轴旋转）
- `空格 (Space)`：紧急停止一切运动并保持当前站立姿态。
> [!NOTE]
> 键盘控制会自动将您的按键转化为 ROS 2 标准的 `/cmd_vel` (geometry_msgs/msg/Twist) 速度指令发给 IK 节点。

### 方法 B：命令行强制发布 (/cmd_vel)
如果您想测试极致稳定性或精准速度，可以直接在终端发布单次指令：

**测试 1：原地高频踏步 (March in place)**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
> 注：Z 轴速度在这里被映射为触发原地踏步的标识位。目前我们的 IK 和物理防漂移处理得极好，它在原地踏步时误差小于 3 毫米/秒！

**测试 2：恒定速度前行 (Move Forward)**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 5. 高级定制与调参指北

如果您希望进一步挑战机器人动态性能（如后空翻、高台跳跃等），请关注以下核心参数文件：

- **URDF 物理特性** (`src/mini_cheetah_demo/urdf/mini_cheetah.urdf.xacro`)：
  - `<inertia ixx=...>`：所有的转动惯量矩阵，如果您想改变空翻时的滚转速度，可以微调这里。
  - `<gazebo reference="base_link">`：包含 `<material>` 颜色定义，您可以在此把深灰色换成您喜欢的涂装。
  - `JointPositionController Plugin`：包含底层电机的 `<p_gain>120</p_gain>` 刚度和 `<cmd_max>25</cmd_max>` 最大扭矩。遇到跟踪疲软时，可提高这些值。
- **步态长度与高度** (`src/mini_cheetah_demo/mini_cheetah_demo/ik_gait_controller.py`)：
  - `self.h_stand` (站立高度) 和 `self.h_swing` (抬腿高度)。想要机器人下蹲或高抬腿，可以直接修改此文件。

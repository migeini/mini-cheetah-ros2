# Mini Cheetah 逆运动学 (IK) 轨迹规划与键盘遥控 (`/cmd_vel`) 进阶实战指南

> **文档标识**：`KB-MINI-CHEETAH-IK-TELEOP-2026`  
> **适用版本**：ROS 2 Jazzy / Humble / Iron  
> **进阶亮点**：从“正弦硬编码角度”全面升级为“3D 笛卡尔摆线轨迹 + 单腿逆运动学 (IK) + ROS 2 标准 `/cmd_vel` 速度键盘遥控系统”！

---

## 1. 进阶核心原理

本次进阶将运动控制分为了 **两层架构**：

```text
┌─────────────────────────────────────────────────────────────┐
│                 teleop_keyboard (键盘控制节点)               │
│  按 W/S/A/D/J/L 键交互，实时发布机器狗的进退、侧移与旋转速度     │
└──────────────────────────────┬──────────────────────────────┘
                               │ 话题: /cmd_vel
                               │ 消息: geometry_msgs/msg/Twist
                               ▼
┌─────────────────────────────────────────────────────────────┐
│               ik_gait_controller (IK 步态控制器)             │
│  1. 根据速度指令动态调整步长                                  │
│  2. 计算足端摆线 3D 轨迹 (x, y, z)                             │
│  3. 求解单腿逆运动学 (IK) 反算 12 个关节角度                 │
└──────────────────────────────┬──────────────────────────────┘
                               │ 话题: /joint_states
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                 robot_state_publisher + RViz2               │
│  计算 TF 树并在 3D 界面中实时绘制根据键盘命令运动的机器狗姿态   │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 核心代理解算模块

### 2.1 单腿逆运动学 (IK) 几何求解器

在 [ik_gait_controller.py](file:///home/wang/Ros2/demo_ws/src/mini_cheetah_demo/mini_cheetah_demo/ik_gait_controller.py#L55-L95) 中，给定足端目标点 $(X, Y, Z)$，反算 3 个关节角度：

```python
def leg_ik(self, x, y, z, is_left=True):
    # 1. 计算 Hip 髋关节 Roll 侧倾角
    sign_y = 1.0 if is_left else -1.0
    y_offset = y - sign_y * self.l1
    l_yz = math.sqrt(y_offset**2 + z**2)
    th_hip = math.atan2(y_offset, -z)

    # 2. 计算矢状面 (X-Z) 有效腿长 d
    d = math.sqrt(x**2 + l_yz**2)

    # 3. 余弦定理解算 Calf 小腿膝盖角度
    cos_gamma = (self.l2**2 + self.l3**2 - d**2) / (2.0 * self.l2 * self.l3)
    gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
    th_calf = gamma - math.pi

    # 4. 余弦定理解算 Thigh 大腿角度
    alpha = math.atan2(x, -z)
    cos_beta = (self.l2**2 + d**2 - self.l3**2) / (2.0 * self.l2 * d)
    beta = math.acos(max(-1.0, min(1.0, cos_beta)))
    th_thigh = alpha + beta

    return th_hip, th_thigh, th_calf
```

---

### 2.2 摆线足端轨迹规划 (Cycloid Trajectory)

为了让脚尖在空中平滑抬起、迈步时不踩硬地：

```python
# 悬空摆动相 (Swing Phase): 摆线轨迹高高抬起
dx = -step_x/2.0 + step_x * (s - math.sin(2.0*math.pi*s)/(2.0*math.pi))
dz = self.h_swing * math.sin(math.pi * s)

# 地面支撑相 (Stance Phase): 直线向后蹬地
dx = step_x/2.0 - step_x * s
dz = 0.0
```

---

## 3. 键盘遥控按键指南

在新的终端窗口中运行 `teleop_keyboard` 节点后：

| 按键 | 功能描述 |
| :---: | :--- |
| **`W` / `S`** | **增加 / 减少 前后前进速度 ($V_x$)** |
| **`A` / `D`** | **增加 / 减少 左右旋转角速度 ($\omega_z$)** |
| **`J` / `L`** | **增加 / 减少 左右侧移速度 ($V_y$)** |
| **`Space` 或 `K`** | **急停刹车（重置速度为 0）** |
| **`Q`** | **退出遥控节点** |

---

## 4. 完整运行与操控流程

### 步骤 1：启动 IK 与 RViz 仿真界面
```bash
cd ~/Ros2/demo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mini_cheetah_demo mini_cheetah_ik_launch.py
```

### 步骤 2：打开新终端，启动键盘遥控节点
```bash
cd ~/Ros2/demo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mini_cheetah_demo teleop_keyboard
```

按下 `W` 键，可在 RViz 中看到机器狗根据您的输入加速向前小跑；按下 `A` 或 `D` 键，机器狗将动态进行原地旋转与转向！

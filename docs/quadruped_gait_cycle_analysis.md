# Mini Cheetah 四足机器人：运动计算全周期拆解与多关节协调联系指南

> **文档标识**：`KB-QUADRUPED-GAIT-CYCLE-2026`  
> **核心目标**：结合核心 Python 源码 [sine_joint_publisher.py](file:///home/wang/Ros2/demo_ws/src/mini_cheetah_demo/mini_cheetah_demo/sine_joint_publisher.py)，彻底理清 12 个关节在一个完整步态周期（Cycle）内的代码计算逻辑、单腿连杆运动耦合机制及四腿之间的相位时空联系。

---

## 1. 运动周期的概念与代码时间基准

在控制节点 `sine_joint_publisher.py` 中，时间与角频率的定义如下：

```python
# 1. 设置 50Hz 定时器 (每隔 0.02 秒触发一次回调计算)
self.timer = self.create_timer(0.02, self.timer_callback)

def timer_callback(self):
    # 2. 计算当前时间点 t (单位：秒)
    now_sec = self.get_clock().now().nanoseconds / 1e9
    t = now_sec - self.start_time

    # 3. 设置摆动角频率
    freq = 3.0  # rad/s
```

### 周期计算公式：
- **角频率 ($\omega$)**：`freq = 3.0 rad/s`
- **单次完整摆动周期 ($T$)**：
  $$ T = \frac{2\pi}{\omega} = \frac{2 \times 3.14159}{3.0} \approx 2.09 \text{ 秒} $$

> **代码物理含义**：每隔 $2.09\text{s}$，随着 `t` 的递增，正弦函数完成一整圈（$2\pi$ 弧度）循环，机器狗的 4 条腿刚好完整重复一次“抬腿迈步-下踩支撑-蹬地向后”的复位循环。

---

## 2. 一个完整周期 ($T$) 内部 4 个阶段的代码与数值演变

在 `timer_callback` 中，正弦波相位的核心计算代码如下：

```python
# Group A (左前 FL & 右后 RR): 基础正弦相位
phase_fl_rr = math.sin(freq * t)

# Group B (右前 FR & 左后 RL): 反向相位 (相差 pi)
phase_fr_rl = math.sin(freq * t + math.pi)
```

我们将一个周期（$0 \to T \approx 2.09\text{s}$）拆解为 4 个关键时间点，对照代码中的 `phase_fl_rr` 变化：

```text
时间切片   代码运行时间 t   正弦波代码 phase_fl_rr    FL/RR 组动作 (Group A)       FR/RL 组动作 (Group B)
───────── ──────────────── ────────────────────── ────────────────────────── ──────────────────────────
t = 0        t = 0.00s        math.sin(0) = 0.0      中立起点 (准备抬腿迈步)       中立起点 (准备下踩蹬地)
t = 0.25T    t = 0.52s        math.sin(π/2) = +1.0   摆动顶点 (腿收缩高高抬起)     支撑顶点 (腿伸直踩地蹬地)
t = 0.50T    t = 1.05s        math.sin(π) = 0.0      交接点 (脚尖落地触地)         交接点 (离地准备抬起)
t = 0.75T    t = 1.57s        math.sin(3π/2) = -1.0  支撑顶点 (腿伸直踩地蹬地)     摆动顶点 (腿收缩高高抬起)
t = 1.00T    t = 2.09s        math.sin(2π) = 0.0     终点复位 (完成一个周期)       终点复位 (完成一个周期)
```

---

## 3. 各关节运动的代码联系与耦合机制

### 3.1 单腿内部：大腿 (Thigh) 与 小腿 (Calf) 的代码协同

在单条腿内部，大腿 `thigh` 控制前后摆动，小腿 `calf` 控制屈膝伸直。

看 [sine_joint_publisher.py](file:///home/wang/Ros2/demo_ws/src/mini_cheetah_demo/mini_cheetah_demo/sine_joint_publisher.py#L48-L51) 中左前腿 (FL) 的计算代码：

```python
# FL (左前腿) 代码实现
fl_thigh = 0.4 + 0.4 * phase_fl_rr   # 大腿：控制前后摆动
fl_calf  = -1.0 - 0.5 * phase_fl_rr  # 小腿：控制膝盖屈伸
```

#### 代码协同逻辑解析：

```text
               【phase_fl_rr 变大 (比如变为 +1.0)】
                                │
          ┌─────────────────────┴─────────────────────┐
          ▼                                           ▼
【fl_thigh = 0.4 + 0.4*(1.0) = 0.8rad】    【fl_calf = -1.0 - 0.5*(1.0) = -1.5rad】
  大腿向前摆动 (向前迈步)                    小腿膝盖深度弯曲 (收缩屈膝)
          │                                           │
          └─────────────────────┬─────────────────────┘
                                ▼
              【脚尖高高提起，空中迈步绝不刮蹭地面！】
```

对比两个极端时刻的代码计算结果：

1. **空中迈步顶点 ($t = 0.52\text{s}$, `phase_fl_rr = +1.0`)**：
   - `fl_thigh = 0.4 + 0.4 * (1.0) = 0.8 rad` ($\approx 46^\circ$，大腿向前摆到极限)
   - `fl_calf  = -1.0 - 0.5 * (1.0) = -1.5 rad` ($\approx -86^\circ$，小腿膝盖收紧折叠，把脚提离地面)

2. **蹬地推进顶点 ($t = 1.57\text{s}$, `phase_fl_rr = -1.0`)**：
   - `fl_thigh = 0.4 + 0.4 * (-1.0) = 0.0 rad` ($\approx 0^\circ$，大腿向后摆到垂直)
   - `fl_calf  = -1.0 - 0.5 * (-1.0) = -0.5 rad` ($\approx -28.6^\circ$，小腿伸直，下踩支撑蹬地)

---

### 3.2 四条腿之间：对角线的相位“绑定”与“互补”代码

四腿协调的关键在于利用了 `math.pi`（$\pi$ 弧度，即半个周期）的相位差：

```python
# 1. 定义对角相位
phase_fl_rr = math.sin(freq * t)               # 对角组 1: 左前 FL + 右后 RR
phase_fr_rl = math.sin(freq * t + math.pi)     # 对角组 2: 右前 FR + 左后 RL (反相位)

# 2. FL (左前腿) - 属于组 1
fl_hip   = 0.15 * math.sin(freq * t)
fl_thigh = 0.4 + 0.4 * phase_fl_rr
fl_calf  = -1.0 - 0.5 * phase_fl_rr

# 3. FR (右前腿) - 属于组 2 (使用 phase_fr_rl)
fr_hip   = -0.15 * math.sin(freq * t)
fr_thigh = 0.4 + 0.4 * phase_fr_rl
fr_calf  = -1.0 - 0.5 * phase_fr_rl

# 4. RL (左后腿) - 属于组 2 (使用 phase_fr_rl)
rl_hip   = 0.15 * math.sin(freq * t)
rl_thigh = 0.4 + 0.4 * phase_fr_rl
rl_calf  = -1.0 - 0.5 * phase_fr_rl

# 5. RR (右后腿) - 属于组 1 (使用 phase_fl_rr)
rr_hip   = -0.15 * math.sin(freq * t)
rr_thigh = 0.4 + 0.4 * phase_fl_rr
rr_calf  = -1.0 - 0.5 * phase_fl_rr
```

#### 代码联动的物理效果：
- 当 `phase_fl_rr` 等于 $+1.0$ 时，`phase_fr_rl` 刚好等于 $-1.0$。
- 这意味着：**FL 和 RR 正在悬空迈步时，FR 和 RL 必然在地面用力支撑**。两条对角线腿完美轮流交替，维持机身永远不会倾覆。

---

### 3.3 髋关节 (Hip Roll) 控制机身平衡的代码

髋关节代码负责左右摇晃：

```python
fl_hip =  0.15 * math.sin(freq * t)  # 左侧腿 (FL & RL) 正向摇
fr_hip = -0.15 * math.sin(freq * t)  # 右侧腿 (FR & RR) 反向摇
```

- **代码作用**：在腿向前迈出的同时，髋关节产生 $\pm 0.15\text{ rad} (\approx \pm 8.6^\circ)$ 的小幅微摇，把机身重心微微压向正踩在地面上的支撑腿侧，仅凭运动学代码就实现了逼真的左右平稳摇晃。

---

## 4. 总结：一个周期内 12 关节运动与 ROS 2 消息发送代码

每隔 $0.02\text{s}$，节点将计算出的 12 个关节角度打包发布给 ROS 2：

```python
# 构造 ROS 2 消息
msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = self.joint_names

# 填充计算出的 12 关节位置数组
msg.position = [
    fl_hip, fl_thigh, fl_calf,   # 1, 2, 3  (左前腿)
    fr_hip, fr_thigh, fr_calf,   # 4, 5, 6  (右前腿)
    rl_hip, rl_thigh, rl_calf,   # 7, 8, 9  (左后腿)
    rr_hip, rr_thigh, rr_calf    # 10,11,12 (右后腿)
]

# 发布到 /joint_states 话题，供 RViz2 渲染
self.publisher_.publish(msg)
```

### 一个完整周期（2.09s）12 关节计算结果对应表：

| 时间点 | 周期进度 | 正弦值 `phase_fl_rr` | 代码输出 FL/RR 角度 (弧度/角度) | 代码输出 FR/RL 角度 (弧度/角度) | 动作状态说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **$t = 0.00\text{s}$** | $0$ | `0.0` | Thigh: `0.4` ($23^\circ$)<br>Calf: `-1.0` ($-57^\circ$) | Thigh: `0.4` ($23^\circ$)<br>Calf: `-1.0` ($-57^\circ$) | 中立位置 |
| **$t = 0.52\text{s}$** | $\frac{1}{4}T$ | `+1.0` | Thigh: **`0.8` ($46^\circ$)**<br>Calf: **`-1.5` ($-86^\circ$)** | Thigh: **`0.0` ($0^\circ$)**<br>Calf: **`-0.5` ($-28.6^\circ$)** | **FL/RR 高高抬起迈步**<br>**FR/RL 伸直着地支撑** |
| **$t = 1.05\text{s}$** | $\frac{1}{2}T$ | `0.0` | Thigh: `0.4` ($23^\circ$)<br>Calf: `-1.0` ($-57^\circ$) | Thigh: `0.4` ($23^\circ$)<br>Calf: `-1.0` ($-57^\circ$) | 脚尖触地交接 |
| **$t = 1.57\text{s}$** | $\frac{3}{4}T$ | `-1.0` | Thigh: **`0.0` ($0^\circ$)**<br>Calf: **`-0.5` ($-28.6^\circ$)** | Thigh: **`0.8` ($46^\circ$)**<br>Calf: **`-1.5` ($-86^\circ$)** | **FL/RR 伸直着地支撑**<br>**FR/RL 高高抬起迈步** |
| **$t = 2.09\text{s}$** | $1.0T$ | `0.0` | Thigh: `0.4` ($23^\circ$)<br>Calf: `-1.0` ($-57^\circ$) | Thigh: `0.4` ($23^\circ$)<br>Calf: `-1.0` ($-57^\circ$) | 完成一个完整周期循环 |

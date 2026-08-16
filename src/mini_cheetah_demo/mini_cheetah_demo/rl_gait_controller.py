#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import os

try:
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

class RLGaitController(Node):
    def __init__(self):
        super().__init__('rl_gait_controller')
        
        # --- 1. RL 模型参数 (遵循 rl_sar 业界标准) ---
        self.freq = 50.0
        self.dt = 1.0 / self.freq
        
        # 默认的 nominal 关节角度 (Hip, Thigh, Calf)
        self.default_dof_pos = np.array([
            0.0, 0.8, -1.6,  # FL
            0.0, 0.8, -1.6,  # FR
            0.0, 0.8, -1.6,  # RL
            0.0, 0.8, -1.6   # RR
        ])
        
        self.action_scale = 0.25
        
        # --- 2. 状态记忆区 ---
        self.commands = np.zeros(3)      # [vx, vy, yaw_rate]
        self.base_lin_vel = np.zeros(3)  # 估算的线速度
        self.base_ang_vel = np.zeros(3)  # IMU 角速度
        self.projected_gravity = np.array([0.0, 0.0, -1.0]) # 旋转后的重力
        
        self.dof_pos = np.zeros(12)
        self.dof_vel = np.zeros(12)
        self.actions = np.zeros(12)      # Previous actions
        
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        # --- 3. 初始化 ONNX 推理引擎 ---
        self.model_path = os.path.expanduser('~/Ros2/demo_ws/models/policy.onnx')
        self.ort_session = None
        if HAS_ONNX:
            if os.path.exists(self.model_path):
                self.ort_session = ort.InferenceSession(self.model_path)
                self.get_logger().info(f"✅ 成功加载开源 ONNX 模型: {self.model_path}")
            else:
                self.get_logger().warn(f"❌ 找不到 ONNX 模型: {self.model_path}")
                self.get_logger().warn("⚠️ 系统将进入 [Dummy] 模拟模式 (输出默认站立姿势)。")
        else:
            self.get_logger().error("❌ 未安装 onnxruntime 库！请运行: pip3 install onnxruntime numpy")
            self.get_logger().warn("⚠️ 系统将进入 [Dummy] 模拟模式 (输出默认站立姿势)。")

        # --- 4. ROS 2 通信接口 ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.joint_pubs = {}
        for name in self.joint_names:
            self.joint_pubs[name] = self.create_publisher(Float64, f'/cmd_pos/{name}', 10)
            
        self.timer = self.create_timer(self.dt, self.inference_loop)

    def cmd_vel_callback(self, msg):
        self.commands[0] = msg.linear.x
        self.commands[1] = msg.linear.y
        self.commands[2] = msg.angular.z

    def imu_callback(self, msg):
        self.base_ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # 将世界坐标系的重力 [0, 0, -1] 投影到机身局部坐标系 (Projected Gravity)
        q = msg.orientation
        # 使用四元数旋转矩阵公式计算 Z 轴的投影
        self.projected_gravity = np.array([
            2 * (q.x * q.z - q.w * q.y),
            2 * (q.y * q.z + q.w * q.x),
            1 - 2 * (q.x**2 + q.y**2)
        ]) * -1.0 # 重力向下

    def joint_state_callback(self, msg):
        if not msg.name: return
        for idx, name in enumerate(self.joint_names):
            if name in msg.name:
                i = msg.name.index(name)
                if len(msg.position) > i: self.dof_pos[idx] = msg.position[i]
                if len(msg.velocity) > i: self.dof_vel[idx] = msg.velocity[i]

    def inference_loop(self):
        """核心推断循环 50Hz"""
        
        # 1. 组装标准 48 维观测矩阵 (Observation Matrix)
        # 大部分开源模型要求顺序：[线速度(3), 角速度(3), 投影重力(3), 命令(3), 关节误差(12), 关节速度(12), 上一帧动作(12)]
        obs = np.concatenate([
            self.base_lin_vel,                   # 3
            self.base_ang_vel,                   # 3
            self.projected_gravity,              # 3
            self.commands,                       # 3
            (self.dof_pos - self.default_dof_pos), # 12
            self.dof_vel,                        # 12
            self.actions                         # 12
        ]).astype(np.float32)

        # 扩充 batch 维度
        obs_tensor = np.expand_dims(obs, axis=0)
        
        # 为了不刷屏，只在运动时打印感知矩阵的前 6 维
        if np.linalg.norm(self.commands) > 0.1:
            self.get_logger().debug(f"Obs Gravity: {self.projected_gravity.round(2)}, Cmd: {self.commands.round(2)}")

        # 2. 推理计算 (Action Decoder)
        if self.ort_session is not None:
            # 真实 ONNX 推理
            ort_inputs = {self.ort_session.get_inputs()[0].name: obs_tensor}
            ort_outs = self.ort_session.run(None, ort_inputs)
            self.actions = ort_outs[0][0] # 获取模型输出 [12]
        else:
            # Dummy 模式：输出极小的随机噪音，让机器狗仅仅保持站立
            self.actions = np.zeros(12)
            
        # 3. 动作解码 (Action Scaling) 并转换回物理角度
        # 经典公式: Target_Pos = Nominal_Pos + Action * Action_Scale
        target_pos = self.default_dof_pos + self.actions * self.action_scale
        
        # 4. 发布执行
        for idx, name in enumerate(self.joint_names):
            msg = Float64()
            msg.data = float(target_pos[idx])
            self.joint_pubs[name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLGaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

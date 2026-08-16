#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64

class RLGaitController(Node):
    def __init__(self):
        super().__init__('rl_gait_controller')
        
        # --- 1. 策略超参数 ---
        self.freq = 50.0  # 控制频率 50Hz
        self.dt = 1.0 / self.freq
        
        # --- 2. 状态缓冲 (State Buffer) ---
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.imu_ang_vel = [0.0, 0.0, 0.0]
        
        self.contact_states = {'FL': False, 'FR': False, 'RL': False, 'RR': False}
        self.joint_positions = [0.0] * 12
        self.joint_velocities = [0.0] * 12
        
        # 关节名称顺序必须严格对齐 RL 模型的期望顺序
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        # --- 3. 订阅器 (感知神经系统) ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 4 个脚底触觉传感器
        self.create_subscription(Contacts, '/contact/FL', lambda msg: self.contact_callback(msg, 'FL'), 10)
        self.create_subscription(Contacts, '/contact/FR', lambda msg: self.contact_callback(msg, 'FR'), 10)
        self.create_subscription(Contacts, '/contact/RL', lambda msg: self.contact_callback(msg, 'RL'), 10)
        self.create_subscription(Contacts, '/contact/RR', lambda msg: self.contact_callback(msg, 'RR'), 10)

        # --- 4. 发布器 (运动执行系统) ---
        self.joint_pubs = {}
        for name in self.joint_names:
            self.joint_pubs[name] = self.create_publisher(Float64, f'/cmd_pos/{name}', 10)
            
        # --- 5. 主循环定时器 ---
        self.timer = self.create_timer(self.dt, self.inference_loop)
        
        self.get_logger().info("✅ 强化学习神经系统节点已启动！正在等待传感器数据...")

    # ================= 回调函数 =================
    def cmd_vel_callback(self, msg):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    def imu_callback(self, msg):
        # 简单的四元数转欧拉角 (为了给 RL 更好的观测值)
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.imu_pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi/2, sinp)
        
        self.imu_ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def joint_state_callback(self, msg):
        # 将无序的 joint_states 重新按照 self.joint_names 的顺序排列
        if not msg.name:
            return
            
        for idx, name in enumerate(self.joint_names):
            if name in msg.name:
                i = msg.name.index(name)
                if len(msg.position) > i:
                    self.joint_positions[idx] = msg.position[i]
                if len(msg.velocity) > i:
                    self.joint_velocities[idx] = msg.velocity[i]

    def contact_callback(self, msg, leg_name):
        # 如果 msg.contacts 有数据，说明发生了碰撞（触地）
        self.contact_states[leg_name] = len(msg.contacts) > 0

    # ================= RL 推理循环 =================
    def inference_loop(self):
        """
        这个函数以 50Hz 运行。负责打包 Observation 并调用深度学习模型。
        """
        # 1. 构造 Observation 向量 (假装我们在喂给神经网络)
        # 典型的四足 RL 观测值包括：目标指令(3)、IMU姿态(2)、IMU角速度(3)、12关节角度(12)、12关节速度(12)、脚底触地(4)
        obs = [
            self.target_vx, self.target_vy, self.target_wz,
            self.imu_roll, self.imu_pitch,
            self.imu_ang_vel[0], self.imu_ang_vel[1], self.imu_ang_vel[2]
        ]
        obs.extend(self.joint_positions)
        obs.extend(self.joint_velocities)
        obs.extend([
            1.0 if self.contact_states['FL'] else 0.0,
            1.0 if self.contact_states['FR'] else 0.0,
            1.0 if self.contact_states['RL'] else 0.0,
            1.0 if self.contact_states['RR'] else 0.0,
        ])
        
        obs_array = np.array(obs)
        
        # 打印调试信息：神经元是否感受到触地？
        contacts_str = f"FL:{self.contact_states['FL']} FR:{self.contact_states['FR']} RL:{self.contact_states['RL']} RR:{self.contact_states['RR']}"
        
        # 为了不刷屏，只在有非零指令或发生触地变化时打印部分日志
        if abs(self.target_vx) > 0.1 or sum(self.contact_states.values()) > 0:
            self.get_logger().info(f"RL 观测向量维度: {len(obs_array)}, 触地感知: {contacts_str}")
        
        # 2. 调用模型推理 (Dummy)
        # TODO: 这里未来将加载 policy.onnx。目前我们只输出默认的站立角度 (0, 0.8, -1.6)
        actions = []
        for i in range(4):
            actions.extend([0.0, 0.8, -1.6]) # hip, thigh, calf 初始站立姿态
            
        # 3. 将推理结果打向底层物理引擎
        for idx, name in enumerate(self.joint_names):
            msg = Float64()
            msg.data = actions[idx]
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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import math

def euler_from_quaternion(q):
    x, y, z, w = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    return roll_x, pitch_y, 0.0

def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,  2*x*y - 2*z*w,      2*x*z + 2*y*w,      0],
        [2*x*y + 2*z*w,      1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w,      0],
        [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y,  0],
        [0,                  0,                  0,                  1]
    ])

class VMCController(Node):
    def __init__(self):
        super().__init__('vmc_wbc_controller')

        # 机器狗运动学参数 (米)
        self.l1 = 0.06   # hip length
        self.l2 = 0.209  # thigh length
        self.l3 = 0.195  # calf length

        # 关节名称
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]
        
        # 腿的 Y 轴方向符号 (左=1, 右=-1)
        self.side_signs = [1, -1, 1, -1]

        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.roll = 0.0
        self.pitch = 0.0
        self.z = 0.35
        self.vz = 0.0

        # ROS 接口
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom_ground_truth', self.odom_cb, 10)

        self.pubs = {}
        for name in self.joint_names:
            self.pubs[name] = self.create_publisher(Float64, f'/cmd_effort/{name}', 10)

        # 控制频率 200Hz (Python 环境下更稳定)
        self.timer = self.create_timer(1.0/200.0, self.control_loop)

        # VMC PID 参数 (引入积分项：既能保持低刚度消除跳动，又能完美站立在 0.28m 不软趴)
        self.Kp_z = 800.0
        self.Kd_z = 60.0
        self.Ki_z = 1000.0
        self.z_err_sum = 0.0
        
        self.Kp_roll = 100.0
        self.Kd_roll = 10.0
        self.Kp_pitch = 100.0
        self.Kd_pitch = 10.0
        
        # 期望状态
        self.z_des = 0.28

        self.q_imu = [0.0, 0.0, 0.0, 1.0]

    def joint_cb(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]
                self.dq[i] = msg.velocity[idx]

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        
        # 引入一阶低通滤波器 (EMA Filter)，消除 Gazebo 离散差分产生的极高频速度噪声！
        # 这个噪声会被 Kd 放大，导致机器人原地不断微小抽搐（极限环）。
        alpha = 0.15  # 平滑系数，越小越平滑
        self.vz = alpha * msg.twist.twist.linear.z + (1.0 - alpha) * getattr(self, 'vz', 0.0)
        self.wx = alpha * msg.twist.twist.angular.x + (1.0 - alpha) * getattr(self, 'wx', 0.0)
        self.wy = alpha * msg.twist.twist.angular.y + (1.0 - alpha) * getattr(self, 'wy', 0.0)
        
        q = msg.pose.pose.orientation
        self.q_imu = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(self.q_imu)
        self.roll = euler[0]
        self.pitch = euler[1]

    def forward_kinematics(self, q, side_sign):
        q1, q2, q3 = q
        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)
        
        x = -self.l2 * s2 - self.l3 * s23
        y = side_sign * self.l1 * c1 + self.l2 * c2 * s1 + self.l3 * c23 * s1
        z = side_sign * self.l1 * s1 - self.l2 * c2 * c1 - self.l3 * c23 * c1
        return np.array([x, y, z])

    def compute_leg_jacobian(self, q, side_sign):
        """使用有限差分计算高精度数值雅可比矩阵 (Numerical Jacobian)"""
        eps = 1e-5
        J = np.zeros((3, 3))
        for i in range(3):
            q_plus = q.copy()
            q_plus[i] += eps
            q_minus = q.copy()
            q_minus[i] -= eps
            
            p_plus = self.forward_kinematics(q_plus, side_sign)
            p_minus = self.forward_kinematics(q_minus, side_sign)
            
            J[:, i] = (p_plus - p_minus) / (2 * eps)
        return J

    def control_loop(self):
        # 0. 腾空保护 (Flight Phase)
        # 如果机器狗在半空中（例如刚出生时从0.5m落下），VMC会计算出奇怪的受力让腿部乱甩。
        # 因此在高度 > 0.32 时，锁定腿部关节为降落姿态，不计算 VMC。
        if getattr(self, 'z', 0.0) > 0.32:
            tau_cmd = [0.0] * 12
            q_nominal = np.array([0.0, 0.8, -1.6])
            for i in range(4):
                q_leg = self.q[i*3 : i*3+3]
                dq_leg = self.dq[i*3 : i*3+3]
                # 在空中使用高刚度保持姿态，准备冲击
                tau_posture = 20.0 * (q_nominal - q_leg) - 2.0 * dq_leg
                tau_cmd[i*3 : i*3+3] = tau_posture
                
            for idx, name in enumerate(self.joint_names):
                msg = Float64()
                msg.data = float(tau_cmd[idx])
                self.pubs[name].publish(msg)
            return

        # 1. 计算躯干虚拟弹簧恢复力与力矩 (引入积分控制 Z 方向)
        err_z = self.z_des - self.z
        self.z_err_sum += err_z * (1.0 / 200.0)
        self.z_err_sum = max(-0.1, min(0.1, self.z_err_sum)) # 防止积分飞车，最大提供 ±100N 补偿
        
        Fz_total = self.Kp_z * err_z + self.Ki_z * self.z_err_sum - self.Kd_z * self.vz + 8.5 * 9.81
        
        # 增加缺失的角速度阻尼(Kd)，防止无阻尼导致剧烈震荡乱跳！
        wx = getattr(self, 'wx', 0.0)
        wy = getattr(self, 'wy', 0.0)
        Tau_roll = self.Kp_roll * (0.0 - self.roll) - self.Kd_roll * wx
        Tau_pitch = self.Kp_pitch * (0.0 - self.pitch) - self.Kd_pitch * wy

        # 简单均分 Fz，根据 Tau_roll 和 Tau_pitch 进行四个脚的推力分配
        # 假设腿间距: dx (前后) ~ 0.38, dy (左右) ~ 0.20
        dx = 0.38 / 2
        dy = 0.20 / 2
        
        F_legs = []
        for i in range(4):
            # i: 0=FL, 1=FR, 2=RL, 3=RR
            f_z = Fz_total / 4.0
            
            # Roll 分配: 左边腿向上抬(减力)，右边腿向下压(加力)
            if i in [0, 2]: # Left
                f_z += Tau_roll / (2 * dy)
            else:           # Right
                f_z -= Tau_roll / (2 * dy)
                
            # Pitch 分配: 前边腿向下压，后边腿向上抬
            if i in [0, 1]: # Front
                f_z -= Tau_pitch / (2 * dx)
            else:           # Rear
                f_z += Tau_pitch / (2 * dx)
                
            # 限制最小反作用力，防止把腿拉离地面。提高上限以承受冲击。
            f_z = max(5.0, min(300.0, f_z))
            
            # 我们给脚底施加的力 F_foot_world = [0, 0, f_z] (世界坐标系下)
            F_foot_world = np.array([0.0, 0.0, f_z])
            F_legs.append(F_foot_world)

        # 获取机身到世界的旋转矩阵 R_body_to_world
        R_b2w_4x4 = quaternion_matrix(self.q_imu)
        R_w2b = R_b2w_4x4[:3, :3].T

        # 2. 映射为关节力矩 (Tau = J^T * F_body)
        tau_cmd = np.zeros(12)
        for i in range(4):
            q_leg = self.q[i*3 : i*3+3]
            
            # 把世界坐标系下的力转换到机身坐标系下
            F_foot_body = R_w2b @ F_legs[i]
            
            # 由于我们的 F_foot_body 是地面对腿的作用力，关节力矩 = J^T * (-F_foot_body)
            # 等效于支撑身体需要施加的反作用力
            J = self.compute_leg_jacobian(q_leg, self.side_signs[i])
            tau_leg = J.T @ (-F_foot_body)
            
            # 【关键修复】: 增加关节空间弹簧，拉回默认的“半蹲”姿态
            # 防止奇异位形（腿完全伸直）
            # q2 (thigh) 为正时向后弯，q3 (calf) 为负时向前弯
            q_nominal = np.array([0.0, 0.8, -1.6])
            # === 附加关节空间 PD 控制 ===
            q_nominal = np.array([0.0, 0.8, -1.6])
            Kp_joint = 2.0
            Kd_joint = 1.0
            tau_posture = Kp_joint * (q_nominal - q_leg) - Kd_joint * self.dq[i*3 : i*3+3]
            
            tau_leg += tau_posture
            
            tau_cmd[i*3 : i*3+3] = tau_leg

        # 3. 发布力矩
        for idx, name in enumerate(self.joint_names):
            msg = Float64()
            msg.data = float(tau_cmd[idx])
            self.pubs[name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VMCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

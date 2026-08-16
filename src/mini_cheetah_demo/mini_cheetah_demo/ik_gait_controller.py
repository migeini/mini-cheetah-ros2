#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage


class MiniCheetahIKController(Node):
    """
    Mini Cheetah 四足机器人逆运动学 (IK) 兼 Gazebo/RViz 双仿真控制器
    1. 订阅 /cmd_vel 指令 (线性速度 vx, vy, 旋转角速度 wz)
    2. 计算足端摆线/椭圆轨迹 (x, y, z)
    3. 解析单腿逆运动学 (IK) 反算 12 个关节角度
    4. 航位推算 (Odometry Integration) 广播 odom -> base_link TF 树
    5. 发布 /odom 里程计、/robot_path 轨迹及 /model_pose 给 Gazebo 同步实体平移！
    """
    def __init__(self):
        super().__init__('ik_gait_controller')

        # 机器人几何参数 (单位: 米)
        self.l1 = 0.06   # hip 长度
        self.l2 = 0.209  # thigh 大腿长度 (与真实 URDF 同步)
        self.l3 = 0.195  # calf 小腿长度 (与真实 URDF 同步)
        self.h_stand = 0.28  # 站立高度
        self.h_swing = 0.06  # 抬腿高度
        
        # 质心(CoM)补偿：通过极小的微调步伐抵消物理引擎的位移
        self.com_x_offset = 0.0

        # 运动速度状态 (来自于 /cmd_vel，默认静止 0.0 m/s)
        self.target_vx = 0.0  # 前后速度 (m/s)
        self.target_vy = 0.0  # 左右侧移速度 (m/s)
        self.target_wz = 0.0  # 转湾角速度 (rad/s)
        self.march_in_place = False

        # 空间平移里程计状态 (odom -> base_link)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # 运动历史轨迹 Path 对象
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 闭环控制状态
        self.ground_truth_x = 0.0
        self.ground_truth_y = 0.0
        self.ground_truth_yaw = 0.0
        self.locked_x = None
        self.locked_y = None
        
        # 订阅真实物理引擎回传的闭环坐标
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_ground_truth',
            self.odom_ground_truth_callback,
            10
        )

        # 步态周期控制 (设置 30Hz 提高渲染流畅度)
        self.freq = 2.5       # 步态频率 Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # 关节名定义
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        # 创建 /cmd_vel 订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 创建话题发布者
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        
        # 为 12 个关节创建 Float64 指令发布者 (对接 Gazebo PID 控制器)
        self.gz_joint_pubs = []
        for j_name in self.joint_names:
            pub = self.create_publisher(Float64, f'/cmd_pos/{j_name}', 10)
            self.gz_joint_pubs.append(pub)

        # 设置 30Hz 定时器 (0.033 秒周期)
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info('逆运动学兼 Gazebo 实体同步控制器已启动！')

    def cmd_vel_callback(self, msg: Twist):
        """接收速度控制指令"""
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z
        self.march_in_place = (msg.linear.z > 0.5)

    def odom_ground_truth_callback(self, msg: Odometry):
        """接收 Gazebo 的真实坐标，用于闭环抗漂移"""
        pose = msg.pose.pose
        self.ground_truth_x = pose.position.x
        self.ground_truth_y = pose.position.y
        
        # 提取 Yaw 角
        q = pose.orientation
        self.ground_truth_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def leg_ik(self, x, y, z, is_left=True):
        """单腿逆运动学 (IK) 解算器"""
        sign_y = 1.0 if is_left else -1.0
        y_offset = y - sign_y * self.l1
        
        l_yz = math.sqrt(y_offset**2 + z**2)
        th_hip = math.atan2(y_offset, -z)

        d_sq = max(0.001, x**2 + l_yz**2)
        d = math.sqrt(d_sq)

        max_reach = 0.99 * (self.l2 + self.l3)
        if d > max_reach:
            d = max_reach
            d_sq = d**2

        cos_gamma = (self.l2**2 + self.l3**2 - d_sq) / (2.0 * self.l2 * self.l3)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)
        th_calf = gamma - math.pi

        alpha = math.atan2(x, -z)
        cos_beta = (self.l2**2 + d_sq - self.l3**2) / (2.0 * self.l2 * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        th_thigh = alpha + beta

        return th_hip, th_thigh, th_calf

    def timer_callback(self):
        dt = 0.033
        now_time = self.get_clock().now()
        t = now_time.nanoseconds / 1e9 - self.start_time

        # 1. 里程计积分计算坐标
        self.robot_yaw += self.target_wz * dt
        self.robot_x += (self.target_vx * math.cos(self.robot_yaw) - self.target_vy * math.sin(self.robot_yaw)) * dt
        self.robot_y += (self.target_vx * math.sin(self.robot_yaw) + self.target_vy * math.cos(self.robot_yaw)) * dt

        # 2. 广播 odom -> base_link TF 变换
        t_tf = TransformStamped()
        t_tf.header.stamp = now_time.to_msg()
        t_tf.header.frame_id = 'odom'
        t_tf.child_frame_id = 'base_link'
        t_tf.transform.translation.x = self.robot_x
        t_tf.transform.translation.y = self.robot_y
        t_tf.transform.translation.z = 0.0

        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        t_tf.transform.rotation.z = qz
        t_tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t_tf)

        # 3. 发布 /odom 里程计
        odom = Odometry()
        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.target_vx
        odom.twist.twist.linear.y = self.target_vy
        odom.twist.twist.angular.z = self.target_wz
        self.odom_pub.publish(odom)

        # 4. (已移除强制实体位置同步，改由 Gazebo 物理力矩闭环驱动机器狗)

        # 5. 发布 /robot_path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = odom.header.stamp
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = odom.pose.pose
        self.path_msg.poses.append(pose_stamped)
        if len(self.path_msg.poses) > 500:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

        # 6. Trot 混合步态相位计算 (加入双脚触地支撑期以防止原地踏步漂移)
        period = 1.0 / self.freq
        phase = (t % period) / period

        step_x = self.target_vx * 0.15
        step_y = self.target_vy * 0.10
        yaw_step = self.target_wz * 0.10

        is_idle = (abs(self.target_vx) < 0.01 and abs(self.target_vy) < 0.01 and abs(self.target_wz) < 0.01)

        if is_idle and self.march_in_place:
            # 原地踏步：采用超稳定 Walk 步态 (单腿轮流抬起，永远保持三角支撑)
            swing_ratio = 0.20
            legs_info = [
                ('FL', True, phase, 1.0),
                ('FR', False, (phase + 0.5) % 1.0, -1.0),
                ('RL', True, (phase + 0.75) % 1.0, 1.0),
                ('RR', False, (phase + 0.25) % 1.0, -1.0)
            ]
        else:
            # 正常移动：采用 Trot 混合步态 (对角线同步，45% 腾空 + 5% 双支撑过渡)
            swing_ratio = 0.45
            phase_A = phase
            phase_B = (phase + 0.5) % 1.0
            legs_info = [
                ('FL', True, phase_A, 1.0),
                ('FR', False, phase_B, -1.0),
                ('RL', True, phase_B, 1.0),
                ('RR', False, phase_A, -1.0)
            ]

        positions = []
        for name, is_left, p, yaw_sign in legs_info:
            # 加入 CoM 补偿
            base_x = self.com_x_offset
            base_y = self.l1 if is_left else -self.l1
            base_z = -self.h_stand

            if is_idle and not self.march_in_place:
                # 彻底静止状态：四腿均站立在地面
                dx = 0.0
                dy = 0.0
                dz = 0.0
                self.locked_x = None
                self.locked_y = None
            else:
                current_h_swing = self.h_swing
                
                # 动态漂移补偿步长
                comp_step_x = 0.0
                comp_step_y = 0.0
                
                if is_idle and self.march_in_place:
                    # 闭环锁定机制：刚进入踏步时锁定当前真值坐标
                    if self.locked_x is None:
                        self.locked_x = self.ground_truth_x
                        self.locked_y = self.ground_truth_y
                    
                    # 比例反馈控制器 (P 控制器)：将物理位移误差转换为反向补偿步幅
                    Kp = 0.5 
                    err_x_world = self.locked_x - self.ground_truth_x
                    err_y_world = self.locked_y - self.ground_truth_y
                    
                    # 将世界坐标系下的误差旋转到机器人当前局部坐标系
                    yaw = self.ground_truth_yaw
                    err_x_local = err_x_world * math.cos(-yaw) - err_y_world * math.sin(-yaw)
                    err_y_local = err_x_world * math.sin(-yaw) + err_y_world * math.cos(-yaw)
                    
                    # 限制最大补偿步幅，防止抖动
                    comp_step_x = max(min(Kp * err_x_local, 0.06), -0.06)
                    comp_step_y = max(min(Kp * err_y_local, 0.06), -0.06)
                    
                    # 原地踏步时：恢复安全抬腿高度，防止足端在摆动期刮蹭地面导致反向漂移
                    current_h_swing = 0.08
                    if p < swing_ratio and name == 'FL':
                        self.get_logger().info(
                            f"PID Debug: world_err(x={err_x_world:.3f}, y={err_y_world:.3f}), "
                            f"yaw={math.degrees(yaw):.1f}deg, "
                            f"local_err(x={err_x_local:.3f}, y={err_y_local:.3f}), "
                            f"comp(x={comp_step_x:.4f}, y={comp_step_y:.4f})"
                        )
                else:
                    self.locked_x = None
                    self.locked_y = None
                    
                if p < swing_ratio:
                    s = p / swing_ratio
                    actual_step_x = step_x if not is_idle else comp_step_x
                    actual_step_y = step_y if not is_idle else comp_step_y
                    # X轴坐标在URDF中正向对应向后摆腿，因此将实际步长反向以匹配物理常识（前进为正）
                    dx = actual_step_x/2.0 - actual_step_x * (s - math.sin(2.0*math.pi*s)/(2.0*math.pi))
                    dy = -actual_step_y/2.0 + actual_step_y * s + yaw_sign * yaw_step * s
                    dz = current_h_swing * math.sin(math.pi * s)
                else:
                    s = (p - swing_ratio) / (1.0 - swing_ratio)
                    actual_step_x = step_x if not is_idle else comp_step_x
                    actual_step_y = step_y if not is_idle else comp_step_y
                    # 同理反向
                    dx = -actual_step_x/2.0 + actual_step_x * s
                    dy = actual_step_y/2.0 - actual_step_y * s - yaw_sign * yaw_step * s
                    dz = 0.0

            target_x = base_x + dx
            target_y = base_y + dy
            target_z = base_z + dz

            th_hip, th_thigh, th_calf = self.leg_ik(target_x, target_y, target_z, is_left)
            positions.extend([th_hip, th_thigh, th_calf])

        # 7. 发布 /joint_states
        msg = JointState()
        msg.header.stamp = now_time.to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_pub.publish(msg)

        # 8. 发布 12 个关节位置给 Gazebo PID 控制器
        for i, pos in enumerate(positions):
            cmd = Float64()
            cmd.data = float(pos)
            self.gz_joint_pubs[i].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MiniCheetahIKController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

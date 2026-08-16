#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SineJointPublisher(Node):
    """
    Mini Cheetah 12 关节正弦波运动发布者
    定时向 '/joint_states' 话题发布所有 12 个关节的位置数据
    """

    def __init__(self):
        super().__init__('sine_joint_publisher')

        # 创建 /joint_states 话题发布者
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # 设置 50Hz 定时器 (0.02秒周期)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # 定义 12 个关节名称
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info('12 关节正弦运动发布节点已启动！')

    def timer_callback(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        t = now_sec - self.start_time

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # 计算正弦步态位置 (模拟四足对角摆动 Trot Gait)
        freq = 3.0  # 运动频率 (Hz)

        # FL 与 RR 同相位，FR 与 RL 反相位 (相位差 pi)
        phase_fl_rr = math.sin(freq * t)
        phase_fr_rl = math.sin(freq * t + math.pi)

        # FL (左前)
        fl_hip = 0.15 * math.sin(freq * t)
        fl_thigh = 0.4 + 0.4 * phase_fl_rr
        fl_calf = -1.0 - 0.5 * phase_fl_rr

        # FR (右前)
        fr_hip = -0.15 * math.sin(freq * t)
        fr_thigh = 0.4 + 0.4 * phase_fr_rl
        fr_calf = -1.0 - 0.5 * phase_fr_rl

        # RL (左后)
        rl_hip = 0.15 * math.sin(freq * t)
        rl_thigh = 0.4 + 0.4 * phase_fr_rl
        rl_calf = -1.0 - 0.5 * phase_fr_rl

        # RR (右后)
        rr_hip = -0.15 * math.sin(freq * t)
        rr_thigh = 0.4 + 0.4 * phase_fl_rr
        rr_calf = -1.0 - 0.5 * phase_fl_rr

        msg.position = [
            fl_hip, fl_thigh, fl_calf,
            fr_hip, fr_thigh, fr_calf,
            rl_hip, rl_thigh, rl_calf,
            rr_hip, rr_thigh, rr_calf
        ]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineJointPublisher()
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

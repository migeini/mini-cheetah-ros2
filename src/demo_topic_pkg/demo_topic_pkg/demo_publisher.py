#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopicPublisher(Node):
    """
    ROS 2 话题发布者节点
    作用：定时向 '/chatter' 话题发送字符串消息
    """
    def __init__(self):
        # 初始化节点名称为 'demo_publisher_node'
        super().__init__('demo_publisher_node')
        
        # 创建发布者对象
        # 参数 1: 消息类型 (String)
        # 参数 2: 话题名称 ('chatter')
        # 参数 3: QoS 消息队列深度 (10)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # 创建定时器，每 1.0 秒触发一次回调函数 timer_callback
        timer_period = 1.0  # 单位：秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 消息计数器
        self.count_ = 0
        self.get_logger().info('发布者节点 [demo_publisher_node] 已启动！')

    def timer_callback(self):
        """定时器回调函数：构造并发布消息"""
        msg = String()
        msg.data = f'Hello ROS 2! 计数器: {self.count_}'
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 在终端打印日志
        self.get_logger().info(f'发送消息: "{msg.data}"')
        self.count_ += 1


def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)
    
    # 实例化节点对象
    node = TopicPublisher()
    
    try:
        # 保持节点运行，等待回调函数被触发
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 显式销毁节点并关闭 rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopicSubscriber(Node):
    """
    ROS 2 话题订阅者节点
    作用：订阅 '/chatter' 话题并接收处理消息
    """
    def __init__(self):
        # 初始化节点名称为 'demo_subscriber_node'
        super().__init__('demo_subscriber_node')
        
        # 创建订阅者对象
        # 参数 1: 消息类型 (String)
        # 参数 2: 话题名称 ('chatter')
        # 参数 3: 回调函数 (listener_callback)
        # 参数 4: QoS 消息队列深度 (10)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        # 防止订阅者引用被垃圾回收
        self.subscription
        self.get_logger().info('订阅者节点 [demo_subscriber_node] 已启动，正在等待消息...')

    def listener_callback(self, msg):
        """接收到消息时的回调函数"""
        self.get_logger().info(f'收到消息: "{msg.data}"')


def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)
    
    # 实例化节点对象
    node = TopicSubscriber()
    
    try:
        # 保持节点运行，响应回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭 rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

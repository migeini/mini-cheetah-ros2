import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    ROS 2 Launch 启动脚本
    作用：同时一键启动发布者节点和订阅者节点
    """
    # 定义发布者节点启动项
    publisher_node = Node(
        package='demo_topic_pkg',      # 功能包名称
        executable='publisher_node',   # 可执行文件名称 (定义在 setup.py 中)
        name='demo_publisher_node',    # 节点名
        output='screen'                # 将日志信息直接输出到控制台终端
    )

    # 定义订阅者节点启动项
    subscriber_node = Node(
        package='demo_topic_pkg',      # 功能包名称
        executable='subscriber_node',  # 可执行文件名称 (定义在 setup.py 中)
        name='demo_subscriber_node',   # 节点名
        output='screen'                # 将日志信息直接输出到控制台终端
    )

    # 返回 LaunchDescription 对象，包含待启动的所有节点动作
    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])

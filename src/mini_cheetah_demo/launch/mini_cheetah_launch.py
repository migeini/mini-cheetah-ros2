import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mini_cheetah_demo')

    # Xacro 文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'mini_cheetah.urdf.xacro')
    # RViz 配置文件路径
    rviz_config_file = os.path.join(pkg_share, 'config', 'mini_cheetah.rviz')

    # 解析 xacro 生成 robot_description 参数 (使用 ParameterValue 防止字符串被按 YAML 解析)
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # 1. 启动 robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. 启动自定义 12 关节正弦波运动发布者节点
    sine_joint_publisher_node = Node(
        package='mini_cheetah_demo',
        executable='sine_joint_publisher',
        name='sine_joint_publisher',
        output='screen'
    )

    # 3. 启动 RViz2 节点
    rviz_env = dict(os.environ)
    rviz_env['QT_QPA_PLATFORM'] = 'xcb'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        env=rviz_env,
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        sine_joint_publisher_node,
        rviz_node
    ])

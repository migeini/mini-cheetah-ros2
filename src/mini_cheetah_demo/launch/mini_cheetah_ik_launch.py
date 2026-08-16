import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mini_cheetah_demo')

    # Xacro 与 RViz 配置文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'mini_cheetah.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mini_cheetah.rviz')

    # 描述参数解析
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # 1. robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. 逆运动学 (IK) 兼速度控制器节点
    ik_gait_controller_node = Node(
        package='mini_cheetah_demo',
        executable='ik_gait_controller',
        name='ik_gait_controller',
        output='screen'
    )

    # 3. RViz2 节点
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
        ik_gait_controller_node,
        rviz_node
    ])

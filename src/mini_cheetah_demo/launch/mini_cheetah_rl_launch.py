import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mini_cheetah_demo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # 文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'mini_cheetah.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'mini_cheetah_world.sdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mini_cheetah.rviz')

    # URDF 解析
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # 1. 启动 Gazebo 物理仿真世界 (gz_sim)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # 2. 将机器人 URDF 模型派生加载到 Gazebo 物理世界中 (-z 0.35 悬空放置)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mini_cheetah',
            '-z', '0.35'
        ],
        output='screen'
    )

    # 3. 配置 ROS 2 与 Gazebo 物理时钟/话题双向桥接器 (ros_gz_bridge)
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 4. robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 5. 逆运动学 (IK) 兼航位推算控制器
    ik_gait_controller_node = Node(
        package='mini_cheetah_demo',
        executable='ik_gait_controller',
        name='ik_gait_controller',
        output='screen'
    )

    # 6. RViz2 节点
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
        gazebo_sim,
        spawn_robot,
        bridge_node,
        robot_state_publisher_node,
        rviz_node
    ])

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径设定
    pkg_project_bringup = get_package_share_directory('rotors_swift_gazebo')
    pkg_project_gazebo = get_package_share_directory('rotors_swift_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. 启动 Gazebo 仿真器
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                'swift_pico_world.sdf'
            ]),
            ' -r'  # 自动运行仿真
        ]}.items(),
    )

    # 3. ROS-GZ 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'swift_pico_bridge.yaml'),
        }],
        output='screen'
    )

    # 4. 静态坐标变换 (TF)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0', '0', '0', '0', 'swift_pico/base_link', 'swift_pico/camera_link/rgbd_camera']
    )
    
    tf_base_to_altimeter = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # XYZ(0, 0, -0.15), RPY(0, 1.570796, 0) -> 把Pitch=1.57表示向下
        arguments=['0', '0', '-0.15', '0', '1.570796', '0', 'swift_pico/base_link', 'swift_pico/altimeter_link']
    )

    # [新增] 2D 激光雷达 TF
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'swift_pico/base_link', 'swift_pico/swift_pico/laser_link/laser_sensor']
    )

    # 5. [替换] SLAM 节点 (Cartographer 2D LiDAR SLAM)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_project_bringup, 'config'),
            '-configuration_basename', 'cartographer.lua'
        ],
    )

    # (可选) Cartographer 栅格地图生成器 (用于 RViz 实时建图显示)
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # 6. 控制节点
    drone_controller_sim = Node(
        package='softdrone_controller',
        executable='drone_controller_sim',
        name='drone_controller_sim',
        output='screen',
    )

    position_controller = Node(
        package='softdrone_controller',
        executable='position_controller',
        name='position_controller',
        output='screen',
        parameters=[{
            'use_ground_truth': False # 现在代表使用 Cartographer TF 控制模式
        }]
    )

    position_cmd_node = Node(
        package='softdrone_controller',
        executable='position_cmd',
        name='position_cmd',
        output='screen',
    )
    
    # 7. 键盘控制
    keyboard_node = Node(
        package='softdrone_controller',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        tf_base_to_camera,
        tf_base_to_altimeter,
        tf_base_to_laser, # 新增
        cartographer_node, # 替换为 Cartographer
        cartographer_occupancy_grid_node, # 新增地图显示
        drone_controller_sim,
        position_controller,
        position_cmd_node,
        #keyboard_node      # 弹出键盘控制窗口
    ])

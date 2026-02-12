#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 路徑設定
    pkg_project_bringup = get_package_share_directory('rotors_swift_gazebo')
    pkg_project_gazebo = get_package_share_directory('rotors_swift_gazebo')
    pkg_project_description = get_package_share_directory('rotors_swift_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 加載 SDF 模型
    sdf_file = os.path.join(pkg_project_description, 'models', 'swift_pico', 'swift_pico.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 啟動 Gazebo 仿真器
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'swift_pico_world.sdf -r'
        ])}.items(),
    )

    # ROS-GZ 橋接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'swift_pico_bridge.yaml'),
        }],
        output='screen'
    )

    # 你的三個節點（移除錯誤的 parameters=）
    drone_controller_sim = Node(
        package='softdrone_controller',
        executable='drone_controller_sim',
        name='drone_controller_sim',
        output='screen',
        # 刪除這段！不需要傳 .py 文件，因為代碼裡已經 import 了
        # parameters=[...]
    )

    position_controller = Node(
        package='softdrone_controller',
        executable='position_controller',
        name='position_controller',
        output='screen',
        # 同樣刪除
        # parameters=[...]
    )

    position_cmd_node = Node(
        package='softdrone_controller',
        executable='position_cmd',
        name='position_cmd',
        output='screen',
        # 同樣刪除
        # parameters=[...]
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        drone_controller_sim,
        position_controller,
        position_cmd_node
    ])

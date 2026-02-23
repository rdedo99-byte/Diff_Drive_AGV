#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Pacchetti
    bringup_pkg_share = FindPackageShare('bringup')
    diffdrive_pkg_share = FindPackageShare('diffdrive_control')
    lidar_pkg_share = FindPackageShare('sllidar_ros2')
    slam_toolbox_pkg_share = FindPackageShare('slam_toolbox') 
    nav2_bringup_pkg_share = FindPackageShare('navigation') 
    
    # Config files
    xacro_file = PathJoinSubstitution([bringup_pkg_share, 'config', 'robot_description', 'diff_drive.urdf.xacro'])
    controller_yaml = PathJoinSubstitution([bringup_pkg_share, 'config', 'ros2_control', 'diff_drive_controller.yaml'])
    slam_params_file = PathJoinSubstitution([bringup_pkg_share, 'config', 'slam', 'slam_params_localization.yaml'])
    nav2_params_file = PathJoinSubstitution([bringup_pkg_share, 'config', 'navigation', 'nav2_params_mine.yaml'])
    ekf_params_file = PathJoinSubstitution([bringup_pkg_share, 'config', 'navigation', 'ekf.yaml']) 
    robot_description_content = Command(['xacro ', xacro_file])
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    imu_node = Node(
        package='imu_bno055',
        executable='bno055_i2c_node',
        name='imu_node',
        namespace='imu',
        parameters=[{
            'i2c_address': 0x28,
            'frame_id': 'imu',
            'operation_mode': 0x0C 
        }],
        output='screen'
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
    )

    # Hardware Interface
    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([diffdrive_pkg_share, 'launch', 'diff_drive.launch.py'])
        ]),
        launch_arguments={
            'controller_params_file': controller_yaml, 
            'robot_description_content': robot_description_content
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([lidar_pkg_share, 'launch', 'sllidar_a1_launch.py'])
        ])
    )

    # SLAM localization 
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                slam_toolbox_pkg_share,
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_pkg_share, 'launch', 'navigation.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'True', 
        }.items()
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False'),
        # 1. TF 
        TimerAction(period=1.0, actions=[robot_state_publisher_node]), 
        # 2. Hardware 
        TimerAction(period=3.0, actions=[diff_drive_launch]),
        TimerAction(period=4.0, actions=[imu_node]),
        TimerAction(period=5.0, actions=[lidar_launch]),   
        # 3. EKF 
        TimerAction(period=8.0, actions=[ekf_node]),
        # 4. SLAM 
        TimerAction(period=12.0, actions=[slam_launch]),
        # 5. Nav2
        TimerAction(period=18.0, actions=[nav2_launch]),
    ])
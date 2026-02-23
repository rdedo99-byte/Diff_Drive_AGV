import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. ARGOMENTI
    controller_params_file = LaunchConfiguration('controller_params_file')
    controller_params_file_arg = DeclareLaunchArgument(
        'controller_params_file',
        description='Path to the ROS 2 Control controller parameters YAML file.'
    )
    
    robot_description_content = LaunchConfiguration('robot_description_content')
    robot_description_content_arg = DeclareLaunchArgument(
        'robot_description_content',
        default_value='<robot name="temp_robot"/>', 
        description='Content of the robot description (URDF/XACRO) needed by ros2_control_node.'
    )

    # 2. NODO ROS 2 CONTROL
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_content}, 
            controller_params_file 
        ],
        output="screen",
    )
    
    # 3. SPAWNER DEI CONTROLLER (Standard, senza timeout extra)
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "-c", "/controller_manager"
        ],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller", 
            "-c", "/controller_manager"
        ],
    )

    return LaunchDescription([
        controller_params_file_arg,
        robot_description_content_arg,
        
        # Lancia tutto immediatamente
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
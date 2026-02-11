#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robotics-core-templates')

    # 1. Define Launch Arguments
    drive_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mecanum',
        description='Drive mode: mecanum or diff'
    )

    # 2. Paths to YAML configs
    mecanum_config = os.path.join(pkg_share, 'config', 'mecanum_params.yaml')
    diff_config = os.path.join(pkg_share, 'config', 'diff_drive_params.yaml')

    # 3. Mecanum Node (only runs if mode := 'mecanum')
    mecanum_node = Node(
        condition=LaunchConfigurationEquals('mode', 'mecanum'),
        package='robotics-core-templates',
        executable='mecanum_drive_node',
        name='mecanum_controller',
        output='screen',
        parameters=[mecanum_config]
    )

    # 4. Differential Node (only runs if mode := 'diff')
    diff_node = Node(
        condition=LaunchConfigurationEquals('mode', 'diff'),
        package='robotics-core-templates',
        executable='diff_drive_node',
        name='diff_drive_node', # Matches your C++ Node Name
        output='screen',
        parameters=[diff_config]
    )

    return LaunchDescription([
        drive_mode_arg,
        mecanum_node,
        diff_node
    ])
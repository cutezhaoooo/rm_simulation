#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 机器人描述
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('pb_rm_simulation'), 'urdf', 'simulation_waking_robot.xacro')])

    # TF发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    
    return ld
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb_rm_simulation')

    # Specify xacro path
    default_robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('pb_rm_simulation'), 'urdf', 'simulation_waking_robot.xacro')])

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    robot_description = LaunchConfiguration('robot_description')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',  # 不使用仿真时间
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    # Joint State Publisher (发布关节状态)
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'source_list': ['joint_states']  # 可以接收实际的关节状态话题
        }],
        output='screen'
    )

    # Robot State Publisher (发布TF变换)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 50.0,  # 发布频率50Hz
            'frame_prefix': ''  # 默认情况下不添加前缀
        }],
        output='screen'
    )

    # RViz (可选，用于可视化TF变换)
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 如果需要发布静态关节状态（例如所有关节都设置为0），可以使用以下节点
    start_static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_description_cmd)

    # 添加TF变换相关节点
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    
    # 可选：添加静态变换发布器
    # ld.add_action(start_static_transform_publisher_cmd)
    
    # 可选：启动RViz进行可视化
    ld.add_action(start_rviz_cmd)

    return ld
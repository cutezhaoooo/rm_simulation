import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()
    
    # 获取包路径
    try:
        pb_rm_simulation_pkg = get_package_share_directory('pb_rm_simulation')
        fast_lio_pkg = get_package_share_directory('fast_lio')
        terrain_analysis_pkg = get_package_share_directory('terrain_analysis')
        local_planner_pkg = get_package_share_directory('local_planner')
    except Exception as e:
        print(f"Error finding packages: {e}")
        return ld
    
    # 1. 启动 rm_simulation
    rm_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pb_rm_simulation_pkg, 'launch', 'rm_simulation.launch.py')
        )
    )
    ld.add_action(rm_simulation_launch)
    
    # 2. 启动 fast_lio mapping
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'mid360.yaml'}.items()
    )
    ld.add_action(fast_lio_launch)
    
    # 3. 启动 publish_body_to_livox_tf 节点
    tf_node = Node(
        package='publish_body_to_livox_tf',
        executable='map_odom_baselink_livox',
        name='map_odom_baselink_livox',
        output='screen'
    )
    ld.add_action(tf_node)
    
    # 4. 启动 vehicleSimulator
    vehicle_simulator_node = Node(
        package='vehicle_simulator',
        executable='vehicleSimulator',
        name='vehicleSimulator',
        output='screen'
    )
    ld.add_action(vehicle_simulator_node)
    
    # 5. 使用ExecuteProcess启动XML格式的terrain_analysis.launch
    terrain_analysis_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'terrain_analysis', 'terrain_analysis.launch'],
        output='screen'
    )
    ld.add_action(terrain_analysis_launch)
    
    # 6. 启动 local_planner
    local_planner_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'local_planner', 'local_planner.launch'],
        output='screen'
    )
    ld.add_action(local_planner_launch)

    lqr_controll_node = Node(
        package='lqr_controller',
        executable='lqr_controller_node',
        name='lqr_controller_node',
        output='screen'
    )
    # ld.add_action(lqr_controll_node)
    
    return ld
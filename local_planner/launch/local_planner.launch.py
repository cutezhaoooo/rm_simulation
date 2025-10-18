from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # === 声明可配置参数（替代 <arg .../>） ===
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    twoWayDrive = LaunchConfiguration('twoWayDrive')
    maxSpeed = LaunchConfiguration('maxSpeed')
    autonomyMode = LaunchConfiguration('autonomyMode')
    autonomySpeed = LaunchConfiguration('autonomySpeed')
    joyToSpeedDelay = LaunchConfiguration('joyToSpeedDelay')
    goalX = LaunchConfiguration('goalX')
    goalY = LaunchConfiguration('goalY')

    # === 声明默认值 ===
    declare_args = [
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0'),
        DeclareLaunchArgument('twoWayDrive', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='2.0'),
        DeclareLaunchArgument('autonomyMode', default_value='true'),
        DeclareLaunchArgument('autonomySpeed', default_value='2.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),
        DeclareLaunchArgument('goalX', default_value='0.0'),
        DeclareLaunchArgument('goalY', default_value='0.0'),
    ]

    # === 自动获取路径 ===
    path_folder = get_package_share_directory('local_planner') + '/paths'
    print('==================================================')
    print(path_folder)

    # === localPlanner 节点 ===
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder': path_folder,
            'vehicleLength': 0.6,
            'vehicleWidth': 0.6,
            'sensorOffsetX': sensorOffsetX,
            'sensorOffsetY': sensorOffsetY,
            'twoWayDrive': twoWayDrive,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': True,
            'checkObstacle': True,
            'checkRotObstacle': False,
            'adjacentRange': 4.25,
            'obstacleHeightThre': 0.15,
            'groundHeightThre': 0.1,
            'costHeightThre': 0.1,
            'costScore': 0.02,
            'useCost': False,
            'pointPerPathThre': 2,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'maxSpeed': maxSpeed,
            'dirWeight': 0.02,
            'dirThre': 90.0,
            'dirToVehicle': False,
            'pathScale': 1.25,
            'minPathScale': 0.75,
            'pathScaleStep': 0.25,
            'pathScaleBySpeed': True,
            'minPathRange': 1.0,
            'pathRangeStep': 0.5,
            'pathRangeBySpeed': True,
            'pathCropByGoal': True,
            'autonomyMode': autonomyMode,
            'autonomySpeed': autonomySpeed,
            'joyToSpeedDelay': joyToSpeedDelay,
            'joyToCheckObstacleDelay': 5.0,
            'goalClearRange': 0.5,
            'goalX': goalX,
            'goalY': goalY
        }]
    )

    # === pathFollower 节点 ===
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX': sensorOffsetX,
            'sensorOffsetY': sensorOffsetY,
            'pubSkipNum': 1,
            'twoWayDrive': twoWayDrive,
            'lookAheadDis': 0.5,
            'yawRateGain': 7.5,
            'stopYawRateGain': 7.5,
            'maxYawRate': 90.0,
            'maxSpeed': maxSpeed,
            'maxAccel': 2.5,
            'switchTimeThre': 1.0,
            'dirDiffThre': 0.1,
            'stopDisThre': 0.2,
            'slowDwnDisThre': 0.85,
            'useInclRateToSlow': False,
            'inclRateThre': 120.0,
            'slowRate1': 0.25,
            'slowRate2': 0.5,
            'slowTime1': 2.0,
            'slowTime2': 2.0,
            'useInclToStop': False,
            'inclThre': 45.0,
            'stopTime': 5.0,
            'noRotAtStop': False,
            'noRotAtGoal': True,
            'autonomyMode': autonomyMode,
            'autonomySpeed': autonomySpeed,
            'joyToSpeedDelay': joyToSpeedDelay,
        }]
    )

    # === LaunchDescription ===
    return LaunchDescription(declare_args + [
        local_planner_node,
        path_follower_node
    ])

fastlio保存地图
ros2 service call /map_save std_srvs/srv/Trigger 

<!-- 运动控制节点 -->
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_chassis

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

在/home/z/rm_simulation/src/far_planner/src/far_planner/config 中修改default.yaml的world_frame为camera init
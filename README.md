fastlio保存地图
ros2 service call /map_save std_srvs/srv/Trigger 

<!-- 运动控制节点 -->
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_chassis

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

<!-- 保存的地图是.pcd文件 需要转换为.ply文件 -->
pcl_pcd2ply input.pcd output.ply


在/home/z/rm_simulation/src/far_planner/src/far_planner/config 中修改default.yaml的world_frame为camera init


拉取仓库的命令
git clone --recursive https://github.com/cutezhaoooo/rm_simulation.git

地图保存在了FAST_LIO/PCD文件夹下面，所以visualization_tools launch 文件的 mapFile 参数需要更改

编译成debug格式
colcon build --packages-select local_planner --cmake-args -DCMAKE_BUILD_TYPE=Debug

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


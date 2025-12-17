# RM Simulation - 基于北极熊战队的开源项目

感谢北极熊战队的开源！

## 项目简介

这是一个机器人仿真项目，集成了建图、导航和运动控制功能。

## 安装与编译

### 克隆仓库
```bash
git clone --recursive https://github.com/cutezhaoooo/rm_simulation.git
cd rm_simulation
```

### 编译项目
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 使用方法

### 建图模式
1. 启动建图节点：
```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

2. 保存地图：
```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### 导航模式
一键启动导航：
```bash
ros2 launch bringup nav.launch.py
```

### 运动控制
启动键盘控制：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

ros2 service call /trigger_service std_srvs/srv/SetBool "{data: true}" 

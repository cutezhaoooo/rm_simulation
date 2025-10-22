
---

# RM Simulation & FAST-LIO Mapping System

This repository contains a ROS 2-based simulation and mapping framework using FAST-LIO for LiDAR-inertial odometry, along with tools for map saving, conversion, and path planning.

---

## 📦 目录

- [1. 环境依赖](#1-环境依赖)
- [2. 项目克隆](#2-项目克隆)
- [3. 编译说明](#3-编译说明)
- [4. 快速启动：建图流程](#4-快速启动建图流程)
- [5. 地图保存与转换](#5-地图保存与转换)
- [6. 配置修改](#6-配置修改)
- [7. 运动控制](#8-运动控制)
- [8. 编译选项（Debug/Release）](#9-编译选项debugrelease)

---

## 1. 环境依赖

确保已安装以下软件包：

/livox/lidar/pointcloud 
pointCloud2

- ROS 2 (推荐 Humble)
- PCL (Point Cloud Library)
- FAST-LIO
---

## 2. 项目克隆

使用递归克隆以包含所有子模块（如 FAST-LIO）：

```bash
git clone --recursive https://github.com/cutezhaoooo/rm_simulation.git
```

若已克隆，请更新子模块：
```bash
git submodule update --init --recursive
```

---

## 3. 编译说明

进入工作空间并编译：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 单独编译某个包（如 `local_planner`）为 Debug 模式：

```bash
colcon build --packages-select local_planner --cmake-args -DCMAKE_BUILD_TYPE=Debug
```


---

## 4. 快速启动：建图流程

1. 启动 FAST-LIO 建图节点（使用 mid360 配置）：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

2. 使用键盘控制机器人运动（可选）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_chassis
```

3. 建图完成后，调用服务保存地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

> ✅ 地图默认保存在 `FAST_LIO/PCD/` 目录下，文件名为 `.pcd`。

---

## 5. 地图保存与转换

### 保存地图

通过服务触发地图保存：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

---

## 6. 配置修改

修改路径规划器的默认坐标系（`world_frame`）：

🔧 文件路径：
```
/home/z/rm_simulation/src/far_planner/src/far_planner/config/default.yaml
```


---

## 7. 运动控制

使用键盘控制底盘运动：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_chassis
```

> 📌 注意：确保底盘驱动节点订阅了 `/cmd_vel_chassis` 主题。

---

## 8. 编译选项（Debug/Release）

| 模式     | 用途                     | 命令 |
|----------|--------------------------|------|
| **Release** | 正常运行，性能优化       | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| **Debug**   | 调试、断点、日志分析     | `colcon build --packages-select local_planner --cmake-args -DCMAKE_BUILD_TYPE=Debug` |

---

## 📝 备注

- 所有生成的地图文件存储在 `FAST_LIO/PCD/` 文件夹中。
- 若使用自定义 LiDAR，请修改 `config_file:=xxx.yaml` 对应的配置。

---

## 📬 反馈与贡献

欢迎提交 Issue 或 Pull Request！

Maintainer: [cutezhaoooo](https://github.com/cutezhaoooo)

---
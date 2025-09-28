cmake_minimum_required(VERSION 3.8)
project(terrain_analysis)

# 设置构建类型为包含调试信息的模式
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)  # 推荐：优化 + 调试符号
  # 或者 set(CMAKE_BUILD_TYPE Debug)   # 完全无优化，最大调试信息
endif()

# 启用调试符号（确保已包含）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(terrainAnalysis src/terrainAnalysis.cpp)
ament_target_dependencies(terrainAnalysis rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_conversions pcl_ros)

install(TARGETS
  terrainAnalysis
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  # launch
  # paths
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

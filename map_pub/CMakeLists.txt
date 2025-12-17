cmake_minimum_required(VERSION 3.8)
project(map_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# 添加可执行文件
add_executable(map_pub src/map_pub.cpp)

# 添加依赖
ament_target_dependencies(map_pub
  rclcpp
  sensor_msgs
  pcl_conversions
)

# 链接PCL库
target_link_libraries(map_pub ${PCL_LIBRARIES})

# 安装
install(TARGETS
  map_pub
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
# ament_export_dependencies(
#   rclcpp
#   sensor_msgs
#   pcl_conversions
# )

# ament_package()

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

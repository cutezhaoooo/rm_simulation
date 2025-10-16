cmake_minimum_required(VERSION 3.8)
project(publish_body_to_livox_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(publish_body_to_livox src/publish_body_to_livox.cpp)
add_executable(map_odom_baselink_livox src/publish_body_to_livox.cpp)
ament_target_dependencies(publish_body_to_livox
  rclcpp
  tf2_ros
  geometry_msgs
)
ament_target_dependencies(map_odom_baselink_livox
  rclcpp
  tf2_ros
  geometry_msgs
)


# target_link_libraries(publish_body_to_livox_tf
#   ${tf2_ros_LIBRARIES}
# )

# 安装
install(TARGETS
  publish_body_to_livox
  map_odom_baselink_livox
  DESTINATION lib/${PROJECT_NAME}
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

cmake_minimum_required(VERSION 3.11)
project(rm_decision_cpp)

# 设置 CMake 策略以消除警告
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

set(ros_dependencies
  rclcpp
  auto_aim_interfaces
  rm_interfaces
  nav_msgs
  nav2_msgs
  nav2_util
  nav2_behavior_tree
  geometry_msgs
  behaviortree_cpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  PCL
)

add_library(${PROJECT_NAME} SHARED
  src/custume_types.cpp
  src/behaviors/attack.cpp
  src/behaviors/nav2pose.cpp
  src/behaviors/anti_autoaim.cpp
  src/behaviors/spin.cpp
  src/behaviors/topics2blackboard.cpp
  src/behaviors/sentry_cmd.cpp
  src/behaviors/control_gimbal.cpp
  src/behaviors/align_chassis.cpp
)

add_library(goal_updater_lib SHARED /root/nav_tutorial_ws/src/SCURM_Nav_Tutorial/follow/src/goal_updater.cpp)

ament_target_dependencies(goal_updater_lib
  rclcpp
  behaviortree_cpp
  geometry_msgs
  auto_aim_interfaces
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})

add_executable(tree_exec_node src/tree_exec_node.cpp
  src/custume_types.cpp
  src/behaviors/attack.cpp
  src/behaviors/nav2pose.cpp
  src/behaviors/anti_autoaim.cpp
  src/behaviors/spin.cpp
  src/behaviors/topics2blackboard.cpp
  src/behaviors/sentry_cmd.cpp
  src/behaviors/control_gimbal.cpp
  src/behaviors/align_chassis.cpp
)

target_link_libraries(tree_exec_node goal_updater_lib)

ament_target_dependencies(tree_exec_node ${ros_dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tree_exec_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
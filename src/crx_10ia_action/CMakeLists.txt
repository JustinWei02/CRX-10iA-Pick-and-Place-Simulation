cmake_minimum_required(VERSION 3.8)
project(crx_10ia_action)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(block_pick_place_node src/block_pick_place_node.cpp)

# Link dependencies
ament_target_dependencies(block_pick_place_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS block_pick_place_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
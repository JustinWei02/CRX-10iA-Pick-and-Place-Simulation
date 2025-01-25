cmake_minimum_required(VERSION 3.8)
project(target_localization)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)


# Add the executable
add_executable(target_localization src/target_localization.cpp)

# Link libraries and dependencies
ament_target_dependencies(target_localization
rclcpp
sensor_msgs
geometry_msgs
tf2_ros
pcl_conversions
PCL
tf2_geometry_msgs
tf2
)

# Install the executable
install(TARGETS target_localization
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()

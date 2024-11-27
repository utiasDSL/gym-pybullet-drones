cmake_minimum_required(VERSION 3.8)
project(rrt_path_finder)

# Add compiler flags for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Manually specify the path to custom_interface_gym if it’s not automatically found
list(APPEND CMAKE_PREFIX_PATH /home/astik/gym-pybullet-drones/ros2/install/custom_interface_gym/share/custom_interface_gym/cmake)
find_package(custom_interface_gym REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add source files
set(SOURCES
  src/corridor_finder.cpp
  src/kdtree.c
  src/rrt_node.cpp
)

# Declare the executable
add_executable(rrt_node ${SOURCES})

# Link dependencies
ament_target_dependencies(rrt_node
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  std_msgs
  custom_interface_gym
)

# Link the PCL libraries
target_link_libraries(rrt_node ${PCL_LIBRARIES})

# Ensure C++17 and C99 standards
target_compile_features(rrt_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS rrt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  custom_interface_gym
)

# This is necessary for building the package
ament_package()

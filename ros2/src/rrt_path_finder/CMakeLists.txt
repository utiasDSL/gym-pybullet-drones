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

# Manually specify the path to custom_interface_gym
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX}/custom_interface_gym/share/custom_interface_gym/cmake)
find_package(custom_interface_gym REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/custom_interface_gym/include
  ${CMAKE_INSTALL_PREFIX}/custom_interface_gym/include/custom_interface_gym
)

# Add source files for rrt_node
set(SOURCES_RRT_NODE
  src/corridor_finder.cpp
  src/kdtree.c
  src/rrt_node.cpp
)

# Declare the executable for rrt_node
add_executable(rrt_node ${SOURCES_RRT_NODE})

# Link dependencies for rrt_node
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

# Link the PCL libraries for rrt_node
target_link_libraries(rrt_node ${PCL_LIBRARIES})

# Ensure C++17 and C99 standards for rrt_node
target_compile_features(rrt_node PUBLIC c_std_99 cxx_std_17)

# Add source files for trajectory_server
set(SOURCES_TRAJECTORY_SERVER
  src/trajectory_server.cpp
)

# Declare the executable for trajectory_server
add_executable(trajectory_server ${SOURCES_TRAJECTORY_SERVER})

# Link dependencies for trajectory_server (SAME AS rrt_node)
ament_target_dependencies(trajectory_server
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

# Link the PCL libraries for trajectory_server (SAME AS rrt_node)
target_link_libraries(trajectory_server ${PCL_LIBRARIES})

# Ensure C++17 and C99 standards for trajectory_server (SAME AS rrt_node)
target_compile_features(trajectory_server PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS rrt_node trajectory_server
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

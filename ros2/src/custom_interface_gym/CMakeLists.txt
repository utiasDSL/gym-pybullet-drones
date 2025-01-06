cmake_minimum_required(VERSION 3.8)
project(custom_interface_gym)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # For message and service generation
find_package(geometry_msgs REQUIRED)  # Include dependencies used in messages
find_package(std_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# Define custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UavControl.srv"  # Define your custom service file here
  "msg/TrajMsg.msg"
  "msg/DesTrajectoryNew.msg"
  "msg/DesTrajectory.msg"         # Define your custom message file here
  DEPENDENCIES geometry_msgs std_msgs geographic_msgs  # Specify dependencies
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()

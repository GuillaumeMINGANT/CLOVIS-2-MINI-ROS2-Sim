cmake_minimum_required(VERSION 3.5)
project(robot_test)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Add executables
add_executable(standard_joint_test_node src/standard_joint_test.cpp)
ament_target_dependencies(standard_joint_test_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

# Install executables
install(TARGETS
  standard_joint_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)



# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Add executables
add_executable(moveit_control_simple src/moveit_control_simple.cpp)
add_executable(end_pose_node src/end_pose_node.cpp)
add_executable(moveit_control_gui src/moveit_control_gui.cpp)

# Link the required dependencies for moveit_control_gui
ament_target_dependencies(moveit_control_gui
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    tf2_ros
)

# Link Qt5Widgets using target_link_libraries
target_link_libraries(moveit_control_gui
  Qt5::Widgets
)

# Link libraries for moveit_control_simple
ament_target_dependencies(moveit_control_simple
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Link libraries for end_pose_node
ament_target_dependencies(end_pose_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  moveit_control_simple
  end_pose_node
  moveit_control_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

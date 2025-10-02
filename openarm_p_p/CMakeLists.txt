cmake_minimum_required(VERSION 3.8)
project(openarm_p_p)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# Add include directories
include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Create the executable
add_executable(openarm_p_p src/openarm_p_p.cpp)
ament_target_dependencies(openarm_p_p
  rclcpp
  rclcpp_action
  action_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_msgs
)

# Install the executable
install(TARGETS openarm_p_p
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have them
 install(DIRECTORY
   launch
   DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

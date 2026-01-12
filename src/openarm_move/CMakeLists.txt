cmake_minimum_required(VERSION 3.8)
project(openarm_move)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED) 

# Create a library instead of executable
add_library(openarm_move SHARED
  src/ptp/planning.cpp
  src/ptp/pose.cpp
  src/ptp/Ptp.cpp
  src/estrategy/motion_strategy.cpp
  src/utils/utils.cpp
  src/openarm_move.cpp
)

target_include_directories(openarm_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(openarm_move
  rclcpp
  rclcpp_action
  action_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  control_msgs 
)

target_compile_options(openarm_move PRIVATE
  $<$<CONFIG:Debug>:-g -O0 -fno-omit-frame-pointer>
  $<$<CONFIG:RelWithDebInfo>:-g>)

# Link libraries explicitly
target_link_libraries(openarm_move
  tf2::tf2
  tf2_ros::tf2_ros
)

# Install the library
install(TARGETS openarm_move
  EXPORT openarm_moveTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export the library
ament_export_targets(openarm_moveTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(openarm_move)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  action_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  control_msgs
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

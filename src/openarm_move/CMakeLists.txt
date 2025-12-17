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
find_package(control_msgs REQUIRED) 


# Create the executable
add_executable(openarm_move
  src/exec.cpp
  src/interpolation.cpp
  src/planning.cpp
  src/pose.cpp
  src/Topics_utils.cpp
  src/openarm_move.cpp
  src/main.cpp
  src/utils.cpp
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

# Install the executable
install(TARGETS openarm_move
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

cmake_minimum_required(VERSION 3.8)
project(openarm_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(openarm_move REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(main src/main.cpp src/Topics_utils.cpp)

# Usar las variables que proporciona find_package
target_include_directories(main PUBLIC
  ${openarm_move_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(main 
  rclcpp 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs
  shape_msgs 
  geometric_shapes 
  moveit_ros_planning_interface
)

target_link_libraries(main 
  ${openarm_move_LIBRARIES}
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

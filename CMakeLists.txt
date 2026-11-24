cmake_minimum_required(VERSION 3.8)
project(lookie_trio_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ugv_sdk REQUIRED)

add_executable(lookie_trio_base_node
  src/lookie_trio_base.cpp
  src/lookie_trio_base_node.cpp
)
target_include_directories(lookie_trio_base_node PRIVATE include)
ament_target_dependencies(lookie_trio_base_node
  rclcpp rcl_interfaces diagnostic_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(lookie_trio_base_node ugv_sdk)

install(TARGETS lookie_trio_base_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md README.zh-CN.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_odometry test/test_odometry.cpp)
  target_include_directories(test_odometry PRIVATE include)
  add_test(NAME test_odometry COMMAND test_odometry)
endif()

ament_package()

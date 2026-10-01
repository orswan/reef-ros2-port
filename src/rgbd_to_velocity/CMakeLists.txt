cmake_minimum_required(VERSION 3.16)
project(rgbd_to_velocity)

# ROS 2 port (P08) of UF REEF rgbd_to_velocity b7637198 (docs/VISION.md).

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(reef_msgs REQUIRED)

# ROS-free core; -ffp-contract=off keeps results bit-identical to the harness build.
add_library(rgbd_to_velocity_core SHARED src/rgbd_to_velocity.cpp)
target_include_directories(rgbd_to_velocity_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_options(rgbd_to_velocity_core PRIVATE -ffp-contract=off)
target_link_libraries(rgbd_to_velocity_core PUBLIC Eigen3::Eigen)

add_library(rgbd_to_velocity_ros SHARED src/rgbd_node.cpp)
target_link_libraries(rgbd_to_velocity_ros PUBLIC rgbd_to_velocity_core rclcpp::rclcpp
  ${nav_msgs_TARGETS} ${reef_msgs_TARGETS})

add_executable(rgbd_to_velocity_node src/rgbd_to_velocity_node.cpp)
target_link_libraries(rgbd_to_velocity_node rgbd_to_velocity_ros)
add_executable(rgbd_to_velocity_event_replay tools/event_replay.cpp)
target_link_libraries(rgbd_to_velocity_event_replay rgbd_to_velocity_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS rgbd_to_velocity_node rgbd_to_velocity_event_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS rgbd_to_velocity_core rgbd_to_velocity_ros
  EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 rclcpp nav_msgs reef_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_converter test/test_converter.cpp)
  target_link_libraries(test_converter rgbd_to_velocity_core)
endif()

ament_package()

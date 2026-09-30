cmake_minimum_required(VERSION 3.16)
project(reef_control)

# ROS 2 port (P06) of UF REEF reef_control 12237b76 (docs/CONTROL_CHAIN.md).

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
find_package(reef_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROS-independent core. -ffp-contract=off: no FMA contraction regardless of
# -march, so results stay bit-identical to the reference harness build.
add_library(reef_control_core SHARED src/simple_pid.cpp src/controller.cpp src/PID.cpp src/gains.cpp
  src/firmware_mux.cpp)
target_include_directories(reef_control_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_options(reef_control_core PRIVATE -ffp-contract=off)
target_link_libraries(reef_control_core PUBLIC reef_msgs::reef_msgs_helpers Eigen3::Eigen)

# ROS 2 layer: parameters, message conversion, the node.
add_library(reef_control_ros SHARED src/control_node.cpp)
target_link_libraries(reef_control_ros PUBLIC reef_control_core rclcpp::rclcpp
  ${reef_msgs_TARGETS} ${builtin_interfaces_TARGETS} ${geometry_msgs_TARGETS}
  ${rosflight_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(reef_control_node src/reef_control_node.cpp)
target_link_libraries(reef_control_node reef_control_ros)

add_executable(reef_control_event_replay tools/event_replay.cpp)
target_link_libraries(reef_control_event_replay reef_control_ros)

# Dry-run command sink (no hardware output).
add_executable(reef_control_sink src/command_sink.cpp)
target_link_libraries(reef_control_sink reef_control_core rclcpp::rclcpp ${rosflight_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS reef_control_node reef_control_event_replay reef_control_sink DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY cfg config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS reef_control_core reef_control_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 reef_msgs rclcpp builtin_interfaces
  geometry_msgs rosflight_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core reef_control_core)
  ament_add_gtest(test_control_node test/test_control_node.cpp TIMEOUT 120)
  target_link_libraries(test_control_node reef_control_ros)
  ament_add_pytest_test(test_legacy_params test/test_legacy_params.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_node_process.py TIMEOUT 180)
endif()

ament_package()

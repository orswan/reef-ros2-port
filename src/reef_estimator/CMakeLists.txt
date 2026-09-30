cmake_minimum_required(VERSION 3.16)
project(reef_estimator)

# ROS 2 port of UF REEF reef_estimator (baseline master e4179f48).
# P03: parameter contract and configuration only. The estimator core and
# node follow in P04 (docs/INTERFACES.md section 3).

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

# ROS-independent core: parameter validation (P03) and the vertical filter
# (P04). No ROS types or clocks.
add_library(reef_estimator_core SHARED
  src/parameters.cpp
  src/estimator.cpp
  src/z_estimator.cpp
  src/vertical_estimator.cpp)
target_include_directories(reef_estimator_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(reef_estimator_core PUBLIC reef_msgs::reef_msgs_helpers Eigen3::Eigen)

# ROS 2 parameter declaration and loading.
add_library(reef_estimator_ros SHARED src/ros_parameters.cpp)
target_link_libraries(reef_estimator_ros PUBLIC
  reef_estimator_core reef_msgs::reef_msgs_parameters rclcpp::rclcpp)

# Event-replay driver for the fidelity comparison with the reference harness.
add_executable(reef_estimator_event_replay tools/event_replay.cpp)
target_link_libraries(reef_estimator_event_replay reef_estimator_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS reef_estimator_event_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS reef_estimator_core reef_estimator_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 reef_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rosflight_msgs REQUIRED)

  ament_add_gtest(test_parameters_core test/test_parameters_core.cpp)
  target_link_libraries(test_parameters_core reef_estimator_core)

  ament_add_gtest(test_ros_parameters test/test_ros_parameters.cpp)
  target_link_libraries(test_ros_parameters reef_estimator_ros)
  target_compile_definitions(test_ros_parameters PRIVATE
    CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  ament_add_gtest(test_rc_raw test/test_rc_raw.cpp)
  target_link_libraries(test_rc_raw reef_estimator_core ${rosflight_msgs_TARGETS})

  ament_add_pytest_test(test_legacy_params test/test_legacy_params.py)
endif()

ament_package()

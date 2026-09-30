cmake_minimum_required(VERSION 3.16)
project(reef_msgs)

# ROS 2 port (P03) of reef_msgs 7fb63ff: the messages and helpers used by
# reef_estimator master e4179f48. See README.md.

# Optimized by default, like the reference harness (-O2): Eigen's dynamic
# matrices are slow at -O0 (colcon's default). Results are unchanged
# (check_port.py compares every field; no -ffast-math, no FMA contraction
# without -march).
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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeltaToVel.msg"
  "msg/XYDebugEstimate.msg"
  "msg/XYEstimate.msg"
  "msg/XYZDebugEstimate.msg"
  "msg/XYZEstimate.msg"
  "msg/ZDebugEstimate.msg"
  "msg/ZEstimate.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Numerical helpers: Eigen only, no ROS. -ffp-contract=off keeps results
# bit-identical to the legacy build regardless of -march (no FMA contraction).
add_library(reef_msgs_helpers SHARED src/dynamics.cpp)
target_include_directories(reef_msgs_helpers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(reef_msgs_helpers PUBLIC Eigen3::Eigen)
target_compile_options(reef_msgs_helpers PRIVATE -ffp-contract=off)

# ROS 2 parameter reading for matrices.
add_library(reef_msgs_parameters SHARED src/parameters.cpp)
target_link_libraries(reef_msgs_parameters PUBLIC reef_msgs_helpers rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS reef_msgs_helpers reef_msgs_parameters
  EXPORT export_reef_msgs_helpers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_reef_msgs_helpers HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs
  eigen3_cmake_module Eigen3 rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

  # ROS-free tests: link the helper library only.
  ament_add_gtest(test_helpers_legacy test/test_helpers_legacy.cpp)
  target_link_libraries(test_helpers_legacy reef_msgs_helpers)
  target_compile_definitions(test_helpers_legacy PRIVATE
    LEGACY_VECTORS="${CMAKE_CURRENT_SOURCE_DIR}/test/data/legacy_helper_vectors.txt")
  target_compile_options(test_helpers_legacy PRIVATE -ffp-contract=off)

  ament_add_gtest(test_matrix_operation test/test_matrix_operation.cpp)
  target_link_libraries(test_matrix_operation reef_msgs_helpers)

  # ROS-dependent tests.
  ament_add_gtest(test_ros_conversions test/test_ros_conversions.cpp)
  target_link_libraries(test_ros_conversions reef_msgs_helpers
    ${cpp_typesupport_target} ${geometry_msgs_TARGETS})
  target_compile_definitions(test_ros_conversions PRIVATE
    LEGACY_VECTORS="${CMAKE_CURRENT_SOURCE_DIR}/test/data/legacy_helper_vectors.txt")
  target_compile_options(test_ros_conversions PRIVATE -ffp-contract=off)

  ament_add_gtest(test_parameters test/test_parameters.cpp)
  target_link_libraries(test_parameters reef_msgs_parameters)

  ament_add_pytest_test(test_messages test/test_messages.py)
  ament_add_pytest_test(test_ros_independence test/test_ros_independence.py
    ENV "REEF_CXX=${CMAKE_CXX_COMPILER}" "REEF_EIGEN_INCLUDE=${EIGEN3_INCLUDE_DIRS}"
        "REEF_PKG_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()

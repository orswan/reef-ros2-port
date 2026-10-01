cmake_minimum_required(VERSION 3.16)
project(reef_x3_adapter)

# C++ IMU adapter for the X3 simulation (P05): the latency-critical part of
# the adapter; reef_sim/reef_adapter.py keeps the velocity observations.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(x3_imu_adapter src/x3_imu_adapter.cpp)
target_include_directories(x3_imu_adapter PRIVATE include)
target_link_libraries(x3_imu_adapter rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS})
install(TARGETS x3_imu_adapter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED)
  ament_add_gtest(test_attitude test/test_attitude.cpp)
  target_include_directories(test_attitude PRIVATE include)
  target_link_libraries(test_attitude Eigen3::Eigen)
endif()

ament_package()

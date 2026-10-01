cmake_minimum_required(VERSION 3.16)
project(reef_fc_standin)

# Stand-in low-level loop (P07, development tool; docs/CONTROL_CHAIN.md section 7).

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
find_package(reef_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(reef_fc_standin_core SHARED src/standin.cpp)
target_include_directories(reef_fc_standin_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(reef_fc_standin_core PUBLIC reef_control::reef_control_core Eigen3::Eigen)

add_executable(reef_fc_standin src/standin_node.cpp)
target_link_libraries(reef_fc_standin reef_fc_standin_core rclcpp::rclcpp ${actuator_msgs_TARGETS}
  ${nav_msgs_TARGETS} ${rosflight_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS reef_fc_standin DESTINATION lib/${PROJECT_NAME})
install(TARGETS reef_fc_standin_core LIBRARY DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_standin test/test_standin.cpp)
  target_link_libraries(test_standin reef_fc_standin_core)
endif()

ament_package()

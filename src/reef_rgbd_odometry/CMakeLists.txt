cmake_minimum_required(VERSION 3.16)
project(reef_rgbd_odometry)

# Replacement RGB-D odometry (P08; docs/VISION.md section 6).

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
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc video)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(reef_rgbd_odometry_core SHARED src/odometry.cpp)
target_include_directories(reef_rgbd_odometry_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(reef_rgbd_odometry_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

add_executable(reef_rgbd_odometry src/odometry_node.cpp)
target_link_libraries(reef_rgbd_odometry reef_rgbd_odometry_core rclcpp::rclcpp cv_bridge::cv_bridge
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${std_msgs_TARGETS})

# Development tool (offline replay of recorded frames; tools/export_frames.py).
add_executable(vo_replay tools/vo_replay.cpp)
target_link_libraries(vo_replay reef_rgbd_odometry_core)

install(TARGETS reef_rgbd_odometry vo_replay DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/export_frames.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS reef_rgbd_odometry_core LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry reef_rgbd_odometry_core)
endif()

ament_package()

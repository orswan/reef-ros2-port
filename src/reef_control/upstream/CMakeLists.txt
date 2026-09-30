cmake_minimum_required(VERSION 2.8.3)
project(reef_control)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  reef_msgs
  roscpp
  rosflight_msgs
  rospy
)

find_package(Eigen REQUIRED)
find_package(PkgConfig REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/Gains.cfg
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_node      src/pid_node.cpp
        src/controller.cpp          include/controller.h
        src/PID.cpp                 include/PID.h
        src/simple_pid.cpp          include/simple_pid.h
        )

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
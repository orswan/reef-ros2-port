#pragma once
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
namespace sensor_msgs {
struct Imu {
  std_msgs::Header header;
  geometry_msgs::Quaternion orientation; boost::array<double, 9> orientation_covariance{};
  geometry_msgs::Vector3 angular_velocity; boost::array<double, 9> angular_velocity_covariance{};
  geometry_msgs::Vector3 linear_acceleration; boost::array<double, 9> linear_acceleration_covariance{};
};
typedef boost::shared_ptr<const Imu> ImuConstPtr;
}

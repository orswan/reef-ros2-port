// ROS 2 message overloads of the reef_msgs helpers.
// Ported from reef_msgs 7fb63ff src/dynamics.cpp (Prashant Ganesh, 2019):
// quaternion_to_rotation(geometry_msgs::Quaternion), body unchanged.

#ifndef REEF_MSGS__ROS_CONVERSIONS_HPP_
#define REEF_MSGS__ROS_CONVERSIONS_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

#include "reef_msgs/dynamics.h"

namespace reef_msgs
{

inline Eigen::Matrix3d quaternion_to_rotation(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond q_temp;
  q_temp.x() = q.x;
  q_temp.y() = q.y;
  q_temp.z() = q.z;
  q_temp.w() = q.w;

  return quaternion_to_rotation(q_temp);
}

}  // namespace reef_msgs

#endif  // REEF_MSGS__ROS_CONVERSIONS_HPP_

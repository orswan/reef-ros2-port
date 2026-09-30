//
// Created by prashant on 2/28/19.
//
// ROS 2 port (P03): bodies unchanged from reef_msgs 7fb63ff; unused
// functions removed (see dynamics.h).

#include "reef_msgs/dynamics.h"

#include <cmath>

namespace reef_msgs
{

Eigen::Matrix3d quaternion_to_rotation(Eigen::Quaterniond q)
{
  Eigen::Matrix3d I;
  I.setIdentity();

  Eigen::Matrix3d skew_mat = skew(Eigen::Vector3d(q.x(), q.y(), q.z()));
  Eigen::Matrix3d rotation_matrix = I - 2 * q.w() * skew_mat + 2 * skew_mat * skew_mat;

  return rotation_matrix;
}

void roll_pitch_yaw_from_rotation321(Eigen::Matrix3d C, double& roll, double& pitch, double& yaw)
{
  roll =  atan2(C(1,2),C(2,2));//roll
  pitch = -asin(C(0,2)); //pitch
  yaw =  atan2(C(0,1),C(0,0)); //yaw
}

}

//
// Created by prashant on 2/28/19.
//
// ROS 2 port (P03): only the functions used by reef_estimator master e4179f48
// are kept, with their bodies unchanged; the rest remain in the history
// (reef_msgs 7fb63ff). This header depends on Eigen only. The
// geometry_msgs overload of quaternion_to_rotation is in ros_conversions.hpp.

#ifndef PROJECT_DYNAMICS_H
#define PROJECT_DYNAMICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reef_msgs
{

// Returns C = I - 2 w [v]x + 2 [v]x^2 for q = (w, v). For a unit Hamilton
// quaternion that is the transpose of q.toRotationMatrix(): with q the
// orientation of the body in NED, C maps NED vectors into the body frame.
// q is not normalized.
Eigen::Matrix3d quaternion_to_rotation(Eigen::Quaterniond q);

// 3-2-1 Euler angles [rad] from C (NED to body):
// roll = atan2(C12, C22), pitch = -asin(C02), yaw = atan2(C01, C00).
void roll_pitch_yaw_from_rotation321(Eigen::Matrix3d C, double& roll, double& pitch, double& yaw );

inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec)
{
  Eigen::Matrix3d skew;
  skew << 0, -vec(2),  vec(1), \
          vec(2),       0, -vec(0), \
          -vec(1),  vec(0),       0;
  return skew;
}

}
#endif //PROJECT_DYNAMICS_H

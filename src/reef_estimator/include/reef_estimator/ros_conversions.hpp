// ROS 2 message <-> core conversions for the vertical estimator (P04).
//
// Inputs keep the field types of the original messages: sensor_msgs/Range is
// float32 in ROS 2 as in ROS 1, and stamps are converted with ROS 1's
// ros::Time::toSec() formula inside the core (INTERFACES.md section 3.4).
//
// Outputs: the horizontal filter is not ported, so every horizontal field of
// xyz_estimate / xyz_debug_estimate is NaN ("not estimated"), never zero.
#ifndef REEF_ESTIMATOR__ROS_CONVERSIONS_HPP_
#define REEF_ESTIMATOR__ROS_CONVERSIONS_HPP_

#include <cmath>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <reef_msgs/msg/xyz_debug_estimate.hpp>
#include <reef_msgs/msg/xyz_estimate.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "reef_estimator/vertical_estimator.h"
#include "reef_msgs/matrix_operation.h"

namespace reef_estimator
{

inline Stamp toStamp(const builtin_interfaces::msg::Time & t)
{
  Stamp s;
  s.sec = t.sec;
  s.nanosec = t.nanosec;
  return s;
}

inline builtin_interfaces::msg::Time toTime(const Stamp & s)
{
  builtin_interfaces::msg::Time t;
  t.sec = s.sec;
  t.nanosec = s.nanosec;
  return t;
}

inline ImuSample fromMsg(const sensor_msgs::msg::Imu & m)
{
  ImuSample s;
  s.stamp = toStamp(m.header.stamp);
  s.ax = m.linear_acceleration.x;
  s.ay = m.linear_acceleration.y;
  s.az = m.linear_acceleration.z;
  s.qx = m.orientation.x;
  s.qy = m.orientation.y;
  s.qz = m.orientation.z;
  s.qw = m.orientation.w;
  return s;
}

inline RangeSample fromMsg(const sensor_msgs::msg::Range & m)
{
  RangeSample s;
  s.stamp = toStamp(m.header.stamp);
  s.range = m.range;
  s.max_range = m.max_range;
  return s;
}

inline MocapPoseSample fromMsg(const geometry_msgs::msg::PoseStamped & m)
{
  MocapPoseSample s;
  s.stamp = toStamp(m.header.stamp);
  s.z = m.pose.position.z;
  return s;
}

inline RcSample fromMsg(const rosflight_msgs::msg::RCRaw & m)
{
  RcSample s;
  s.stamp = toStamp(m.header.stamp);
  s.values = m.values;
  return s;
}

inline void fillHorizontalUnavailable(reef_msgs::msg::XYDebugEstimate & xy)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  xy.x_dot = xy.y_dot = xy.pitch_bias = xy.roll_bias = xy.xa_bias = xy.ya_bias = nan;
  xy.sigma_plus.fill(nan);
  xy.sigma_minus.fill(nan);
}

// ZDebugEstimate block as in XYZEstimator::saveMinusState/publishEstimates.
inline void fillZ(reef_msgs::msg::ZDebugEstimate & out, const ZState & s)
{
  out.z = s.z;
  out.z_dot = s.z_dot;
  out.bias = s.bias;
  out.u = s.u;
  reef_msgs::matrixToArray(s.P, out.p);
  Eigen::Vector3d zSigma;
  zSigma(0) = 3 * sqrt(s.P(0, 0));
  zSigma(1) = 3 * sqrt(s.P(1, 1));
  zSigma(2) = 3 * sqrt(s.P(2, 2));
  out.sigma_plus[0] = out.z + zSigma(0);
  out.sigma_minus[0] = out.z - zSigma(0);
  out.sigma_plus[1] = out.z_dot + zSigma(1);
  out.sigma_minus[1] = out.z_dot - zSigma(1);
  out.sigma_plus[2] = out.bias + zSigma(2);
  out.sigma_minus[2] = out.bias - zSigma(2);
}

inline reef_msgs::msg::XYZEstimate toEstimateMsg(const VerticalEstimator & e)
{
  reef_msgs::msg::XYZEstimate m;
  m.header.stamp = toTime(e.stamp());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  m.xy_plus.x_dot = nan;
  m.xy_plus.y_dot = nan;
  const ZState z = e.plusState();
  m.z_plus.z = z.z;
  m.z_plus.z_dot = z.z_dot;
  return m;
}

inline reef_msgs::msg::XYZDebugEstimate toDebugMsg(const VerticalEstimator & e)
{
  reef_msgs::msg::XYZDebugEstimate m;
  m.header.stamp = toTime(e.stamp());
  fillHorizontalUnavailable(m.xy_minus);
  fillHorizontalUnavailable(m.xy_plus);
  fillZ(m.z_minus, e.minusState());
  fillZ(m.z_plus, e.plusState());
  return m;
}

}  // namespace reef_estimator

#endif  // REEF_ESTIMATOR__ROS_CONVERSIONS_HPP_

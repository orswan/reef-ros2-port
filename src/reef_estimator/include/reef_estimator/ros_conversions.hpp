// ROS 2 message <-> core conversions for the REEF estimator (P04/P05).
//
// Inputs keep the field types of the original messages: sensor_msgs/Range is
// float32 in ROS 2 as in ROS 1, and stamps are converted with ROS 1's
// ros::Time::toSec() formula inside the core (INTERFACES.md section 3.4).
//
// Outputs are filled as XYZEstimator::saveMinusState/publishEstimates did.
#ifndef REEF_ESTIMATOR__ROS_CONVERSIONS_HPP_
#define REEF_ESTIMATOR__ROS_CONVERSIONS_HPP_

#include <cmath>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <reef_msgs/msg/delta_to_vel.hpp>
#include <reef_msgs/msg/xyz_debug_estimate.hpp>
#include <reef_msgs/msg/xyz_estimate.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "reef_estimator/xyz_estimator.h"
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

// Mocap velocity: body-level linear x/y and covariance[0], [7].
inline TwistSample fromMsg(const geometry_msgs::msg::TwistWithCovarianceStamped & m)
{
  TwistSample s;
  s.stamp = toStamp(m.header.stamp);
  s.vx = m.twist.twist.linear.x;
  s.vy = m.twist.twist.linear.y;
  s.cov_xx = m.twist.covariance[0];
  s.cov_yy = m.twist.covariance[7];
  return s;
}

// RGB-D velocity: the vel field of DeltaToVel (other fields unused).
inline TwistSample fromMsg(const reef_msgs::msg::DeltaToVel & m)
{
  return fromMsg(m.vel);
}

inline RcSample fromMsg(const rosflight_msgs::msg::RCRaw & m)
{
  RcSample s;
  s.stamp = toStamp(m.header.stamp);
  s.values = m.values;
  return s;
}

// XYDebugEstimate block as in XYZEstimator::saveMinusState/publishEstimates.
inline void fillXY(reef_msgs::msg::XYDebugEstimate & out, const XYState & s)
{
  out.x_dot = s.x(0);
  out.y_dot = s.x(1);
  out.pitch_bias = s.x(2);
  out.roll_bias = s.x(3);
  out.xa_bias = s.x(4);
  out.ya_bias = s.x(5);
  Eigen::MatrixXd xySigma = Eigen::MatrixXd(6, 1);
  xySigma(0) = 3 * sqrt(s.P(0, 0));
  xySigma(1) = 3 * sqrt(s.P(1, 1));
  xySigma(2) = 3 * sqrt(s.P(2, 2));
  xySigma(3) = 3 * sqrt(s.P(3, 3));
  xySigma(4) = 3 * sqrt(s.P(4, 4));
  xySigma(5) = 3 * sqrt(s.P(5, 5));
  out.sigma_plus[0] = out.x_dot + xySigma(0);
  out.sigma_minus[0] = out.x_dot - xySigma(0);
  out.sigma_plus[1] = out.y_dot + xySigma(1);
  out.sigma_minus[1] = out.y_dot - xySigma(1);
  out.sigma_plus[2] = out.pitch_bias + xySigma(2);
  out.sigma_minus[2] = out.pitch_bias - xySigma(2);
  out.sigma_plus[3] = out.roll_bias + xySigma(3);
  out.sigma_minus[3] = out.roll_bias - xySigma(3);
  out.sigma_plus[4] = out.xa_bias + xySigma(4);
  out.sigma_minus[4] = out.xa_bias - xySigma(4);
  out.sigma_plus[5] = out.ya_bias + xySigma(5);
  out.sigma_minus[5] = out.ya_bias - xySigma(5);
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

inline reef_msgs::msg::XYZEstimate toEstimateMsg(const XYZEstimator & e)
{
  reef_msgs::msg::XYZEstimate m;
  m.header.stamp = toTime(e.stamp());
  const XYState xy = e.xyPlusState();
  m.xy_plus.x_dot = xy.x(0);
  m.xy_plus.y_dot = xy.x(1);
  const ZState z = e.plusState();
  m.z_plus.z = z.z;
  m.z_plus.z_dot = z.z_dot;
  return m;
}

inline reef_msgs::msg::XYZDebugEstimate toDebugMsg(const XYZEstimator & e)
{
  reef_msgs::msg::XYZDebugEstimate m;
  m.header.stamp = toTime(e.stamp());
  fillXY(m.xy_minus, e.xyMinusState());
  fillXY(m.xy_plus, e.xyPlusState());
  fillZ(m.z_minus, e.minusState());
  fillZ(m.z_plus, e.plusState());
  return m;
}

}  // namespace reef_estimator

#endif  // REEF_ESTIMATOR__ROS_CONVERSIONS_HPP_

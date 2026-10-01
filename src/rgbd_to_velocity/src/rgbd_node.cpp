#include "rgbd_to_velocity/rgbd_node.hpp"

#include <functional>
#include <string>
#include <vector>

#include <rclcpp/exceptions.hpp>

namespace rgbd_to_velocity
{

ConverterParameters loadParameters(rclcpp::Node & node)
{
  ConverterParameters p;
  std::vector<std::string> errors;
  rcl_interfaces::msg::ParameterDescriptor d;
  d.read_only = true;
  d.dynamic_typing = true;
  // Number (integers accepted, as roscpp's param<double> did).
  auto number = [&](const std::string & name, double & value) {
      const rclcpp::ParameterValue v = node.declare_parameter(name, rclcpp::ParameterValue(value), d);
      if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = v.get<double>();
      } else if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(v.get<int64_t>());
      } else {
        errors.push_back(name + ": expected a number");
      }
    };
  // Required list of numbers (the original zeroed a missing one, Q10).
  auto list = [&](const std::string & name, std::vector<double> & out) {
      const rclcpp::ParameterValue v = node.declare_parameter(name, rclcpp::ParameterValue(), d);
      if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        out = v.get<std::vector<double>>();
      } else if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        for (auto x : v.get<std::vector<int64_t>>()) {out.push_back(static_cast<double>(x));}
      } else if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        errors.push_back(name + " is required");
      } else {
        errors.push_back(name + ": expected a list of numbers");
      }
    };
  number("alpha", p.alpha);
  number("x_vel_covariance", p.x_vel_covariance);
  number("y_vel_covariance", p.y_vel_covariance);
  list("body_to_camera_quat", p.body_to_camera_quat);
  list("body_to_camera_trans", p.body_to_camera_trans);
  if (errors.empty()) {errors = parameterErrors(p);}
  if (!errors.empty()) {
    std::string m = "invalid rgbd_to_velocity parameters:";
    for (const auto & e : errors) {m += "\n  " + e;}
    throw ParameterError(m);
  }
  return p;
}

reef_msgs::msg::DeltaToVel toRos(const DeltaToVel & m)
{
  reef_msgs::msg::DeltaToVel r;
  r.header.stamp.sec = static_cast<int32_t>(m.header.stamp.sec);
  r.header.stamp.nanosec = m.header.stamp.nsec;
  for (int i = 0; i < 6; ++i) {r.s_upper_bound[i] = m.S_upper_bound[i]; r.s_lower_bound[i] = m.S_lower_bound[i];}
  for (int i = 0; i < 3; ++i) {r.scaled_std_xyz[i] = m.scaled_std_xyz[i];}
  r.vel.header.stamp.sec = static_cast<int32_t>(m.vel.header.stamp.sec);
  r.vel.header.stamp.nanosec = m.vel.header.stamp.nsec;
  r.vel.twist.twist.linear.x = m.vel.twist.twist.linear.x;
  r.vel.twist.twist.linear.y = m.vel.twist.twist.linear.y;
  r.vel.twist.twist.linear.z = m.vel.twist.twist.linear.z;
  for (int i = 0; i < 36; ++i) {r.vel.twist.covariance[i] = m.vel.twist.covariance[i];}
  return r;
}

Odometry fromRos(const nav_msgs::msg::Odometry & m)
{
  Odometry o;
  o.header.stamp.sec = m.header.stamp.sec;
  o.header.stamp.nsec = m.header.stamp.nanosec;
  const auto & p = m.pose.pose.position;
  const auto & q = m.pose.pose.orientation;
  o.pose.pose.position = {p.x, p.y, p.z};
  o.pose.pose.orientation = {q.x, q.y, q.z, q.w};
  return o;
}

RgbdNode::RgbdNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("rgbd_to_velocity_node", options)
{
  core_ = std::make_unique<RgbdToVelocity>(loadParameters(*this));
  // Latched queue-1 publishers in ROS 1: reliable, transient local, depth 1.
  const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  init_pub_ = create_publisher<reef_msgs::msg::DeltaToVel>("rgbd_to_velocity/init_frame", latched);
  body_pub_ = create_publisher<reef_msgs::msg::DeltaToVel>("rgbd_to_velocity/body_level_frame", latched);
  core_->velocity_init_frame_publisher_ = [this](const DeltaToVel & m) {
      last_init_ = toRos(m);
      init_pub_->publish(last_init_);
    };
  core_->velocity_level_body_publisher_ = [this](const DeltaToVel & m) {
      last_body_ = toRos(m);
      body_pub_->publish(last_body_);
    };
  sub_ = create_subscription<nav_msgs::msg::Odometry>("cam_to_init", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const nav_msgs::msg::Odometry & m) {onOdometry(m);});
  RCLCPP_INFO(get_logger(), "alpha %.3f, x/y velocity covariance %.4f/%.4f", core_->alpha,
    core_->x_vel_covariance, core_->y_vel_covariance);
}

void RgbdNode::onOdometry(const nav_msgs::msg::Odometry & m)
{
  core_->poseCallback(fromRos(m));
}

}  // namespace rgbd_to_velocity

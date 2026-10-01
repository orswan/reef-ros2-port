// ROS 2 node for the ported rgbd_to_velocity (P08): parameters, topics, and
// message conversion around the ROS-free converter. docs/VISION.md §4.
#ifndef RGBD_TO_VELOCITY_RGBD_NODE_HPP
#define RGBD_TO_VELOCITY_RGBD_NODE_HPP

#include <memory>
#include <stdexcept>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reef_msgs/msg/delta_to_vel.hpp>

#include "rgbd_to_velocity/rgbd_to_velocity.h"

namespace rgbd_to_velocity
{

class ParameterError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

ConverterParameters loadParameters(rclcpp::Node & node);   // throws ParameterError
reef_msgs::msg::DeltaToVel toRos(const DeltaToVel & m);
Odometry fromRos(const nav_msgs::msg::Odometry & m);

class RgbdNode : public rclcpp::Node
{
public:
  explicit RgbdNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void onOdometry(const nav_msgs::msg::Odometry & m);   // public for replay and tests
  const RgbdToVelocity & converter() const {return *core_;}
  const reef_msgs::msg::DeltaToVel & lastInitFrame() const {return last_init_;}
  const reef_msgs::msg::DeltaToVel & lastBodyLevel() const {return last_body_;}

private:
  std::unique_ptr<RgbdToVelocity> core_;
  reef_msgs::msg::DeltaToVel last_init_, last_body_;
  rclcpp::Publisher<reef_msgs::msg::DeltaToVel>::SharedPtr init_pub_, body_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace rgbd_to_velocity

#endif

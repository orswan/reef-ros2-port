// ROS 2 node for the ported reef_control PID controller (P06): topics,
// parameters, and message conversion around the ROS-free PIDController.
// Interface: docs/INTERFACES.md section 4.
#ifndef REEF_CONTROL_CONTROL_NODE_HPP
#define REEF_CONTROL_CONTROL_NODE_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reef_msgs/msg/desired_state.hpp>
#include <reef_msgs/msg/xyz_estimate.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <std_msgs/msg/bool.hpp>

#include "reef_control/PID.h"

namespace reef_control
{

// Invalid parameters at startup: the message lists every problem.
class ParameterError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Declares and reads the controller parameters; throws ParameterError.
ControllerParameters loadParameters(rclcpp::Node & node);

// Conversions between ROS 2 messages and the core structs.
Stamp toStamp(const builtin_interfaces::msg::Time & t);
builtin_interfaces::msg::Time toRos(const Stamp & s);
DesiredState fromRos(const reef_msgs::msg::DesiredState & m);
reef_msgs::msg::DesiredState toRos(const DesiredState & d);
XYZEstimate fromRos(const reef_msgs::msg::XYZEstimate & m);
PoseStamped fromRos(const geometry_msgs::msg::PoseStamped & m);
// Legacy command (x, y, z, F) as rosflight_msgs 2.x Command: u[0..3].
rosflight_msgs::msg::Command toRos(const Command & c, const Stamp & stamp);

class ControlNode : public rclcpp::Node
{
public:
  explicit ControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  const PIDController & controller() const {return *controller_;}
  const rosflight_msgs::msg::Command & lastCommand() const {return last_command_;}
  const reef_msgs::msg::DesiredState & lastControllerState() const {return last_state_;}

  // Subscription handlers (public so that replay and tests can deliver
  // messages in a fixed order on the calling thread).
  void onDesiredState(const reef_msgs::msg::DesiredState & m);
  void onStatus(const rosflight_msgs::msg::Status & m);
  void onIsFlying(const std_msgs::msg::Bool & m);
  void onEstimate(const reef_msgs::msg::XYZEstimate & m);
  void onPose(const geometry_msgs::msg::PoseStamped & m);

private:
  std::unique_ptr<PIDController> controller_;
  GainsConfig gains_;
  Stamp last_estimate_stamp_;
  rosflight_msgs::msg::Command last_command_;
  reef_msgs::msg::DesiredState last_state_;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;
  rclcpp::Publisher<reef_msgs::msg::DesiredState>::SharedPtr state_pub_;
  rclcpp::Subscription<reef_msgs::msg::DesiredState>::SharedPtr desired_sub_;
  rclcpp::Subscription<reef_msgs::msg::XYZEstimate>::SharedPtr estimate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flying_sub_;
  OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  PostSetParametersCallbackHandle::SharedPtr apply_handle_;
};

}  // namespace reef_control

#endif

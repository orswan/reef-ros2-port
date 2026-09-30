// ROS 2 port (P06) of reef_control 12237b76 controller.cpp: ROS plumbing
// removed (see controller.h); the command computation is unchanged.
#include "reef_control/controller.h"

namespace reef_control
{
  double durationSec(const Stamp& a, const Stamp& b)
  {
    // roscpp_core: Duration::fromNSec(a.toNSec() - b.toNSec()), normalized
    // to nsec in [0, 1e9); toSec() = sec + 1e-9 * nsec.
    const int64_t d = (a.sec * 1000000000LL + a.nsec) - (b.sec * 1000000000LL + b.nsec);
    int64_t sec = d / 1000000000LL;
    int64_t nsec = d % 1000000000LL;
    if (nsec < 0) { nsec += 1000000000LL; --sec; }
    return static_cast<double>(static_cast<int32_t>(sec)) + 1e-9 * static_cast<double>(static_cast<int32_t>(nsec));
  }

  Controller::Controller(const ControllerParameters& params) :
    initialized_(false),
    is_flying_(false),
    armed_(false)
  {

    // Parameters (the original asserted that these exist; gravity was read
    // but never used).
    max_roll_ = params.max_roll;
    max_pitch_ = params.max_pitch;
    max_yaw_rate_ = params.max_yaw_rate;

    time_of_previous_control_ = Stamp();
    dt = 0; thrust = 0; phi_desired = 0; theta_desired = 0;

  }

  void Controller::desiredStateCallback(const DesiredState& msg)
  {
    desired_state_ = msg;
  }

  void Controller::currentStateCallback(const XYZEstimate& msg)
  {
    current_state_.header = msg.header;
    current_state_.twist.twist.linear.x = msg.xy_plus.x_dot;
    current_state_.twist.twist.linear.y = msg.xy_plus.y_dot;
    current_state_.twist.twist.linear.z = msg.z_plus.z_dot;
    current_state_.pose.pose.position.z = msg.z_plus.z;
    computeCommand();
  }

  void Controller::poseCallback(const PoseStamped& msg)
  {
    current_state_.pose.pose.position.x = msg.pose.position.x;
    current_state_.pose.pose.position.y = msg.pose.position.y;
    current_state_.pose.pose.orientation = msg.pose.orientation;

  }

  void Controller::statusCallback(const Status &msg)
  {
    armed_ = msg.armed;
    initialized_ = armed_;
  }

  void Controller::isflyingCallback(const Bool &msg)
  {
    is_flying_ = msg.data;
    initialized_ = is_flying_ && armed_;
  }

  void Controller::computeCommand()
  {
    // Time calculation
    dt = durationSec(current_state_.header.stamp, time_of_previous_control_);
    time_of_previous_control_ = current_state_.header.stamp;
    if(dt <= 0.0000001)
    {
      // Don't do anything if dt is really close (or equal to) zero
      return;
    }

    computeCommand(current_state_ ,desired_state_,dt);

    phi_desired = desired_state_.acceleration.y;
    theta_desired = -desired_state_.acceleration.x;
    thrust = -desired_state_.acceleration.z;

    command.mode = Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command.F = std::min(std::max(thrust, 0.0), 1.0);
    if(!desired_state_.attitude_valid && !desired_state_.altitude_only) {
      command.ignore = 0x00;
      command.x = std::min(std::max(phi_desired, -1.0 * max_roll_), max_roll_);
      command.y = std::min(std::max(theta_desired, -1.0 * max_pitch_), max_pitch_);
      command.z = std::min(std::max(desired_state_.velocity.yaw, -1.0 * max_yaw_rate_), max_yaw_rate_);
    }else if(desired_state_.altitude_only)
      command.ignore = 0x07;
    else
    {
      command.ignore = 0x00;
      command.x = desired_state_.attitude.x;
      command.y = desired_state_.attitude.y;
      command.z = desired_state_.attitude.yaw;
    }

    ++numCommands;
    if (command_publisher_) command_publisher_(command);
  }

} //namespace

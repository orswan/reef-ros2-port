#ifndef CONTROLLER_H
#define CONTROLLER_H

// ROS 2 port (P06) of reef_control 12237b76: the Controller base without
// ROS. Messages are the plain structs of messages.hpp (same member paths and
// field types); publishers are replaced by sinks set by the node
// (control_node.hpp); parameters come in ControllerParameters, validated
// before construction. Statement order and arithmetic are the original's.
// See docs/CONTROL_CHAIN.md.

#include <math.h>
#include <eigen3/Eigen/Core>

#include <algorithm>
#include <functional>
#include <string>

#include "reef_control/gains.hpp"
#include "reef_control/messages.hpp"
#include "reef_control/simple_pid.h"


namespace reef_control
{
  class Controller
  {
  public:
    explicit Controller(const ControllerParameters& params);
    virtual ~Controller(){}

    bool initialized_;

    // Inputs: the original subscriber callbacks (desired_state, xyz_estimate,
    // pose_stamped, is_flying, status). rc_raw had an empty callback.
    void currentStateCallback(const XYZEstimate& msg);
    void desiredStateCallback(const DesiredState& msg);
    void poseCallback(const PoseStamped& msg);
    void isflyingCallback(const Bool& msg);
    void statusCallback(const Status &msg);

    // Output: the original command publisher.
    std::function<void(const Command&)> command_publisher_;

    // Introspection (read only).
    long commandCount() const { return numCommands; }
    bool armed() const { return armed_; }
    bool isFlying() const { return is_flying_; }
    const Odometry& currentState() const { return current_state_; }
    const DesiredState& desiredState() const { return desired_state_; }
    const Command& lastCommand() const { return command; }
    const Stamp& timeOfPreviousControl() const { return time_of_previous_control_; }
    double lastDt() const { return dt; }
    double phiDesired() const { return phi_desired; }
    double thetaDesired() const { return theta_desired; }
    double thrustDesired() const { return thrust; }
    double maxRoll() const { return max_roll_; }
    double maxPitch() const { return max_pitch_; }
    double maxYawRate() const { return max_yaw_rate_; }

   private:
    bool is_flying_;                       // Set by is_flying callback
    bool armed_;

    Odometry current_state_;
    DesiredState desired_state_;

    Stamp time_of_previous_control_;
    Command command;

    double max_roll_, max_pitch_, max_yaw_rate_;
    double dt;
    double thrust;
    double phi_desired;
    double theta_desired;
    long numCommands = 0;

    void computeCommand();  // Computes and sends command message

    // Virtual Function
    virtual void computeCommand(const Odometry current_state,
                  DesiredState& desired_state,
                  double dt) = 0;

  };
}
#endif

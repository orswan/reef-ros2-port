#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// ROS 2 port (P06) of reef_control 12237b76 PID.h without ROS. The
// dynamic_reconfigure server is replaced by gainsCallback(), called by the
// constructor (as setCallback did) and by the node on runtime parameter
// changes. Members the original declared but never used are dropped.
#include "reef_control/simple_pid.h"
#include <math.h>
#include <algorithm>
#include <reef_msgs/dynamics.h>
#include "reef_control/controller.h"

namespace reef_control
{
  class PIDController : public Controller
  {
  public:
    explicit PIDController(const ControllerParameters& params);

    // dynamic_reconfigure callback of the original. The gains must be valid
    // (gainErrors() empty): the original's clamping is replaced by rejection
    // before this call (CONTROL_CHAIN.md K13).
    void gainsCallback(const GainsConfig &config);

    // Output: the original controller_state publisher.
    std::function<void(const DesiredState&)> desired_state_pub_;
    // Optional log sink (the original used ROS_INFO).
    std::function<void(const std::string&)> log;

    // Introspection (read only).
    long controllerStateCount() const { return numControllerStates; }
    const SimplePID& dPID() const { return d_; }
    const SimplePID& wPID() const { return w_; }
    const SimplePID& yawPID() const { return yaw_; }
    const SimplePID& uPID() const { return u_; }
    const SimplePID& vPID() const { return v_; }
    double currentYaw() const { return current_yaw; }
    double headingToTarget() const { return theta; }
    double lookupKp() const { return kp; }
    double lookupDeadzone() const { return deadzone; }
    double lookupVelMax() const { return vel_max; }
    double lookupCenter() const { return x_0; }
    double lookupAlpha() const { return alpha; }
    double lookupSigma() const { return sigma; }
    bool faceTarget() const { return face_target_; }
    bool flyFixedWing() const { return fly_fixed_wing_; }

  private:
    reef_control::SimplePID d_; //simple pid object for Down
    reef_control::SimplePID yaw_; //simple pid object for Yaw
    reef_control::SimplePID u_; //simple pid object for velocity
    reef_control::SimplePID v_; //simple pid object for velocity
    reef_control::SimplePID w_; //simple pid object for velocity

    double current_yaw = 0;
    double kp = 0;
    double deadzone = 0;
    double vel_max = 0;
    double x_0 = 0;
    double alpha = 0;
    double sigma = 0;
    double theta = 0;   // uninitialized in the original (K10); 0 here and in the reference harness
    bool face_target_;
    bool fly_fixed_wing_;
    long numControllerStates = 0;

    void lookupTable(DesiredState& desired_state ,const Odometry& current_state);

    void computeCommand(const Odometry current_state,
                        DesiredState& desired_state,
                        double dt) override;

  };
}
#endif

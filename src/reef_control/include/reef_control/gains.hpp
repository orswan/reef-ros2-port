// Controller parameters (P06). The gains are the fields of reef_control
// 12237b76 cfg/Gains.cfg that the controller uses, with the same defaults;
// the ranges are Gains.cfg's (test_gains_cfg checks this table against the
// file). The original clamped out-of-range values silently
// (dynamic_reconfigure); the port rejects them (CONTROL_CHAIN.md K13).
// No ROS types.
#ifndef REEF_CONTROL_GAINS_HPP
#define REEF_CONTROL_GAINS_HPP

#include <string>
#include <vector>

// name, default, min, max (doubles)
#define REEF_CONTROL_GAIN_DOUBLES(X) \
  X(uP, 0, 0, 2) X(uI, 0, 0, 1) X(uD, 0, 0, 0.5) \
  X(vP, 0, 0, 2) X(vI, 0, 0, 1) X(vD, 0, 0, 0.5) \
  X(wP, 0, 0, 2) X(wI, 0, 0, 1) X(wD, 0, 0, 0.5) \
  X(uvtau, 0, 0, 1) \
  X(kp, 0, 0, 1) X(deadzone, 0, 0, 0.5) X(max_vel, 0, 0, 3) X(center_point, 0, 0, 2.0) X(alpha, 0, 0, 2) \
  X(dP, 0, 0, 5) X(dI, 0, 0, 1) X(dD, 0, 0, 0.5) X(nedtau, 0, 0, 1) \
  X(yawP, 0, 0, 2) X(yawI, 0, 0, 1) X(yawD, 0, 0, 0.5) X(yawtau, 0, 0, 1) \
  X(max_u, 0, 0, 2.5) X(max_v, 0, 0, 2.5) X(max_w, 0, 0, 2.5) X(max_d, 0, 0, 2.0)
// name, default (booleans)
#define REEF_CONTROL_GAIN_BOOLS(X) X(xIntegrator, true) X(uIntegrator, true)

namespace reef_control
{
    struct GainsConfig
    {
#define REEF_CONTROL_MEMBER_D(n, d, lo, hi) double n = d;
#define REEF_CONTROL_MEMBER_B(n, d) bool n = d;
        REEF_CONTROL_GAIN_DOUBLES(REEF_CONTROL_MEMBER_D)
        REEF_CONTROL_GAIN_BOOLS(REEF_CONTROL_MEMBER_B)
#undef REEF_CONTROL_MEMBER_D
#undef REEF_CONTROL_MEMBER_B
    };

    struct GainRange { std::string name; double def, lo, hi; };
    const std::vector<GainRange>& gainRanges();          // the doubles
    const std::vector<std::string>& gainBools();
    double* gainField(GainsConfig& g, const std::string& name);   // nullptr if not a double gain
    bool* gainBoolField(GainsConfig& g, const std::string& name); // nullptr if not a bool gain
    // One message per gain that is not finite or outside its range; empty if valid.
    std::vector<std::string> gainErrors(const GainsConfig& g);

    struct ControllerParameters
    {
        // Required by the original (ROS_ASSERT); finite and >= 0 here.
        double max_roll = 0, max_pitch = 0, max_yaw_rate = 0;
        // Read from the global namespace by the original (K12); default false.
        bool face_target = false, fly_fixed_wing = false;
        GainsConfig gains;
    };
    // Errors of the limits and gains; empty if valid.
    std::vector<std::string> parameterErrors(const ControllerParameters& p);
}

#endif

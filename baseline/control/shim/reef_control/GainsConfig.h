#pragma once
// Adaptation C2 (baseline/README.md): stand-in for the header that
// dynamic_reconfigure generates from reef_control cfg/Gains.cfg (12237b76).
// Same fields, types, defaults and [min, max] as Gains.cfg (checked against
// the file by baseline/control/check_gains_cfg.py), with the generated
// behaviour the controller relies on:
//   __getDefault__()  the cfg defaults
//   __fromServer__()  each field from the node's parameter (getParam
//                     semantics of the stand-in NodeHandle); absent -> kept
//   __clamp__()       every field clamped to [min, max]
#include <ros/ros.h>
#include <string>

// name, C type, default, min, max
#define REEF_GAINS_FIELDS(X) \
  X(uP, double, 0, 0, 2) X(uI, double, 0, 0, 1) X(uD, double, 0, 0, 0.5) \
  X(vP, double, 0, 0, 2) X(vI, double, 0, 0, 1) X(vD, double, 0, 0, 0.5) \
  X(wP, double, 0, 0, 2) X(wI, double, 0, 0, 1) X(wD, double, 0, 0, 0.5) \
  X(uvtau, double, 0, 0, 1) \
  X(kp, double, 0, 0, 1) X(deadzone, double, 0, 0, 0.5) X(max_vel, double, 0, 0, 3) \
  X(center_point, double, 0, 0, 2.0) X(alpha, double, 0, 0, 2) \
  X(dP, double, 0, 0, 5) X(dI, double, 0, 0, 1) X(dD, double, 0, 0, 0.5) X(nedtau, double, 0, 0, 1) \
  X(yawP, double, 0, 0, 2) X(yawI, double, 0, 0, 1) X(yawD, double, 0, 0, 0.5) X(yawtau, double, 0, 0, 1) \
  X(yawRateP, double, 0, 0, 2) X(yawRateI, double, 0, 0, 1) X(yawRateD, double, 0, 0, 0.5) X(yawRatetau, double, 0, 0, 1) \
  X(xIntegrator, bool, true, false, true) X(uIntegrator, bool, true, false, true) \
  X(max_u, double, 0, 0, 2.5) X(max_v, double, 0, 0, 2.5) X(max_w, double, 0, 0, 2.5) X(max_d, double, 0, 0, 2.0) \
  X(max_n, double, 0, 0, 1.5) X(max_e, double, 0, 0, 1.5) X(max_yaw_rate, double, 0, 0, 0.50)

namespace reef_control {
struct GainsConfig {
#define REEF_GAINS_MEMBER(n, T, d, lo, hi) T n = d;
  REEF_GAINS_FIELDS(REEF_GAINS_MEMBER)
#undef REEF_GAINS_MEMBER
  static GainsConfig __getDefault__() { return GainsConfig(); }
  void __fromServer__(const ros::NodeHandle& nh) {
#define REEF_GAINS_READ(n, T, d, lo, hi) nh.getParam(#n, n);
    REEF_GAINS_FIELDS(REEF_GAINS_READ)
#undef REEF_GAINS_READ
  }
  void __clamp__() {
#define REEF_GAINS_CLAMP(n, T, d, lo, hi) if (n > static_cast<T>(hi)) n = static_cast<T>(hi); if (n < static_cast<T>(lo)) n = static_cast<T>(lo);
    REEF_GAINS_FIELDS(REEF_GAINS_CLAMP)
#undef REEF_GAINS_CLAMP
  }
  // Harness only: set one field by name (runtime reconfigure event).
  bool set(const std::string& name, double v) {
#define REEF_GAINS_SET(n, T, d, lo, hi) if (name == #n) { n = static_cast<T>(v); return true; }
    REEF_GAINS_FIELDS(REEF_GAINS_SET)
#undef REEF_GAINS_SET
    return false;
  }
};
}  // namespace reef_control

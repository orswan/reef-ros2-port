// reef_fc_standin core: a SIMPLIFIED LOW-LEVEL STAND-IN (DEVELOPMENT TOOL) for
// the ROSflight firmware's job in the closed loop (docs/CONTROL_CHAIN.md
// section 7). It is not ROSflight and not flight-representative.
//
//   command mux   reef_control::FirmwareMux (firmware b77c3854 semantics:
//                 ignore bits, 100 ms offboard timeout); inactive channels
//                 take the neutral "RC" value (0 angle, 0 rate, 0 throttle);
//                 disarmed -> motors stopped
//   attitude      roll/pitch: angle P with D on the body rate (the firmware's
//                 structure), yaw: rate P; torque = inertia * (gain law)
//   thrust        linear: T = F * T_max, T_max = 4 k w_max^2 (assumption: a
//                 thrust-linearized motor; ROSflight maps F to PWM)
//   allocation    exact inverse of the rotor geometry (FRD):
//                 T = sum f, tau_x = sum -y f, tau_y = sum x f,
//                 tau_z = sum dir m f; each f clamped to [0, k w_max^2];
//                 w = sqrt(f / k)
// Frames: body FRD, world NED (REEF's). No ROS types.
#ifndef REEF_FC_STANDIN_STANDIN_HPP
#define REEF_FC_STANDIN_STANDIN_HPP

#include <Eigen/Dense>

#include <array>
#include <cstdint>

#include "reef_control/firmware_mux.hpp"

namespace reef_fc_standin
{

struct Rotor
{
  double x = 0, y = 0;   // position in body FRD [m]
  int dir = 1;           // +1: propeller turns counter-clockwise seen from above (body reaction +z FRD)
};

struct Params
{
  std::array<Rotor, 4> rotors{};
  double motor_constant = 0;    // thrust = k w^2 [N/(rad/s)^2]
  double moment_constant = 0;   // drag torque = m * thrust [m]
  double max_rot_velocity = 0;  // [rad/s]
  double ixx = 0, iyy = 0, izz = 0;
  double kp_angle = 0;          // [1/s^2]
  double kd_rate = 0;           // [1/s]
  double kp_yaw_rate = 0;       // [1/s]
  int64_t offboard_timeout_ms = 100;
};

// Attitude of FRD in NED (3-2-1 angles) and body rates in FRD.
struct State
{
  double roll = 0, pitch = 0, yaw = 0;
  double p = 0, q = 0, r = 0;
};

struct Output
{
  bool armed = false;
  double F = 0, roll_c = 0, pitch_c = 0, yaw_rate_c = 0;   // what the mux selected
  uint8_t active = 0;                                     // bit i: channel i (x, y, z, F) offboard
  double T = 0, tau_x = 0, tau_y = 0, tau_z = 0;           // demanded
  std::array<double, 4> thrust{}, omega{};                 // per rotor, after clamping
  bool saturated = false;                                  // any rotor clamped
};

Eigen::Matrix4d allocation(const Params & p);   // rows: T, tau_x, tau_y, tau_z
double maxThrust(const Params & p);              // T_max
// roll, pitch, yaw of a unit quaternion (w, x, y, z) of FRD in NED.
void eulerFromQuaternion(double w, double x, double y, double z, double & roll, double & pitch, double & yaw);

class StandIn
{
public:
  explicit StandIn(const Params & p);
  void command(int64_t now_ms, uint8_t mode, uint16_t ignore, const std::array<float, 4> & xyzF);
  void setArmed(bool armed) {mux_.setArmed(armed);}
  bool armed() const {return mux_.armed();}
  long offboardTimeouts() const {return mux_.timeouts();}
  Output step(int64_t now_ms, const State & s);

private:
  Params p_;
  Eigen::Matrix4d inv_;
  reef_control::FirmwareMux mux_;
};

}  // namespace reef_fc_standin

#endif

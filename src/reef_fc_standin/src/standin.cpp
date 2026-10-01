#include "reef_fc_standin/standin.hpp"

#include <algorithm>
#include <cmath>

namespace reef_fc_standin
{

Eigen::Matrix4d allocation(const Params & p)
{
  Eigen::Matrix4d a;
  for (int i = 0; i < 4; ++i) {
    const Rotor & r = p.rotors[i];
    a(0, i) = 1.0;
    a(1, i) = -r.y;
    a(2, i) = r.x;
    a(3, i) = r.dir * p.moment_constant;
  }
  return a;
}

double maxThrust(const Params & p)
{
  return 4.0 * p.motor_constant * p.max_rot_velocity * p.max_rot_velocity;
}

void eulerFromQuaternion(double w, double x, double y, double z, double & roll, double & pitch, double & yaw)
{
  roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

StandIn::StandIn(const Params & p)
: p_(p), inv_(allocation(p).inverse()), mux_(p.offboard_timeout_ms) {}

void StandIn::command(int64_t now_ms, uint8_t mode, uint16_t ignore, const std::array<float, 4> & xyzF)
{
  mux_.command(now_ms, mode, ignore, xyzF);
}

Output StandIn::step(int64_t now_ms, const State & s)
{
  mux_.tick(now_ms);
  Output o;
  o.armed = mux_.armed();
  const auto & ch = mux_.channels();
  for (int i = 0; i < 4; ++i) {
    if (ch[i].active) {o.active |= static_cast<uint8_t>(1u << i);}
  }
  const bool supported = mux_.modeSupported();
  auto value = [&](int i) {return (ch[i].active && supported) ? static_cast<double>(ch[i].value) : 0.0;};
  o.roll_c = value(0);
  o.pitch_c = value(1);
  o.yaw_rate_c = value(2);
  o.F = std::clamp(value(3), 0.0, 1.0);
  if (!o.armed) {
    return o;   // motors stopped
  }
  o.T = o.F * maxThrust(p_);
  o.tau_x = p_.ixx * (p_.kp_angle * (o.roll_c - s.roll) - p_.kd_rate * s.p);
  o.tau_y = p_.iyy * (p_.kp_angle * (o.pitch_c - s.pitch) - p_.kd_rate * s.q);
  o.tau_z = p_.izz * p_.kp_yaw_rate * (o.yaw_rate_c - s.r);
  const Eigen::Vector4d f = inv_ * Eigen::Vector4d(o.T, o.tau_x, o.tau_y, o.tau_z);
  const double f_max = p_.motor_constant * p_.max_rot_velocity * p_.max_rot_velocity;
  for (int i = 0; i < 4; ++i) {
    const double c = std::clamp(f(i), 0.0, f_max);
    o.saturated = o.saturated || c != f(i);
    o.thrust[i] = c;
    o.omega[i] = std::sqrt(c / p_.motor_constant);
  }
  return o;
}

}  // namespace reef_fc_standin

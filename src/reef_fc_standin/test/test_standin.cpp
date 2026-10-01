// Stand-in core: allocation round trip and torque signs from geometry (FRD:
// a thrust f at (x, y) gives tau = r x (0, 0, -f) = (-y f, x f, 0)), hover
// thrust, attitude law signs, firmware mux behaviour (disarmed, timeout,
// ignore bits), quaternion -> Euler.
#include <gtest/gtest.h>

#include <cmath>

#include "reef_fc_standin/standin.hpp"

using namespace reef_fc_standin;

namespace
{
Params x3()
{
  Params p;
  p.rotors = {{{0.13, 0.22, 1}, {-0.13, -0.20, 1}, {0.13, -0.22, -1}, {-0.13, 0.20, -1}}};
  p.motor_constant = 8.54858e-06; p.moment_constant = 0.016; p.max_rot_velocity = 800.0;
  p.ixx = 0.0347563; p.iyy = 0.07; p.izz = 0.0977;
  p.kp_angle = 64.0; p.kd_rate = 12.8; p.kp_yaw_rate = 1.0;
  return p;
}
}  // namespace

TEST(Allocation, RoundTripAndSigns)
{
  const Params p = x3();
  const Eigen::Matrix4d a = allocation(p);
  const Eigen::Vector4d w(14.9, 0.05, -0.03, 0.01);
  EXPECT_LT((a * (a.inverse() * w) - w).norm(), 1e-12);
  // More thrust on the right-side rotors (y > 0 in FRD) rolls left: tau_x < 0.
  EXPECT_LT((a * Eigen::Vector4d(1, 0, 0, 1))(1), 0.0);
  // More thrust on the front rotors pitches nose up: tau_y > 0.
  EXPECT_GT((a * Eigen::Vector4d(1, 0, 1, 0))(2), 0.0);
}

TEST(StandIn, HoverThrottleGivesWeight)
{
  StandIn s(x3());
  s.setArmed(true);
  const float F = static_cast<float>(1.52 * 9.8 / maxThrust(x3()));
  s.command(1000, 2, 0, {0, 0, 0, F});
  const Output o = s.step(1004, State());
  EXPECT_NEAR(o.thrust[0] + o.thrust[1] + o.thrust[2] + o.thrust[3], 1.52 * 9.8, 1e-5);
  EXPECT_FALSE(o.saturated);
  EXPECT_NEAR(F, 0.681, 0.001);
}

TEST(StandIn, AttitudeLawSigns)
{
  StandIn s(x3());
  s.setArmed(true);
  s.command(1000, 2, 0, {0.1f, -0.1f, 0.2f, 0.7f});   // roll right, pitch nose down, yaw right
  const Output o = s.step(1004, State());
  EXPECT_GT(o.tau_x, 0.0);
  EXPECT_LT(o.tau_y, 0.0);
  EXPECT_GT(o.tau_z, 0.0);
  State rolling;
  rolling.p = 1.0;                                     // rate damping opposes motion
  s.command(1010, 2, 0, {0, 0, 0, 0.7f});
  EXPECT_LT(s.step(1014, rolling).tau_x, 0.0);
}

TEST(StandIn, MuxDisarmedTimeoutAndIgnore)
{
  StandIn s(x3());
  s.command(1000, 2, 0, {0, 0, 0, 0.7f});
  EXPECT_EQ(s.step(1004, State()).omega[0], 0.0);      // disarmed: motors stopped
  s.setArmed(true);
  s.command(1010, 2, 0x07, {0.2f, 0.2f, 0.2f, 0.7f});  // altitude-only: angles from "RC" (0)
  Output o = s.step(1014, State());
  EXPECT_EQ(o.roll_c, 0.0);
  EXPECT_EQ(o.active, 0x08);
  EXPECT_NEAR(o.F, 0.7, 1e-6);
  o = s.step(1111, State());                           // > 100 ms after the command
  EXPECT_EQ(o.F, 0.0);
  EXPECT_EQ(s.offboardTimeouts(), 1);
}

TEST(Euler, KnownRotations)
{
  double r, p, y;
  eulerFromQuaternion(std::cos(0.15), std::sin(0.15), 0, 0, r, p, y);   // roll 0.3
  EXPECT_NEAR(r, 0.3, 1e-12); EXPECT_NEAR(p, 0.0, 1e-12); EXPECT_NEAR(y, 0.0, 1e-12);
  eulerFromQuaternion(std::cos(0.2), 0, 0, std::sin(0.2), r, p, y);     // yaw 0.4
  EXPECT_NEAR(y, 0.4, 1e-12);
}

// Core tests with expected values that do not come from the code:
// physical signs in NED/FRD (CONTROL_CHAIN.md section 3), hand-computed ROS 1
// durations, output limits, parameter validation, and the firmware
// offboard-timeout rule (OFFBOARD_TIMEOUT = 100 ms, "now > stamp + timeout").
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "reef_control/PID.h"
#include "reef_control/firmware_mux.hpp"

using namespace reef_control;

namespace
{

ControllerParameters quad()
{
  ControllerParameters p;
  p.max_roll = 0.25; p.max_pitch = 0.25; p.max_yaw_rate = 2.0;
  GainsConfig & g = p.gains;
  g.kp = 0.8; g.deadzone = 0.1; g.max_vel = 1.5; g.center_point = 1.0; g.alpha = 0.4;
  g.uP = 0.3; g.uI = 0.025; g.vP = 0.34; g.vI = 0.03; g.vD = 0.005;
  g.wP = 0.6; g.wI = 0.2; g.uvtau = 0.15; g.dP = 0.75; g.dI = 0.05; g.nedtau = 0.15;
  g.yawP = 0.7; g.yawI = 0.08; g.yawD = 0.01; g.yawtau = 0.15;
  g.max_u = 1.0; g.max_v = 1.0; g.max_w = 1.0; g.max_d = 1.0;
  return p;
}

XYZEstimate estimate(double t, double z, double zd, double vx, double vy)
{
  XYZEstimate e;
  e.header.stamp.sec = static_cast<int64_t>(std::floor(t));
  e.header.stamp.nsec = static_cast<uint32_t>(std::lround((t - std::floor(t)) * 1e9));
  e.z_plus.z = z; e.z_plus.z_dot = zd; e.xy_plus.x_dot = vx; e.xy_plus.y_dot = vy;
  return e;
}

struct Step {Command cmd; double thrust;};

// Two steps, unarmed: the integrators are cleared at every step, so only the
// P and D terms act (the first step's huge dt, K4, would otherwise saturate
// them). Returns the second command and the throttle demand before clamping.
Step twoSteps(const ControllerParameters & p, const DesiredState & d, double z, double zd, double vx, double vy,
              double yaw = 0.0)
{
  PIDController c(p);
  PoseStamped pose;
  pose.pose.orientation.z = std::sin(yaw / 2); pose.pose.orientation.w = std::cos(yaw / 2);
  c.poseCallback(pose);
  c.desiredStateCallback(d);
  c.currentStateCallback(estimate(100.000, z, zd, vx, vy));
  c.currentStateCallback(estimate(100.004, z, zd, vx, vy));
  return {c.lastCommand(), c.thrustDesired()};
}

DesiredState altitude(double z) {DesiredState d; d.pose.z = z; return d;}
DesiredState velocity(double vx, double vy, double z = -1.0)
{
  DesiredState d; d.velocity_valid = true; d.velocity.x = vx; d.velocity.y = vy; d.pose.z = z; return d;
}

}  // namespace

TEST(Signs, AboveTheTargetGivesLessThrottle)
{
  // NED: z = -1.1 is 10 cm above a -1.0 target. Throttle demand = -acceleration.z.
  const double above = twoSteps(quad(), altitude(-1.0), -1.1, 0, 0, 0).thrust;
  const double at = twoSteps(quad(), altitude(-1.0), -1.0, 0, 0, 0).thrust;
  const double below = twoSteps(quad(), altitude(-1.0), -0.9, 0, 0, 0).thrust;
  EXPECT_LT(above, at);
  EXPECT_LT(at, below);
}

TEST(Signs, ClimbingFasterThanCommandedGivesLessThrottle)
{
  // At the target altitude (commanded climb rate 0); z_dot < 0 means climbing (NED).
  EXPECT_LT(twoSteps(quad(), altitude(-1.0), -1.0, -0.2, 0, 0).thrust,
            twoSteps(quad(), altitude(-1.0), -1.0, 0.2, 0, 0).thrust);
}

TEST(Signs, ForwardDeficitPitchesNoseDown)
{
  // Commanded 0.2 m/s forward, flying 0: accelerate forward = negative pitch (FRD).
  const Command c = twoSteps(quad(), velocity(0.2, 0.0), -1.0, 0, 0.0, 0.0).cmd;
  EXPECT_LT(c.y, 0.0f);
  EXPECT_EQ(c.ignore, 0);
  EXPECT_EQ(c.mode, Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE);
}

TEST(Signs, RightwardDeficitRollsRight)
{
  // Commanded 0.2 m/s to the right: positive roll (right side down).
  EXPECT_GT(twoSteps(quad(), velocity(0.0, 0.2), -1.0, 0, 0.0, 0.0).cmd.x, 0.0f);
  EXPECT_LT(twoSteps(quad(), velocity(0.0, -0.2), -1.0, 0, 0.0, 0.0).cmd.x, 0.0f);
}

TEST(Signs, PositionModeHeadsTowardsTheTarget)
{
  // Target 2 m north, heading east (yaw +90 deg): the target is to the LEFT
  // in body-level axes, so the velocity request is -y (and about 0 forward).
  DesiredState d; d.position_valid = true; d.pose.x = 2.0; d.pose.y = 0.0; d.pose.z = -1.0; d.pose.yaw = M_PI / 2;
  PIDController c(quad());
  PoseStamped pose;
  pose.pose.orientation.z = std::sin(M_PI / 4); pose.pose.orientation.w = std::cos(M_PI / 4);
  c.poseCallback(pose);
  c.desiredStateCallback(d);
  c.currentStateCallback(estimate(100.0, -1.0, 0, 0, 0));
  EXPECT_LT(c.desiredState().velocity.y, -1.0);
  EXPECT_NEAR(c.desiredState().velocity.x, 0.0, 1e-12);
  EXPECT_TRUE(c.desiredState().velocity_valid);
}

TEST(Signs, YawErrorGivesYawRateTowardsIt)
{
  // Desired heading 0.3 rad, current 0.1 rad (NED yaw positive clockwise): positive yaw rate.
  DesiredState d; d.position_valid = true; d.pose.z = -1.0; d.pose.yaw = 0.3;
  EXPECT_GT(twoSteps(quad(), d, -1.0, 0, 0, 0, 0.1).cmd.z, 0.0f);
}

TEST(Limits, CommandsStayWithinLimitsForFiniteInputs)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-5, 5);
  for (int trial = 0; trial < 200; ++trial) {
    PIDController c(quad());
    Status armed; armed.armed = true;
    c.statusCallback(armed);
    DesiredState d;
    d.position_valid = trial % 2; d.velocity_valid = true;
    d.pose.x = u(rng); d.pose.y = u(rng); d.pose.z = u(rng); d.pose.yaw = u(rng);
    d.velocity.x = u(rng); d.velocity.y = u(rng); d.velocity.yaw = u(rng);
    c.desiredStateCallback(d);
    for (int k = 0; k < 50; ++k) {
      PoseStamped p;
      p.pose.position.x = u(rng); p.pose.position.y = u(rng);
      const double yaw = u(rng);
      p.pose.orientation.z = std::sin(yaw / 2); p.pose.orientation.w = std::cos(yaw / 2);
      c.poseCallback(p);
      c.currentStateCallback(estimate(100.0 + 0.004 * k, u(rng), u(rng), u(rng), u(rng)));
      const Command & cmd = c.lastCommand();
      ASSERT_LE(std::abs(cmd.x), 0.25f);
      ASSERT_LE(std::abs(cmd.y), 0.25f);
      ASSERT_LE(std::abs(cmd.z), 2.0f);
      ASSERT_GE(cmd.F, 0.0f);
      ASSERT_LE(cmd.F, 1.0f);
    }
  }
}

TEST(Time, DurationIsRos1Normalized)
{
  // Hand-computed: 101.000000003 - 100.999999999 = 0 s + 4 ns.
  EXPECT_EQ(durationSec({101, 3}, {100, 999999999}), 0.0 + 1e-9 * 4.0);
  // Backwards by 1 ns: sec -1, nsec 999999999 -> -1 + 0.999999999.
  EXPECT_EQ(durationSec({100, 0}, {100, 1}), -1.0 + 1e-9 * 999999999.0);
  // From zero: the stamp itself (K4).
  EXPECT_EQ(durationSec({1790000000, 4000000}, {0, 0}), 1790000000.0 + 1e-9 * 4000000.0);
}

TEST(Steps, EqualOrTinyStampStepsDoNotPublish)
{
  PIDController c(quad());
  c.currentStateCallback(estimate(100.0, 0, 0, 0, 0));
  EXPECT_EQ(c.commandCount(), 1);
  XYZEstimate e = estimate(100.0, 0, 0, 0, 0);
  c.currentStateCallback(e);           // dt = 0
  e.header.stamp.nsec += 99;           // dt = 1e-9 * 99 = 9.9e-08: not > 1e-7
  c.currentStateCallback(e);
  EXPECT_EQ(c.commandCount(), 1);
  // 100 ns: 1e-9 * 100 = 1.0000000000000001e-07 in double, which IS > 1e-7
  // (the literal 0.0000001), so the step runs, as in the original.
  e.header.stamp.nsec += 100;
  c.currentStateCallback(e);
  EXPECT_EQ(c.commandCount(), 2);
}

TEST(Parameters, OutOfRangeAndNonFiniteGainsAreErrors)
{
  ControllerParameters p = quad();
  EXPECT_TRUE(parameterErrors(p).empty());
  p.gains.uP = 2.5;                     // Gains.cfg max 2
  p.gains.alpha = std::nan("");
  p.max_roll = -0.1;
  const auto e = parameterErrors(p);
  ASSERT_EQ(e.size(), 3u);
  EXPECT_NE(e[0].find("uP"), std::string::npos);
  EXPECT_NE(e[1].find("alpha"), std::string::npos);
  EXPECT_NE(e[2].find("max_roll"), std::string::npos);
}

TEST(FirmwareMux, OffboardTimeoutAndIgnoreBits)
{
  FirmwareMux m(100);
  EXPECT_EQ(m.motors(), "off (disarmed)");
  m.setArmed(true);
  m.command(1000, 2, 0x07, {0.1f, -0.1f, 0.5f, 0.6f});   // altitude-only: x, y, z ignored
  EXPECT_EQ(m.source(0), "rc");
  EXPECT_EQ(m.source(2), "rc");
  EXPECT_EQ(m.source(3), "offboard");
  EXPECT_FALSE(m.tick(1100));                             // not "> stamp + 100" yet
  EXPECT_EQ(m.source(3), "offboard");
  EXPECT_TRUE(m.tick(1101));
  EXPECT_EQ(m.source(3), "rc");
  EXPECT_EQ(m.motors(), "rc throttle");
  EXPECT_EQ(m.timeouts(), 1);
  m.command(1200, 1, 0, {0, 0, 0, 0});                    // rate mode: reef_control never sends it
  EXPECT_EQ(m.motors(), "unsupported mode");
}

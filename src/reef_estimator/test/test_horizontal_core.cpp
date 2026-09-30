// Horizontal filter: body-level frame transformations checked against an
// independent construction (Eigen AngleAxis rotations, not the code's
// formula), with nonzero yaw and tilt; observation accounting; C1.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "reef_estimator/xyz_estimator.h"

using namespace reef_estimator;

namespace
{

constexpr double kG = 9.81;
constexpr double kDeg = M_PI / 180.0;

// R_NED<-body for 3-2-1 angles, built independently of reef_msgs.
Eigen::Matrix3d body_to_ned(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Specific force in the body frame for a kinematic acceleration given in the
// BODY-LEVEL frame (NED rotated by yaw): f = C (R_z a_level - g_ned).
Eigen::Vector3d specific_force(const Eigen::Vector3d & a_level, double roll, double pitch, double yaw)
{
  const Eigen::Matrix3d C = body_to_ned(roll, pitch, yaw).transpose();
  const Eigen::Vector3d a_ned = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * a_level;
  return C * (a_ned - Eigen::Vector3d(0, 0, kG));
}

EstimatorParameters params()
{
  EstimatorParameters p;
  p.xy_x0.setZero();
  p.xy_P0 = (Eigen::Matrix<double, 6, 1>() << 0.01, 0.01, 0.0031, 0.0031, 0.141, 0.141).finished().asDiagonal();
  p.xy_Q = (Eigen::Matrix<double, 6, 1>() << 0, 0, 0.03, 0.03, 0.1, 0.1).finished().asDiagonal();
  p.xy_R0 = Eigen::Vector2d(0.02, 0.02).asDiagonal();
  p.xy_beta << 1, 1, 0.03, 0.03, 0.001, 0.001;
  p.z_x0 << -0.25, 0, 0;
  p.z_P0 = Eigen::Vector3d(0.025, 1.0, 0.01).asDiagonal();
  p.z_P0_flying = Eigen::Vector3d(0.25, 1.0, 0.01).asDiagonal();
  p.z_Q = Eigen::Vector2d(0.03, 0.0001).asDiagonal();
  p.z_R0 << 0.04;
  p.z_R_flying << 0.00016;
  p.z_beta << 1, 1, 0.5;
  p.enable_rgbd = false;       // mocap velocity selected
  p.enable_mocap_z = false;
  return p;
}

struct Attitude { double roll, pitch, yaw; };

const std::vector<Attitude> kAttitudes = {
  {0, 0, 0}, {0, 0, 30 * kDeg}, {0, 0, 90 * kDeg}, {0, 0, -135 * kDeg},
  {10 * kDeg, -5 * kDeg, 30 * kDeg}, {-20 * kDeg, 15 * kDeg, 90 * kDeg},
  {15 * kDeg, 20 * kDeg, -135 * kDeg}, {-8 * kDeg, -20 * kDeg, 170 * kDeg},
};

}  // namespace

TEST(HorizontalFrames, OnePropagationStepGivesBodyLevelAccelerationTimesDt)
{
  const Eigen::Vector3d a_level(0.4, -0.3, 0.2);   // body-level x, y, and down
  for (const auto & att : kAttitudes) {
    XYEstimator xy;
    xy.xHat0 = Eigen::MatrixXd::Zero(6, 1);
    xy.P0 = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    xy.R0 = Eigen::MatrixXd::Identity(2, 2) * 0.02;
    xy.Q = Eigen::MatrixXd::Zero(6, 6);
    xy.betaVector = Eigen::MatrixXd::Ones(6, 1);
    xy.initialize();
    xy.dt = 0.004;
    Eigen::Matrix3d C = body_to_ned(att.roll, att.pitch, att.yaw).transpose();
    xy.nonlinearPropagation(C, kG, specific_force(a_level, att.roll, att.pitch, att.yaw), 0.0f);
    EXPECT_NEAR(xy.xHat(0), a_level(0) * 0.004, 1e-12) << att.roll << " " << att.pitch << " " << att.yaw;
    EXPECT_NEAR(xy.xHat(1), a_level(1) * 0.004, 1e-12) << att.roll << " " << att.pitch << " " << att.yaw;
    // roll/pitch recovered from C must be the generating angles (yaw removed)
    EXPECT_NEAR(xy.roll, att.roll, 1e-12);
    EXPECT_NEAR(xy.pitch, att.pitch, 1e-12);
  }
}

TEST(HorizontalFrames, SameThroughTheQuaternionInputOfTheCombinedEstimator)
{
  const Eigen::Vector3d a_level(-0.25, 0.35, 0.0);
  for (const auto & att : kAttitudes) {
    XYZEstimator e(params());
    const Eigen::Quaterniond q(body_to_ned(att.roll, att.pitch, att.yaw));
    const Eigen::Vector3d f_rest = specific_force(Eigen::Vector3d::Zero(), att.roll, att.pitch, att.yaw);
    const Eigen::Vector3d f = specific_force(a_level, att.roll, att.pitch, att.yaw);
    ImuSample s;
    s.qx = q.x(); s.qy = q.y(); s.qz = q.z(); s.qw = q.w();
    for (int k = 0; k <= 20; k++) {   // 20 initialization samples, then one propagation
      s.stamp.sec = 1;
      s.stamp.nanosec = static_cast<uint32_t>(k * 4000000);
      const Eigen::Vector3d & ff = (k < 20) ? f_rest : f;
      s.ax = ff(0); s.ay = ff(1); s.az = ff(2);
      e.sensorUpdate(s);
    }
    const double dt = e.xyFilter().dt;
    EXPECT_NEAR(e.xyFilter().xHat(0), a_level(0) * dt, 1e-12) << att.yaw;
    EXPECT_NEAR(e.xyFilter().xHat(1), a_level(1) * dt, 1e-12) << att.yaw;
  }
}

TEST(HorizontalFrames, RelabellingAxesIsNotEnough)
{
  // With yaw 90 deg a body-level +x acceleration points east. Feeding the NED
  // acceleration as if it were body-level (a relabelling) gives the wrong axis.
  const double yaw = 90 * kDeg;
  const Eigen::Vector3d a_level(0.5, 0.0, 0.0);
  const Eigen::Vector3d a_ned = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * a_level;
  EXPECT_NEAR(a_ned(1), 0.5, 1e-15);   // east
  XYEstimator xy;
  xy.xHat0 = Eigen::MatrixXd::Zero(6, 1);
  xy.P0 = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  xy.R0 = Eigen::MatrixXd::Identity(2, 2) * 0.02;
  xy.Q = Eigen::MatrixXd::Zero(6, 6);
  xy.betaVector = Eigen::MatrixXd::Ones(6, 1);
  xy.initialize();
  xy.dt = 0.01;
  Eigen::Matrix3d C = body_to_ned(0, 0, yaw).transpose();
  xy.nonlinearPropagation(C, kG, specific_force(a_level, 0, 0, yaw), 0.0f);
  EXPECT_NEAR(xy.xHat(0), 0.005, 1e-12);   // forward, not east
  EXPECT_NEAR(xy.xHat(1), 0.0, 1e-12);
}

namespace
{
void fly(XYZEstimator & e, int steps, double vx_obs, bool observe = true)
{
  ImuSample s;
  TwistSample tw;
  tw.cov_xx = tw.cov_yy = 0.02;
  for (int k = 0; k < steps; k++) {
    s.stamp.sec = 1 + k / 250;
    s.stamp.nanosec = static_cast<uint32_t>((k % 250) * 4000000);
    s.az = -kG;
    if (observe && k % 5 == 0) {
      tw.stamp = s.stamp;
      tw.vx = vx_obs;
      e.mocapUpdate(tw);
    }
    e.sensorUpdate(s);
  }
}
}  // namespace

TEST(Observations, D1ReFusesTheLastObservationAtEveryStep)
{
  EstimatorParameters p = params();
  p.correction_c1 = false;   // legacy master (C1 is the default since R1)
  XYZEstimator e(p);
  fly(e, 200, 0.0);
  const long accepted = e.xyObservationsAccepted();
  EXPECT_GT(accepted, 0);
  EXPECT_GT(e.xyFusions(), 5 * (accepted - 5));   // about one fusion per IMU step
}

TEST(Observations, C1FusesEachObservationAtMostOnce)
{
  EstimatorParameters p = params();
  EXPECT_TRUE(p.correction_c1);   // default since R1
  XYZEstimator e(p);
  fly(e, 200, 0.0);
  EXPECT_TRUE(e.correctionC1());
  EXPECT_LE(e.xyFusions(), e.xyObservationsAccepted());
  EXPECT_GE(e.xyFusions(), e.xyObservationsAccepted() - 4);   // up to 4 superseded during initialization
  EXPECT_FALSE(e.pendingXYMeasurement());
}

TEST(Observations, OutlierIsRejectedByTheMocapGate)
{
  XYZEstimator e(params());
  fly(e, 100, 0.0);
  const long accepted = e.xyObservationsAccepted();
  TwistSample tw;
  tw.vx = 5.0;
  tw.cov_xx = tw.cov_yy = 0.02;
  e.mocapUpdate(tw);
  EXPECT_EQ(e.xyObservationsAccepted(), accepted);
  EXPECT_GT(e.lastMahalanobisSquared(), 20.0);
}

TEST(Observations, RgbdIsIgnoredWhileMocapIsSelected)
{
  XYZEstimator e(params());   // enable_rgbd false -> useMocapXY
  EXPECT_TRUE(e.usingMocapXY());
  TwistSample tw;
  tw.cov_xx = tw.cov_yy = 0.02;
  e.rgbdUpdate(tw);
  EXPECT_EQ(e.xyGateCount(), 0);
}

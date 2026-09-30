// validate()/warnings() without ROS.
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "reef_estimator/parameters.hpp"

using reef_estimator::EstimatorParameters;
using reef_estimator::validate;

namespace
{

// The values shipped with master e4179f48 (config/estimator_master.yaml).
EstimatorParameters master()
{
  EstimatorParameters p;
  p.xy_x0.setZero();
  p.xy_P0 = (Eigen::Matrix<double, 6, 1>() << 0.01, 0.01, 0.0031, 0.0031, 0.141, 0.141)
    .finished().asDiagonal();
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
  return p;
}

bool any_contains(const std::vector<std::string> & v, const std::string & part)
{
  for (const auto & s : v) {
    if (s.find(part) != std::string::npos) {return true;}
  }
  return false;
}

}  // namespace

TEST(Validate, MasterValuesAreValid)
{
  EXPECT_EQ(validate(master()), std::vector<std::string>{});
}

TEST(Validate, DefaultsMatchLegacyCode)
{
  const EstimatorParameters p;
  EXPECT_FALSE(p.debug_mode);
  EXPECT_TRUE(p.enable_xy && p.enable_z && p.enable_mocap_xy && p.enable_rgbd &&
    p.enable_mocap_z && p.enable_sonar && p.enable_partial_update && p.enable_measurements);
  EXPECT_FALSE(p.enable_mocap_switch);
  EXPECT_EQ(p.mocap_override_channel, 4);
  EXPECT_EQ(p.mahalanobis_d_sonar, 20.0);
  EXPECT_EQ(p.mahalanobis_d_rgbd_velocity, 20.0);
  EXPECT_EQ(p.mahalanobis_d_mocap_z, 20.0);
  EXPECT_EQ(p.mahalanobis_d_mocap_velocity, 20.0);
  EXPECT_EQ(p.estimator_dt, 0.002);
  EXPECT_EQ(p.mocap_twist_topic, "mocap_velocity/body_level_frame");
  EXPECT_EQ(p.mocap_pose_topic, "mocap_ned");
  EXPECT_EQ(p.rgbd_twist_topic, "rgbd_velocity_body_frame");
}

TEST(Validate, UnloadedMatricesAreRejected)
{
  const auto e = validate(EstimatorParameters{});
  EXPECT_EQ(e.size(), 12u);
  EXPECT_TRUE(any_contains(e, "xy_x0[0] is not finite"));
  EXPECT_TRUE(any_contains(e, "z_R_flying(0,0) is not finite"));
}

TEST(Validate, OverrideChannelMustIndexRcRaw)
{
  for (int ch : {-1, 8, 100}) {
    auto p = master();
    p.mocap_override_channel = ch;
    EXPECT_TRUE(any_contains(validate(p), "mocap_override_channel")) << ch;
  }
  for (int ch : {0, 4, 6, 7}) {
    auto p = master();
    p.mocap_override_channel = ch;
    EXPECT_TRUE(validate(p).empty()) << ch;
  }
}

TEST(Validate, GatesMustBePositiveInfinityAllowed)
{
  auto p = master();
  p.mahalanobis_d_sonar = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(validate(p).empty());
  for (double bad : {0.0, -20.0, std::numeric_limits<double>::quiet_NaN()}) {
    p.mahalanobis_d_sonar = bad;
    EXPECT_TRUE(any_contains(validate(p), "mahalanobis_d_sonar")) << bad;
  }
}

TEST(Validate, EstimatorDtMustBeFinitePositive)
{
  for (double bad : {0.0, -0.002, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto p = master();
    p.estimator_dt = bad;
    EXPECT_TRUE(any_contains(validate(p), "estimator_dt")) << bad;
  }
}

TEST(Validate, CovariancesAndWeights)
{
  auto p = master();
  p.z_P0(1, 1) = -1.0;
  p.xy_R0(0, 1) = 0.001;   // asymmetric
  p.xy_beta(2) = 1.5;
  p.z_beta(0) = -0.1;
  const auto e = validate(p);
  EXPECT_TRUE(any_contains(e, "z_P0(1,1) is negative"));
  EXPECT_TRUE(any_contains(e, "xy_R0 is not symmetric"));
  EXPECT_TRUE(any_contains(e, "xy_beta[2]"));
  EXPECT_TRUE(any_contains(e, "z_beta[0]"));
  EXPECT_EQ(e.size(), 4u);
}

TEST(Validate, EmptyTopicRejected)
{
  auto p = master();
  p.mocap_pose_topic.clear();
  EXPECT_TRUE(any_contains(validate(p), "mocap_pose_topic is empty"));
}

TEST(Warnings, ShippedHardwareConfigurationIgnoresRgbd)
{
  auto p = master();
  p.enable_measurements = false;   // as in basic_params.yaml
  EXPECT_TRUE(any_contains(reef_estimator::warnings(p), "RGB-D messages are ignored"));
  p.enable_xy = true;
  p.enable_mocap_xy = false;
  p.enable_rgbd = false;
  EXPECT_TRUE(any_contains(reef_estimator::warnings(p), "horizontal filter only propagates"));
}

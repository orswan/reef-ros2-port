// ROS 2 message side: the geometry_msgs overload of quaternion_to_rotation
// (the one reef_estimator calls with imu.orientation) against the legacy
// vectors, and the generated C++ message layout.
#include <gtest/gtest.h>

#include <array>
#include <type_traits>

#include <geometry_msgs/msg/quaternion.hpp>

#include "legacy_vectors.hpp"
#include "reef_msgs/matrix_operation.h"
#include "reef_msgs/msg/delta_to_vel.hpp"
#include "reef_msgs/msg/xyz_debug_estimate.hpp"
#include "reef_msgs/msg/xyz_estimate.hpp"
#include "reef_msgs/ros_conversions.hpp"

// Fixed-size arrays of the legacy messages stay fixed-size (std::array).
static_assert(std::is_same_v<decltype(reef_msgs::msg::ZDebugEstimate::p), std::array<double, 9>>);
static_assert(std::is_same_v<decltype(reef_msgs::msg::ZDebugEstimate::truth), std::array<double, 2>>);
static_assert(std::is_same_v<decltype(reef_msgs::msg::ZDebugEstimate::sigma_plus), std::array<double, 3>>);
static_assert(std::is_same_v<decltype(reef_msgs::msg::XYDebugEstimate::sigma_minus), std::array<double, 6>>);
static_assert(std::is_same_v<decltype(reef_msgs::msg::DeltaToVel::s_upper_bound), std::array<double, 6>>);
static_assert(std::is_same_v<decltype(reef_msgs::msg::DeltaToVel::scaled_std_xyz), std::array<double, 3>>);
static_assert(std::is_same_v<decltype(reef_msgs::msg::XYZEstimate::node_id), uint32_t>);

TEST(RosConversions, GeometryMsgsQuaternionOverloadIsBitIdenticalToLegacy)
{
  int checked = 0;
  for (const auto & c : legacy::load(LEGACY_VECTORS)) {
    if (c.kind != "Q2R") {continue;}
    geometry_msgs::msg::Quaternion q;
    q.x = c.in[0];
    q.y = c.in[1];
    q.z = c.in[2];
    q.w = c.in[3];
    const Eigen::Matrix3d C = reef_msgs::quaternion_to_rotation(q);
    for (int k = 0; k < 9; k++) {
      ASSERT_TRUE(legacy::same(C(k / 3, k % 3), c.out[k])) << "case " << checked << " element " << k;
    }
    checked++;
  }
  EXPECT_GE(checked, 1000);
}

TEST(RosConversions, CovarianceFillsZDebugEstimateRowMajor)
{
  Eigen::Matrix3d P;
  P << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  reef_msgs::msg::ZDebugEstimate z;
  ASSERT_TRUE(reef_msgs::matrixToArray(P, z.p));
  EXPECT_EQ(z.p[1], 2);
  EXPECT_EQ(z.p[3], 4);
}

TEST(RosConversions, DefaultsAreZeroAndHeaderIsEmpty)
{
  // Fields the estimator never sets must read as zero/empty, not as data.
  reef_msgs::msg::XYZDebugEstimate d;
  EXPECT_EQ(d.node_id, 0u);
  EXPECT_EQ(d.z_plus.truth[0], 0.0);
  EXPECT_EQ(d.z_plus.z_error, 0.0);
  EXPECT_TRUE(d.header.frame_id.empty());
}

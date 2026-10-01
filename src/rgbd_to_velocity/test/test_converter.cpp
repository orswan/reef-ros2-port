// Converter tests with expected values that do not come from the code:
// DEMO's camera convention (x left, y up, z forward) and the shipped kiwi
// extrinsics (camera looking forward): forward camera motion is +x in the
// body-level frame, leftward motion -y (FRD: y right); the gate boundary
// (1/30 s) from the stamps; parameter rejection (VISION.md Q10).
#include <gtest/gtest.h>

#include "rgbd_to_velocity/rgbd_to_velocity.h"

using namespace rgbd_to_velocity;

namespace
{
ConverterParameters kiwi()
{
  ConverterParameters p;
  p.body_to_camera_quat = {0.5495504171301594, -0.4573648426502658, 0.5078083888936552, -0.4805646469610469};
  p.body_to_camera_trans = {0.1354765750332004, 0.0175314092839309, -0.0455417178618858};
  return p;
}
Odometry at(int64_t sec, uint32_t nsec, double x, double y, double z)
{
  Odometry o;
  o.header.stamp.sec = sec;
  o.header.stamp.nsec = nsec;
  o.pose.pose.position = {x, y, z};
  o.pose.pose.orientation = {0, 0, 0, 1};
  return o;
}
}  // namespace

TEST(Signs, ForwardCameraMotionIsForwardBodyVelocity)
{
  RgbdToVelocity c(kiwi());
  c.poseCallback(at(100, 0, 0, 0, 0));
  c.poseCallback(at(100, 50000000, 0, 0, 0.05));   // 1 m/s along the optical axis
  EXPECT_GT(c.vel_msg.vel.twist.twist.linear.x, 0.95);
  EXPECT_LT(std::abs(c.vel_msg.vel.twist.twist.linear.y), 0.1);
}

TEST(Signs, LeftwardCameraMotionIsNegativeBodyY)
{
  RgbdToVelocity c(kiwi());
  c.poseCallback(at(100, 0, 0, 0, 0));
  c.poseCallback(at(100, 50000000, 0.05, 0, 0));   // DEMO x = left
  EXPECT_LT(c.vel_msg.vel.twist.twist.linear.y, -0.95);
}

TEST(Gate, SpacingBelowOneThirtiethIsRejected)
{
  RgbdToVelocity c(kiwi());
  c.poseCallback(at(100, 0, 0, 0, 0));
  c.poseCallback(at(100, 33333333, 0, 0, 0.01));   // 33.333333 ms < 1/30 s
  EXPECT_EQ(c.bodyLevelPublished, 1);
  c.poseCallback(at(100, 33333334, 0, 0, 0.01));   // 33.333334 ms >= 1/30 s
  EXPECT_EQ(c.bodyLevelPublished, 2);
}

TEST(Parameters, ExtrinsicsAreRequiredAndCovariancesNonNegative)
{
  ConverterParameters p = kiwi();
  EXPECT_TRUE(parameterErrors(p).empty());
  p.body_to_camera_quat = {0, 0, 1};
  p.x_vel_covariance = -0.1;
  const auto e = parameterErrors(p);
  ASSERT_EQ(e.size(), 2u);
  EXPECT_NE(e[0].find("x_vel_covariance"), std::string::npos);
  EXPECT_NE(e[1].find("body_to_camera_quat"), std::string::npos);
}

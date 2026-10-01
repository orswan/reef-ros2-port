// Odometry core on synthetic frames with known motion: a fronto-parallel
// textured plane at depth Z; a camera translation (tx, ty) in the optical
// frame shifts the image by (-fx tx / Z, -fy ty / Z) px. Expected values come
// from that geometry, not from the code. Also: weak texture and depth loss
// give LOST and no pose; DEMO's convention (x left, y up, z forward).
#include <gtest/gtest.h>

#include <random>

#include <opencv2/imgproc.hpp>

#include "reef_rgbd_odometry/odometry.hpp"

using namespace reef_rgbd_odometry;

namespace
{
const Intrinsics K{277.128, 277.128, 160.0, 120.0};
const double Z = 3.8;

cv::Mat texture()
{
  std::mt19937 rng(3);
  cv::Mat big(480, 640, CV_8UC1, cv::Scalar(128));
  std::uniform_int_distribution<int> x(0, 639), y(0, 479), c(0, 255);
  for (int size : {40, 20, 10, 5}) {
    for (int i = 0; i < 4000 / size; ++i) {
      cv::rectangle(big, cv::Rect(x(rng), y(rng), size, size), cv::Scalar(c(rng)), -1);
    }
  }
  return big;
}

// The 320x240 view of the plane for a camera at (tx, ty) (optical frame, metres).
cv::Mat view(const cv::Mat & big, double tx, double ty)
{
  const double du = -K.fx * tx / Z, dv = -K.fy * ty / Z;
  cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, -160 + du, 0, 1, -120 + dv);
  cv::Mat out;
  cv::warpAffine(big, out, M, cv::Size(320, 240), cv::INTER_LINEAR);
  return out;
}

cv::Mat depth(float z) {return cv::Mat(240, 320, CV_32FC1, cv::Scalar(z));}
}  // namespace

TEST(Odometry, LateralAndVerticalTranslationRecovered)
{
  const cv::Mat big = texture();
  RgbdOdometry vo;
  ASSERT_TRUE(vo.process(view(big, 0, 0), depth(Z), K).publish);   // anchor
  const Result r = vo.process(view(big, 0.05, -0.03), depth(Z), K);
  ASSERT_EQ(r.state, State::Ok);
  EXPECT_TRUE(r.publish);
  EXPECT_NEAR(r.p.x(), 0.05, 0.003);
  EXPECT_NEAR(r.p.y(), -0.03, 0.003);
  EXPECT_NEAR(r.p.z(), 0.0, 0.01);
  EXPECT_GT(r.inliers, 100);
}

TEST(Odometry, WeakTextureIsLostAndNotPublished)
{
  const cv::Mat big = texture();
  RgbdOdometry vo;
  vo.process(view(big, 0, 0), depth(Z), K);
  const cv::Mat plain(240, 320, CV_8UC1, cv::Scalar(140));
  const Result r = vo.process(plain, depth(Z), K);
  EXPECT_EQ(r.state, State::Lost);
  EXPECT_FALSE(r.publish);
  // Back on texture: publishing resumes only after 2 consecutive good frames.
  vo.process(view(big, 0.01, 0), depth(Z), K);
  const Result a = vo.process(view(big, 0.02, 0), depth(Z), K);
  const Result b = vo.process(view(big, 0.03, 0), depth(Z), K);
  EXPECT_FALSE(a.publish);
  EXPECT_TRUE(b.publish);
  EXPECT_EQ(vo.lostEvents(), 1);
}

TEST(Odometry, InvalidDepthIsNeverUsed)
{
  const cv::Mat big = texture();
  RgbdOdometry vo;
  vo.process(view(big, 0, 0), depth(-std::numeric_limits<float>::infinity()), K);
  const Result r = vo.process(view(big, 0.02, 0), depth(Z), K);
  EXPECT_EQ(r.state, State::Lost);   // the previous frame had no valid depth
  EXPECT_EQ(r.depth_tracks, 0);
  EXPECT_FALSE(r.publish);
}

TEST(Convention, DemoAxesAreLeftUpForward)
{
  Eigen::Matrix3d Rd;
  Eigen::Vector3d pd;
  // Optical: x right, y down, z forward. A move right and down and forward:
  toDemo(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 2, 3), Rd, pd);
  EXPECT_EQ(pd, Eigen::Vector3d(-1, -2, 3));   // left -1, up -2, forward 3
  // A yaw to the right: +0.2 rad about the optical y axis (pointing DOWN) is
  // -0.2 rad about DEMO's y axis (pointing UP).
  const Eigen::Matrix3d Ry = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  toDemo(Ry, Eigen::Vector3d::Zero(), Rd, pd);
  EXPECT_TRUE(Rd.isApprox(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix()));
}

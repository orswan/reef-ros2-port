// attitude.hpp against an independent construction (matrices T R B) and the
// interpolation/extrapolation rules shared with reef_sim/reef_adapter.py.
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <random>

#include "reef_x3_adapter/attitude.hpp"

using reef_x3_adapter::Quat;

namespace
{
Eigen::Matrix3d R(const Quat & q) {return Eigen::Quaterniond(q[0], q[1], q[2], q[3]).toRotationMatrix();}
}

TEST(Attitude, MatchesTRB)
{
  Eigen::Matrix3d T;
  T << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Eigen::Matrix3d B = Eigen::Vector3d(1, -1, -1).asDiagonal();
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0, 1);
  for (int k = 0; k < 200; k++) {
    Eigen::Quaterniond e(n(rng), n(rng), n(rng), n(rng));
    e.normalize();
    const Quat q{e.w(), e.x(), e.y(), e.z()};
    const Quat r = reef_x3_adapter::ned_frd_from_enu_flu(q);
    EXPECT_GE(r[0], 0.0);
    EXPECT_TRUE(R(r).isApprox(T * R(q) * B, 1e-12));
  }
}

TEST(Attitude, InterpolatesAndExtrapolatesYawLinearly)
{
  reef_x3_adapter::TruthBuffer b;
  EXPECT_FALSE(b.at(5).has_value());
  b.add(10, {1, 0, 0, 0});
  EXPECT_FALSE(b.at(12).has_value());   // one truth sample is not enough
  b.add(20, {std::cos(0.1), 0, 0, std::sin(0.1)});   // ENU yaw 0.2
  auto yaw = [](const Quat & q) {return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));};
  const double base = yaw(reef_x3_adapter::ned_frd_from_enu_flu({1, 0, 0, 0}));
  EXPECT_NEAR(base - yaw(*b.at(15)), 0.1, 1e-12);
  EXPECT_NEAR(base - yaw(*b.at(30)), 0.4, 1e-12);
  EXPECT_NEAR(base - yaw(*b.at(20 + reef_x3_adapter::kMaxExtrapolationNs + 1)), 0.2, 1e-12);
}

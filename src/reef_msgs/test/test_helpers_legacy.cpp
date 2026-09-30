// Port vs legacy: the ported helpers must reproduce the recorded outputs of
// the pinned legacy reef_msgs 7fb63ff code bit for bit. ROS-free: this test
// links reef_msgs_helpers (Eigen) only.
#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <string>

#include "legacy_vectors.hpp"
#include "reef_msgs/dynamics.h"
#include "reef_msgs/matrix_operation.h"

namespace
{

const std::vector<legacy::Case> & cases()
{
  static const std::vector<legacy::Case> c = legacy::load(LEGACY_VECTORS);
  return c;
}

std::string hex(double v)
{
  char buf[64];
  std::snprintf(buf, sizeof buf, "%a", v);
  return buf;
}

int count(const std::string & kind)
{
  int n = 0;
  for (const auto & c : cases()) {
    n += c.kind == kind;
  }
  return n;
}

}  // namespace

TEST(LegacyVectors, CoverageMatchesAcceptanceCriteria)
{
  EXPECT_GE(count("Q2R"), 1000);
  EXPECT_GE(count("RPY"), 1000);
  EXPECT_GE(count("MAT"), 12 * 4);
  EXPECT_GE(count("ARR"), 1);
}

TEST(LegacyVectors, QuaternionToRotationIsBitIdentical)
{
  int checked = 0;
  for (const auto & c : cases()) {
    if (c.kind != "Q2R") {continue;}
    Eigen::Quaterniond q;
    q.x() = c.in[0]; q.y() = c.in[1]; q.z() = c.in[2]; q.w() = c.in[3];
    const Eigen::Matrix3d C = reef_msgs::quaternion_to_rotation(q);
    for (int k = 0; k < 9; k++) {
      ASSERT_TRUE(legacy::same(C(k / 3, k % 3), c.out[k]))
        << "q = (" << hex(c.in[0]) << ", " << hex(c.in[1]) << ", " << hex(c.in[2]) << ", "
        << hex(c.in[3]) << ") element " << k << ": port " << hex(C(k / 3, k % 3))
        << " legacy " << hex(c.out[k]);
    }
    checked++;
  }
  EXPECT_EQ(checked, count("Q2R"));
}

TEST(LegacyVectors, RollPitchYawFrom321IsBitIdentical)
{
  for (const auto & c : cases()) {
    if (c.kind != "RPY") {continue;}
    Eigen::Matrix3d C;
    for (int k = 0; k < 9; k++) {C(k / 3, k % 3) = c.in[k];}
    double r = 0, p = 0, y = 0;
    reef_msgs::roll_pitch_yaw_from_rotation321(C, r, p, y);
    const double got[3] = {r, p, y};
    for (int k = 0; k < 3; k++) {
      ASSERT_TRUE(legacy::same(got[k], c.out[k]))
        << "C02 = " << hex(c.in[2]) << " angle " << k << ": port " << hex(got[k])
        << " legacy " << hex(c.out[k]);
    }
  }
}

TEST(LegacyVectors, MatrixImportLegalFormsMatchAndIllegalFormsAreRejected)
{
  int legal = 0, rejected = 0;
  for (const auto & c : cases()) {
    if (c.kind != "MAT") {continue;}
    Eigen::MatrixXd m = Eigen::MatrixXd::Constant(c.rows, c.cols, -1.5);
    const reef_msgs::MatrixImport r = reef_msgs::importMatrixFromVector(m, c.in, c.name);
    if (c.status == 'F' || c.status == 'D') {
      ASSERT_TRUE(r.ok) << c.name << ": " << r.error;
      EXPECT_EQ(r.layout, c.status == 'F' ? reef_msgs::MatrixLayout::Full :
        reef_msgs::MatrixLayout::Diagonal) << c.name;
      for (int k = 0; k < c.rows * c.cols; k++) {
        ASSERT_TRUE(legacy::same(m(k / c.cols, k % c.cols), c.out[k])) << c.name << " element " << k;
      }
      legal++;
    } else {
      // Legacy zero-filled a missing parameter (Z) and left a wrong-sized
      // one uninitialized (U). The port rejects both and leaves m unchanged.
      ASSERT_FALSE(r.ok) << c.name << " status " << c.status;
      EXPECT_NE(r.error.find(c.name), std::string::npos) << r.error;
      EXPECT_TRUE((m.array() == -1.5).all()) << c.name;
      rejected++;
    }
  }
  EXPECT_GE(legal, 12 * 3);
  EXPECT_EQ(rejected, 12 * 2);
}

TEST(LegacyVectors, MatrixToArrayIsRowMajorLikeLegacy)
{
  for (const auto & c : cases()) {
    if (c.kind != "ARR") {continue;}
    Eigen::Matrix3d P;
    for (int k = 0; k < 9; k++) {P(k / 3, k % 3) = c.in[k];}
    std::array<double, 9> a{};
    ASSERT_TRUE(reef_msgs::matrixToArray(P, a));
    for (int k = 0; k < 9; k++) {
      ASSERT_TRUE(legacy::same(a[k], c.out[k])) << "element " << k;
    }
  }
}

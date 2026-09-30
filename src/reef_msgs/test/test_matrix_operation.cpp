// Matrix import and validation rules (ROS-free; links reef_msgs_helpers only).
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "reef_msgs/matrix_operation.h"

using reef_msgs::MatrixLayout;
using reef_msgs::importMatrixFromVector;

namespace
{
bool contains(const std::string & s, const std::string & part)
{
  return s.find(part) != std::string::npos;
}
}  // namespace

TEST(MatrixImport, FullIsRowMajor)
{
  Eigen::Matrix<double, 2, 3> m;
  const auto r = importMatrixFromVector(m, {1, 2, 3, 4, 5, 6}, "m");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.layout, MatrixLayout::Full);
  EXPECT_EQ(m(0, 2), 3);
  EXPECT_EQ(m(1, 0), 4);
}

TEST(MatrixImport, DiagonalOnSquareMatrixZeroesOffDiagonal)
{
  Eigen::Matrix3d m = Eigen::Matrix3d::Constant(7);
  const auto r = importMatrixFromVector(m, {1, 2, 3}, "m");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.layout, MatrixLayout::Diagonal);
  EXPECT_EQ(m, Eigen::Vector3d(1, 2, 3).asDiagonal().toDenseMatrix());
}

TEST(MatrixImport, ColumnVectorAndScalarUseTheFullRuleFirst)
{
  Eigen::Matrix<double, 6, 1> v;
  auto r = importMatrixFromVector(v, {1, 2, 3, 4, 5, 6}, "v");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.layout, MatrixLayout::Full);
  Eigen::Matrix<double, 1, 1> s;
  r = importMatrixFromVector(s, {0.04}, "s");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.layout, MatrixLayout::Full);
  EXPECT_EQ(s(0, 0), 0.04);
}

TEST(MatrixImport, DynamicMatrixKeepsItsSize)
{
  Eigen::MatrixXd m(2, 2);
  ASSERT_TRUE(importMatrixFromVector(m, {0.03, 0.0001}, "z_Q").ok);
  EXPECT_EQ(m.rows(), 2);
  EXPECT_EQ(m(1, 1), 0.0001);
  EXPECT_EQ(m(0, 1), 0.0);
}

TEST(MatrixImport, WrongLengthIsRejectedWithSizesAndMatrixUnchanged)
{
  Eigen::Matrix<double, 6, 6> m = Eigen::Matrix<double, 6, 6>::Constant(-1.5);
  const auto r = importMatrixFromVector(m, std::vector<double>(35, 0.1), "xy_P0");
  ASSERT_FALSE(r.ok);
  EXPECT_TRUE(contains(r.error, "xy_P0")) << r.error;
  EXPECT_TRUE(contains(r.error, "35")) << r.error;
  EXPECT_TRUE(contains(r.error, "36 values (6x6, row-major) or 6 values (diagonal)")) << r.error;
  EXPECT_TRUE((m.array() == -1.5).all());
}

TEST(MatrixImport, DiagonalRuleRequiresSquareMatrix)
{
  // Legacy wrote out of bounds here (3 values on a 3x2 matrix).
  Eigen::Matrix<double, 3, 2> m = Eigen::Matrix<double, 3, 2>::Constant(-1.5);
  const auto r = importMatrixFromVector(m, {1, 2, 3}, "m");
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE((m.array() == -1.5).all());
}

TEST(MatrixImport, EmptyIsRejected)
{
  Eigen::Matrix2d m;
  const auto r = importMatrixFromVector(m, {}, "xy_R0");
  ASSERT_FALSE(r.ok);
  EXPECT_TRUE(contains(r.error, "xy_R0 is empty")) << r.error;
}

TEST(MatrixImport, NonFiniteIsRejected)
{
  Eigen::Matrix2d m = Eigen::Matrix2d::Constant(-1.5);
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    const auto r = importMatrixFromVector(m, {0.02, bad}, "xy_R0");
    ASSERT_FALSE(r.ok);
    EXPECT_TRUE(contains(r.error, "xy_R0[1] is not finite")) << r.error;
    EXPECT_TRUE((m.array() == -1.5).all());
  }
}

TEST(MatrixOperation, VectorToDiagMatrixAndMatrixToArrayRejectMismatchedSizes)
{
  Eigen::Matrix3d m;
  EXPECT_FALSE(reef_msgs::vectorToDiagMatrix(m, {1, 2}));
  std::array<double, 8> a{};
  EXPECT_FALSE(reef_msgs::matrixToArray(m, a));
}

TEST(Covariance, AcceptsSymmetricNonNegativeDiagonal)
{
  Eigen::Matrix2d P;
  P << 0.02, 0.001, 0.001, 0.0;
  EXPECT_EQ(reef_msgs::covarianceError(P, "P"), "");
}

TEST(Covariance, RejectsNegativeDiagonal)
{
  Eigen::Matrix3d P = Eigen::Vector3d(0.025, -1.0, 0.01).asDiagonal();
  const std::string e = reef_msgs::covarianceError(P, "z_P0");
  EXPECT_TRUE(contains(e, "z_P0(1,1) is negative")) << e;
}

TEST(Covariance, RejectsAsymmetry)
{
  Eigen::Matrix2d P;
  P << 0.02, 0.001, 0.002, 0.02;
  const std::string e = reef_msgs::covarianceError(P, "xy_R0");
  EXPECT_TRUE(contains(e, "xy_R0 is not symmetric")) << e;
}

TEST(Covariance, RejectsNonSquareAndNonFinite)
{
  Eigen::Matrix<double, 2, 3> A = Eigen::Matrix<double, 2, 3>::Zero();
  EXPECT_TRUE(contains(reef_msgs::covarianceError(A, "A"), "not square"));
  Eigen::Matrix2d P = Eigen::Matrix2d::Identity();
  P(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(contains(reef_msgs::covarianceError(P, "P"), "not finite"));
}

TEST(Range, UnitIntervalForPartialUpdateWeights)
{
  Eigen::Matrix<double, 6, 1> beta;
  beta << 1, 1, 0.03, 0.03, 0.001, 0.001;
  EXPECT_EQ(reef_msgs::rangeError(beta, "xy_beta", 0.0, 1.0), "");
  beta(4) = 1.5;
  EXPECT_TRUE(contains(reef_msgs::rangeError(beta, "xy_beta", 0.0, 1.0), "xy_beta[4]"));
  beta(4) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(contains(reef_msgs::rangeError(beta, "xy_beta", 0.0, 1.0), "xy_beta[4]"));
}

TEST(AcceptedSizes, DescribesEachShape)
{
  EXPECT_EQ(reef_msgs::acceptedSizes(6, 1), "6 values (6x1, row-major)");
  EXPECT_EQ(reef_msgs::acceptedSizes(1, 1), "1 values (1x1, row-major)");
  EXPECT_EQ(reef_msgs::acceptedSizes(2, 2), "4 values (2x2, row-major) or 2 values (diagonal)");
}

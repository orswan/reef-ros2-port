// importMatrixFromParameter / getNumberArrayParameter on a real rclcpp node.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "reef_msgs/parameters.hpp"

using reef_msgs::MatrixLayout;
using reef_msgs::ParameterError;

namespace
{

class Parameters : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<rclcpp::Node> node(const std::vector<rclcpp::Parameter> & overrides)
  {
    return std::make_shared<rclcpp::Node>(
      "reef_msgs_parameters_test", rclcpp::NodeOptions().parameter_overrides(overrides));
  }

  // Runs the import and returns the error message ("" if it succeeded).
  template<class M>
  std::string error_of(rclcpp::Node & n, M & m, const std::string & name)
  {
    try {
      reef_msgs::importMatrixFromParameter(*n.get_node_parameters_interface(), m, name);
    } catch (const ParameterError & e) {
      return e.what();
    }
    return "";
  }
};

bool contains(const std::string & s, const std::string & part)
{
  return s.find(part) != std::string::npos;
}

}  // namespace

TEST_F(Parameters, DoubleArrayFullAndDiagonal)
{
  auto n = node({rclcpp::Parameter("full", std::vector<double>{0.02, 0.0, 0.0, 0.02}),
      rclcpp::Parameter("diag", std::vector<double>{0.03, 0.0001})});
  Eigen::Matrix2d full, diag;
  EXPECT_EQ(reef_msgs::importMatrixFromParameter(*n->get_node_parameters_interface(), full, "full"),
    MatrixLayout::Full);
  EXPECT_EQ(reef_msgs::importMatrixFromParameter(*n->get_node_parameters_interface(), diag, "diag"),
    MatrixLayout::Diagonal);
  EXPECT_EQ(full(1, 1), 0.02);
  EXPECT_EQ(diag(1, 1), 0.0001);
  EXPECT_EQ(diag(0, 1), 0.0);
}

TEST_F(Parameters, IntegerArrayConvertsExactly)
{
  // Legacy YAML such as `xy_x0: [0, 0, 0, 0, 0, 0]` is an integer array in ROS 2.
  auto n = node({rclcpp::Parameter("xy_x0", std::vector<int64_t>{0, 0, 0, 0, 0, -3})});
  Eigen::Matrix<double, 6, 1> v;
  EXPECT_EQ(error_of(*n, v, "xy_x0"), "");
  EXPECT_EQ(v(5), -3.0);
}

TEST_F(Parameters, IntegerNotExactlyRepresentableIsRejected)
{
  auto n = node({rclcpp::Parameter("z_R0", std::vector<int64_t>{(int64_t{1} << 53) + 1})});
  Eigen::Matrix<double, 1, 1> m;
  EXPECT_TRUE(contains(error_of(*n, m, "z_R0"), "not exactly representable"));
}

TEST_F(Parameters, MissingIsRejectedWithAcceptedSizes)
{
  auto n = node({});
  Eigen::Matrix<double, 6, 6> m = Eigen::Matrix<double, 6, 6>::Constant(-1.5);
  const std::string e = error_of(*n, m, "xy_P0");
  EXPECT_TRUE(contains(e, "xy_P0 is not set")) << e;
  EXPECT_TRUE(contains(e, "36 values (6x6, row-major) or 6 values (diagonal)")) << e;
  EXPECT_TRUE((m.array() == -1.5).all());
}

TEST_F(Parameters, WrongLengthIsRejected)
{
  auto n = node({rclcpp::Parameter("z_Q", std::vector<double>{0.03, 0.0001, 0.1})});
  Eigen::Matrix2d m = Eigen::Matrix2d::Constant(-1.5);
  const std::string e = error_of(*n, m, "z_Q");
  EXPECT_TRUE(contains(e, "z_Q has 3 values; expected 4 values (2x2, row-major) or 2 values (diagonal)")) << e;
  EXPECT_TRUE((m.array() == -1.5).all());
}

TEST_F(Parameters, EmptyIsRejected)
{
  auto n = node({rclcpp::Parameter("z_beta", std::vector<double>{})});
  Eigen::Vector3d m;
  EXPECT_TRUE(contains(error_of(*n, m, "z_beta"), "z_beta is empty"));
}

TEST_F(Parameters, WrongTypesAreRejected)
{
  auto n = node({rclcpp::Parameter("as_string", std::string("0.02")),
      rclcpp::Parameter("as_bool", true),
      rclcpp::Parameter("as_double", 0.02),
      rclcpp::Parameter("as_strings", std::vector<std::string>{"0.02", "0.02"})});
  Eigen::Matrix<double, 1, 1> m;
  EXPECT_TRUE(contains(error_of(*n, m, "as_string"), "as_string has type string"));
  EXPECT_TRUE(contains(error_of(*n, m, "as_bool"), "as_bool has type bool"));
  // A scalar is not a list (legacy getParam(vector<double>) also refused it).
  EXPECT_TRUE(contains(error_of(*n, m, "as_double"), "as_double has type double"));
  Eigen::Matrix<double, 2, 1> v;
  EXPECT_TRUE(contains(error_of(*n, v, "as_strings"), "as_strings has type string_array"));
}

TEST_F(Parameters, NonFiniteIsRejected)
{
  auto n = node({rclcpp::Parameter("xy_R0",
      std::vector<double>{0.02, std::numeric_limits<double>::quiet_NaN()})});
  Eigen::Matrix2d m;
  EXPECT_TRUE(contains(error_of(*n, m, "xy_R0"), "xy_R0[1] is not finite"));
}

TEST_F(Parameters, DeclaredReadOnlyWithDescription)
{
  auto n = node({rclcpp::Parameter("xy_R0", std::vector<double>{0.02, 0.02})});
  Eigen::Matrix2d m;
  ASSERT_EQ(error_of(*n, m, "xy_R0"), "");
  const auto d = n->describe_parameter("xy_R0");
  EXPECT_TRUE(d.read_only);
  EXPECT_TRUE(contains(d.description, "4 values (2x2, row-major) or 2 values (diagonal)"));
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("xy_R0", std::vector<double>{1, 1})).successful);
}

TEST_F(Parameters, ParamsFileWithDoubleAndIntegerLists)
{
  const std::string path = testing::TempDir() + "reef_msgs_params.yaml";
  std::ofstream(path) <<
    "/**:\n"
    "  ros__parameters:\n"
    "    z_P0: [0.025, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01]\n"
    "    z_x0: [0, 0, 0]\n"
    "    z_R0: [0.04]\n";
  auto n = std::make_shared<rclcpp::Node>("reef_msgs_parameters_test",
      rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", path}));
  Eigen::Matrix3d P;
  Eigen::Vector3d x;
  Eigen::Matrix<double, 1, 1> R;
  EXPECT_EQ(error_of(*n, P, "z_P0"), "");
  EXPECT_EQ(error_of(*n, x, "z_x0"), "");
  EXPECT_EQ(error_of(*n, R, "z_R0"), "");
  EXPECT_EQ(P(1, 1), 1.0);
  EXPECT_EQ(R(0, 0), 0.04);
}

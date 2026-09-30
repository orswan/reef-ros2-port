// loadParameters() on a real rclcpp node: the shipped configuration, the
// simulation overrides, and every rejection path.
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "reef_estimator/ros_parameters.hpp"

using reef_estimator::EstimatorParameters;
using reef_estimator::loadParameters;

namespace
{

const std::string kMaster = std::string(CONFIG_DIR) + "/estimator_master.yaml";
const std::string kSim = std::string(CONFIG_DIR) + "/simulation.yaml";

class RosParameters : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<rclcpp::Node> node(
    const std::vector<std::string> & files, const std::vector<rclcpp::Parameter> & overrides = {})
  {
    std::vector<std::string> args{"--ros-args"};
    for (const auto & f : files) {
      args.insert(args.end(), {"--params-file", f});
    }
    return std::make_shared<rclcpp::Node>("reef_estimator",
             rclcpp::NodeOptions().arguments(args).parameter_overrides(overrides));
  }

  // Loads and returns the error text ("" on success).
  static std::string error_of(rclcpp::Node & n)
  {
    try {
      loadParameters(*n.get_node_parameters_interface());
    } catch (const reef_msgs::ParameterError & e) {
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

TEST_F(RosParameters, MasterConfigurationLoadsWithShippedValues)
{
  auto n = node({kMaster});
  const EstimatorParameters p = loadParameters(*n->get_node_parameters_interface());
  EXPECT_TRUE(p.debug_mode);
  EXPECT_TRUE(p.enable_mocap_switch);
  EXPECT_FALSE(p.enable_measurements);
  EXPECT_EQ(p.mocap_override_channel, 6);
  EXPECT_EQ(p.mahalanobis_d_rgbd_velocity, 80.0);
  EXPECT_EQ(p.mahalanobis_d_mocap_velocity, 50.0);
  EXPECT_EQ(p.estimator_dt, 0.002);   // not in the shipped files: code default
  EXPECT_EQ(p.xy_P0(4, 4), 0.141);
  EXPECT_EQ(p.xy_P0(4, 3), 0.0);
  EXPECT_EQ(p.xy_Q(2, 2), 0.03);
  EXPECT_EQ(p.xy_beta(4), 0.001);
  EXPECT_EQ(p.z_x0(0), -0.25);
  EXPECT_EQ(p.z_Q(0, 0), 0.03);       // 2 values on 2x2: diagonal
  EXPECT_EQ(p.z_Q(1, 1), 0.0001);
  EXPECT_EQ(p.z_Q(0, 1), 0.0);
  EXPECT_EQ(p.z_R_flying(0, 0), 0.00016);
  EXPECT_EQ(p.z_beta(2), 0.5);
}

TEST_F(RosParameters, SimulationDisablesTheRcSwitchExplicitly)
{
  auto n = node({kMaster, kSim});
  const EstimatorParameters p = loadParameters(*n->get_node_parameters_interface());
  EXPECT_FALSE(p.enable_mocap_switch);
  EXPECT_EQ(p.mocap_override_channel, 6);   // unchanged, but unused
}

TEST_F(RosParameters, DescriptorsAreReadOnlyExceptEnableMeasurements)
{
  auto n = node({kMaster});
  loadParameters(*n->get_node_parameters_interface());
  EXPECT_TRUE(n->describe_parameter("xy_P0").read_only);
  EXPECT_TRUE(n->describe_parameter("enable_mocap_switch").read_only);
  EXPECT_FALSE(n->describe_parameter("enable_measurements").read_only);
  EXPECT_TRUE(n->set_parameter(rclcpp::Parameter("enable_measurements", true)).successful);
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("enable_measurements", 1)).successful);
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("debug_mode", false)).successful);
  const auto ch = n->describe_parameter("mocap_override_channel");
  ASSERT_EQ(ch.integer_range.size(), 1u);
  EXPECT_EQ(ch.integer_range[0].from_value, 0);
  EXPECT_EQ(ch.integer_range[0].to_value, 7);
}

TEST_F(RosParameters, IntegerAcceptedForDoubleParameters)
{
  // basic_params.yaml writes gates as integers (20, 80, 50); roscpp accepted them.
  auto n = node({kMaster}, {rclcpp::Parameter("mahalanobis_d_sonar", 25)});
  EXPECT_EQ(loadParameters(*n->get_node_parameters_interface()).mahalanobis_d_sonar, 25.0);
}

TEST_F(RosParameters, OverrideChannelOutOfRangeIsRejected)
{
  for (int64_t ch : {-1, 8}) {
    auto n = node({kMaster}, {rclcpp::Parameter("mocap_override_channel", ch)});
    EXPECT_TRUE(contains(error_of(*n), "mocap_override_channel: invalid value")) << ch;
  }
}

TEST_F(RosParameters, WrongScalarTypesAreRejected)
{
  auto n = node({kMaster}, {
      rclcpp::Parameter("mocap_override_channel", 6.0),   // roscpp silently used the default
      rclcpp::Parameter("enable_sonar", 1),
      rclcpp::Parameter("mocap_pose_topic", 3),
      rclcpp::Parameter("estimator_dt", std::string("0.002"))});
  const std::string e = error_of(*n);
  EXPECT_TRUE(contains(e, "mocap_override_channel: wrong type")) << e;
  EXPECT_TRUE(contains(e, "enable_sonar: wrong type")) << e;
  EXPECT_TRUE(contains(e, "mocap_pose_topic: wrong type")) << e;
  EXPECT_TRUE(contains(e, "estimator_dt has type string; expected a number")) << e;
}

TEST_F(RosParameters, MissingMatrixIsRejected)
{
  auto n = node({});   // no configuration at all
  const std::string e = error_of(*n);
  for (const char * name : {"xy_x0", "xy_P0", "xy_Q", "xy_R0", "xy_beta", "z_x0", "z_P0",
      "z_P0_flying", "z_Q", "z_R0", "z_R_flying", "z_beta"})
  {
    EXPECT_TRUE(contains(e, std::string(name) + " is not set")) << name;
  }
}

TEST_F(RosParameters, MatrixSizeTypeAndFinitenessAreRejected)
{
  auto n = node({kMaster}, {
      rclcpp::Parameter("xy_P0", std::vector<double>(35, 0.01)),
      rclcpp::Parameter("z_R0", std::string("0.04")),
      rclcpp::Parameter("z_x0", std::vector<double>{-0.25, 0.0,
        std::numeric_limits<double>::infinity()}),
      rclcpp::Parameter("xy_beta", std::vector<double>{})});
  const std::string e = error_of(*n);
  EXPECT_TRUE(contains(e, "xy_P0 has 35 values; expected 36 values (6x6, row-major) or 6 values"));
  EXPECT_TRUE(contains(e, "z_R0 has type string"));
  EXPECT_TRUE(contains(e, "z_x0[2] is not finite"));
  EXPECT_TRUE(contains(e, "xy_beta is empty"));
}

TEST_F(RosParameters, ValidationRejectsBadCovarianceWeightsAndGates)
{
  auto n = node({kMaster}, {
      rclcpp::Parameter("z_P0", std::vector<double>{0.025, -1.0, 0.01}),
      rclcpp::Parameter("xy_R0", std::vector<double>{0.02, 0.001, 0.002, 0.02}),
      rclcpp::Parameter("z_beta", std::vector<double>{1.0, 1.0, 1.5}),
      rclcpp::Parameter("mahalanobis_d_mocap_z", -20.0),
      rclcpp::Parameter("estimator_dt", 0.0)});
  const std::string e = error_of(*n);
  EXPECT_TRUE(contains(e, "z_P0(1,1) is negative")) << e;
  EXPECT_TRUE(contains(e, "xy_R0 is not symmetric")) << e;
  EXPECT_TRUE(contains(e, "z_beta[2]")) << e;
  EXPECT_TRUE(contains(e, "mahalanobis_d_mocap_z")) << e;
  EXPECT_TRUE(contains(e, "estimator_dt")) << e;
}

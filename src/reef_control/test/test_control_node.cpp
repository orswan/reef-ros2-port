// ControlNode: parameter contract (required limits, Gains.cfg ranges, types),
// runtime gain changes (rejected out of range; applied like gainsCallback,
// integrators kept), and the command message (u[0..3], stamp = estimate).
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "reef_control/control_node.hpp"

using reef_control::ControlNode;

namespace
{

std::vector<rclcpp::Parameter> quad()
{
  return {{"max_roll", 0.25}, {"max_pitch", 0.25}, {"max_yaw_rate", 2.0},
          {"uP", 0.3}, {"uI", 0.025}, {"wP", 0.6}, {"wI", 0.2}, {"dP", 0.75}, {"dI", 0.05},
          {"uvtau", 0.15}, {"nedtau", 0.15}, {"max_u", 1.0}, {"max_v", 1.0}, {"max_w", 1.0}, {"max_d", 1.0}};
}

std::shared_ptr<ControlNode> make(std::vector<rclcpp::Parameter> ps)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides(ps);
  o.use_global_arguments(false);
  return std::make_shared<ControlNode>(o);
}

std::string error_of(std::vector<rclcpp::Parameter> ps)
{
  try {
    make(ps);
  } catch (const reef_control::ParameterError & e) {
    return e.what();
  }
  return "";
}

class Node : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

}  // namespace

TEST_F(Node, MissingLimitIsNamed)
{
  auto ps = quad();
  ps.erase(ps.begin());   // max_roll
  EXPECT_NE(error_of(ps).find("max_roll is required"), std::string::npos);
}

TEST_F(Node, OutOfRangeGainIsRejectedNotClamped)
{
  auto ps = quad();
  ps.emplace_back("uP", 3.0);   // Gains.cfg: [0, 2]
  const std::string e = error_of(ps);
  EXPECT_NE(e.find("uP = 3"), std::string::npos) << e;
}

TEST_F(Node, IntegerAcceptedForNumbersButNotForSwitches)
{
  auto ps = quad();
  ps.emplace_back("max_w", 1);   // roscpp getParam(double) accepted integers
  EXPECT_NO_THROW(make(ps));
  ps.emplace_back("xIntegrator", 1);
  EXPECT_NE(error_of(ps).find("xIntegrator"), std::string::npos);
}

TEST_F(Node, RuntimeGainChangeKeepsIntegrators)
{
  auto node = make(quad());
  rosflight_msgs::msg::Status armed;
  armed.armed = true;
  node->onStatus(armed);
  reef_msgs::msg::DesiredState d;
  d.velocity_valid = true;
  d.velocity.x = 0.3;
  node->onDesiredState(d);
  reef_msgs::msg::XYZEstimate e;
  for (int k = 0; k < 20; ++k) {
    e.header.stamp.sec = 100;
    e.header.stamp.nanosec = 4000000u * k;
    node->onEstimate(e);
  }
  const double integ = node->controller().uPID().integrator();
  ASSERT_NE(integ, 0.0);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("uP", 2.5)).successful);   // out of range
  EXPECT_EQ(node->controller().uPID().kp(), 0.3);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("max_roll", 0.3)).successful);   // read only
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("uP", 1.2)).successful);
  EXPECT_EQ(node->controller().uPID().kp(), 1.2);
  EXPECT_EQ(node->controller().uPID().integrator(), integ);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("uIntegrator", false)).successful);
  EXPECT_EQ(node->controller().uPID().ki(), 0.0);
  EXPECT_EQ(node->controller().uPID().integrator(), integ);
}

TEST_F(Node, CommandMessageLayoutAndStamp)
{
  auto node = make(quad());
  reef_msgs::msg::DesiredState d;
  d.altitude_only = true;
  d.pose.z = -1.0;
  node->onDesiredState(d);
  reef_msgs::msg::XYZEstimate e;
  e.header.stamp.sec = 1234;
  e.header.stamp.nanosec = 567;
  node->onEstimate(e);
  const auto & c = node->lastCommand();
  EXPECT_EQ(c.header.stamp.sec, 1234);
  EXPECT_EQ(c.header.stamp.nanosec, 567u);
  EXPECT_EQ(c.mode, rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE);
  EXPECT_EQ(c.ignore, rosflight_msgs::msg::Command::IGNORE_VALUE0 | rosflight_msgs::msg::Command::IGNORE_VALUE1 |
    rosflight_msgs::msg::Command::IGNORE_VALUE2);
  EXPECT_EQ(c.u[3], node->controller().lastCommand().F);
  EXPECT_EQ(c.u[4], 0.0f);
}

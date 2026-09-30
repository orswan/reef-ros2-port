// SensorManager node in-process: configuration, subscriptions, QoS, outputs,
// parameter errors, reset (service and backward time jump), is_flying_reef.
#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "reef_estimator/sensor_manager.h"
#include "reef_msgs/parameters.hpp"

using reef_estimator::SensorManager;
using namespace std::chrono_literals;

namespace
{

const std::string kMaster = std::string(CONFIG_DIR) + "/estimator_master.yaml";
const std::string kSim = std::string(CONFIG_DIR) + "/simulation.yaml";

class Node : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<SensorManager> make(
    const std::vector<std::string> & files, const std::vector<rclcpp::Parameter> & overrides = {},
    const std::string & ns = "")
  {
    std::vector<std::string> args{"--ros-args"};
    for (const auto & f : files) {args.insert(args.end(), {"--params-file", f});}
    if (!ns.empty()) {args.insert(args.end(), {"-r", "__ns:=" + ns});}
    return std::make_shared<SensorManager>(
      rclcpp::NodeOptions().arguments(args).parameter_overrides(overrides));
  }

  static sensor_msgs::msg::Imu imu(int k, double az = -9.81)
  {
    sensor_msgs::msg::Imu m;
    const long long t = 1000000000LL + k * 2000000LL;
    m.header.stamp.sec = static_cast<int32_t>(t / 1000000000LL);
    m.header.stamp.nanosec = static_cast<uint32_t>(t % 1000000000LL);
    m.linear_acceleration.z = az;
    m.orientation.w = 1.0;
    return m;
  }

  static sensor_msgs::msg::Range range(int k, float r)
  {
    sensor_msgs::msg::Range m;
    m.header.stamp = imu(k).header.stamp;
    m.range = r;
    m.max_range = 7.65f;
    return m;
  }

  // A takeoff-like stream: range 0.30 m every 10th sample, vibration from k = 20.
  static void stream(SensorManager & n, int k0, int count, std::vector<reef_msgs::msg::XYZEstimate> * out = nullptr)
  {
    for (int k = k0; k < k0 + count; k++) {
      if (k % 10 == 0) {n.altimeterCallback(range(k, 0.30f + 0.001f * (k % 7)));}
      const long before = n.publishedCount();
      n.imuCallback(imu(k, -9.81 + ((k >= 20 && k % 2) ? 1.5 : (k >= 20 ? -1.5 : 0.0))));
      if (out && n.publishedCount() != before) {out->push_back(*n.lastEstimate());}
    }
  }
};

bool contains(const std::string & s, const std::string & part) {return s.find(part) != std::string::npos;}

}  // namespace

TEST_F(Node, SimulationConfigurationSubscribesToTheSimulatedInputs)
{
  auto n = make({kMaster, kSim}, {}, "/sim_cfg");
  EXPECT_EQ(n->count_subscribers("/sim_cfg/imu/data"), 1u);
  EXPECT_EQ(n->count_subscribers("/sim_cfg/sonar"), 1u);
  EXPECT_EQ(n->count_subscribers("/sim_cfg/rc_raw"), 0u);                     // switch disabled
  EXPECT_EQ(n->count_subscribers("/sim_cfg/mocap_velocity/body_level_frame"), 1u);  // idealized velocity
  EXPECT_EQ(n->count_subscribers("/sim_cfg/rgbd_velocity_body_frame"), 0u);
  EXPECT_EQ(n->count_subscribers("/sim_cfg/mocap_ned"), 0u);   // disabled in simulation.yaml
}

TEST_F(Node, MasterConfigurationSubscribesToRcRaw)
{
  auto n = make({kMaster}, {}, "/hw_cfg");
  EXPECT_EQ(n->count_subscribers("/hw_cfg/rc_raw"), 1u);
}

TEST_F(Node, QoSMatchesTheContract)
{
  auto n = make({kMaster, kSim}, {}, "/qos");
  for (const char * topic : {"/qos/xyz_estimate", "/qos/xyz_debug_estimate", "/qos/is_flying_reef"}) {
    const auto info = n->get_publishers_info_by_topic(topic);
    ASSERT_EQ(info.size(), 1u) << topic;
    EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable) << topic;
    EXPECT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal) << topic;
    // (depth is not part of the discovery information)
  }
  for (const char * topic : {"/qos/imu/data", "/qos/sonar"}) {
    const auto info = n->get_subscriptions_info_by_topic(topic);
    ASSERT_EQ(info.size(), 1u) << topic;
    EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort) << topic;
  }
}

TEST_F(Node, OutputsCarryTheImuStampAndTheCoreState)
{
  auto n = make({kMaster, kSim});
  for (int k = 0; k < 20; k++) {n->imuCallback(imu(k));}
  EXPECT_EQ(n->publishedCount(), 0);   // accelerometer initialization
  n->imuCallback(imu(20));
  ASSERT_TRUE(n->lastEstimate());
  const auto & m = *n->lastEstimate();
  EXPECT_EQ(m.header.stamp, imu(20).header.stamp);
  EXPECT_TRUE(m.header.frame_id.empty());
  EXPECT_EQ(m.node_id, 0u);
  EXPECT_EQ(m.xy_plus.x_dot, n->core().xyFilter().xHat(0, 0));
  EXPECT_EQ(m.xy_plus.y_dot, n->core().xyFilter().xHat(1, 0));
  EXPECT_EQ(m.z_plus.z, n->core().zFilter().xHat(0, 0));
  ASSERT_TRUE(n->lastDebugEstimate());   // debug_mode is true in the master file
  const auto & d = *n->lastDebugEstimate();
  EXPECT_EQ(d.xy_plus.pitch_bias, n->core().xyFilter().xHat(2, 0));
  EXPECT_EQ(d.xy_minus.sigma_plus[5], d.xy_minus.ya_bias + 3 * std::sqrt(n->core().xyMinusState().P(5, 5)));
  EXPECT_EQ(d.z_plus.p[4], n->core().zFilter().P(1, 1));
  EXPECT_EQ(d.z_plus.truth[0], 0.0);
}

TEST_F(Node, DiagnosticsReportTimingAndObservationAccounting)
{
  auto n = make({kMaster, kSim}, {}, "/diag");
  auto helper = rclcpp::Node::make_shared("diag_listener", "/diag");
  std::vector<diagnostic_msgs::msg::DiagnosticArray> got;
  auto sub = helper->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diag/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray & m) {got.push_back(m);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(n);
  exec.add_node(helper);
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (sub->get_publisher_count() < 1 && std::chrono::steady_clock::now() < end) {exec.spin_some(20ms);}
  for (int k = 0; k < 260; k++) {n->imuCallback(imu(k));}
  const auto end2 = std::chrono::steady_clock::now() + 5s;
  while (got.empty() && std::chrono::steady_clock::now() < end2) {exec.spin_some(20ms);}
  ASSERT_FALSE(got.empty());
  std::map<std::string, std::string> v;
  for (const auto & kvp : got.back().status.at(0).values) {v[kvp.key] = kvp.value;}
  EXPECT_EQ(v["callbacks_total"], "249");   // published inside the 250th IMU callback
  EXPECT_EQ(v["correction_c1"], "true");   // approved at R1, default
  EXPECT_TRUE(v.count("window_callback_us_p99"));
  EXPECT_TRUE(v.count("xy_fusions"));
}

TEST_F(Node, StampAnomaliesAreCountedButProcessed)
{
  auto n = make({kMaster, kSim});
  for (int k = 0; k < 25; k++) {n->imuCallback(imu(k));}
  n->imuCallback(imu(24));   // duplicate stamp
  EXPECT_EQ(n->stampAnomalies(), 1);
  EXPECT_EQ(n->core().zFilter().dt, 0.0);   // processed as in the original
}

TEST_F(Node, InvalidParametersThrowNamingTheParameter)
{
  try {
    make({kMaster}, {rclcpp::Parameter("z_Q", std::vector<double>{0.03, 0.0001, 0.1})});
    FAIL() << "no exception";
  } catch (const reef_msgs::ParameterError & e) {
    EXPECT_TRUE(contains(e.what(), "z_Q has 3 values")) << e.what();
  }
}

TEST_F(Node, ResetMakesTheNodeBehaveLikeAFreshOne)
{
  auto used = make({kMaster, kSim});
  stream(*used, 0, 150);
  EXPECT_TRUE(used->core().isFlying());
  used->reset("test");
  EXPECT_EQ(used->core().estimateCount(), 0);
  std::vector<reef_msgs::msg::XYZEstimate> after, fresh;
  stream(*used, 1000, 150, &after);
  auto clean = make({kMaster, kSim});
  stream(*clean, 1000, 150, &fresh);
  ASSERT_EQ(after.size(), fresh.size());
  ASSERT_GT(after.size(), 100u);
  for (std::size_t i = 0; i < after.size(); i++) {
    // exact equality of every field
    EXPECT_EQ(after[i].header, fresh[i].header) << i;
    EXPECT_EQ(after[i].z_plus.z, fresh[i].z_plus.z) << i;
    EXPECT_EQ(after[i].z_plus.z_dot, fresh[i].z_plus.z_dot) << i;
    EXPECT_EQ(after[i].xy_plus.x_dot, fresh[i].xy_plus.x_dot) << i;
    EXPECT_EQ(after[i].xy_plus.y_dot, fresh[i].xy_plus.y_dot) << i;
  }
}

TEST_F(Node, ResetServiceAndIsFlyingPublication)
{
  auto n = make({kMaster, kSim}, {}, "/svc");
  auto helper = rclcpp::Node::make_shared("reset_client", "/svc");
  std::vector<bool> flying;
  auto sub = helper->create_subscription<std_msgs::msg::Bool>("/svc/is_flying_reef",
      rclcpp::QoS(1).reliable().transient_local(), [&](const std_msgs::msg::Bool & m) {flying.push_back(m.data);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(n);
  exec.add_node(helper);
  auto spin_until = [&](const std::function<bool()> & done) {
      const auto end = std::chrono::steady_clock::now() + 5s;
      while (!done() && std::chrono::steady_clock::now() < end) {exec.spin_some(20ms);}
      return done();
    };
  spin_until([&] {return sub->get_publisher_count() == 1;});
  stream(*n, 0, 150);
  ASSERT_TRUE(spin_until([&] {return !flying.empty();}));
  EXPECT_TRUE(flying.back());   // takeoff published
  auto client = helper->create_client<std_srvs::srv::Trigger>("/svc/reef_estimator/reset");
  ASSERT_TRUE(client->wait_for_service(5s));
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(spin_until([&] {return future.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future.get()->success);
  EXPECT_EQ(n->core().estimateCount(), 0);
  ASSERT_TRUE(spin_until([&] {return flying.size() >= 2;}));
  EXPECT_FALSE(flying.back());   // reset returns to "on the ground"
}

TEST_F(Node, BackwardSimTimeJumpResets)
{
  auto n = make({kMaster, kSim}, {rclcpp::Parameter("use_sim_time", true)}, "/jump");
  auto helper = rclcpp::Node::make_shared("clock_source", "/jump");
  auto clock_pub = helper->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(n);
  exec.add_node(helper);
  auto publish_clock = [&](int32_t sec) {
      rosgraph_msgs::msg::Clock c;
      c.clock.sec = sec;
      const auto end = std::chrono::steady_clock::now() + 5s;
      while (std::chrono::steady_clock::now() < end && n->get_clock()->now().seconds() != sec) {
        clock_pub->publish(c);
        exec.spin_some(20ms);
      }
      return n->get_clock()->now().seconds() == sec;
    };
  ASSERT_TRUE(publish_clock(100));
  for (int k = 0; k < 30; k++) {n->imuCallback(imu(k));}
  EXPECT_EQ(n->core().estimateCount(), 10);
  ASSERT_TRUE(publish_clock(50));   // jump back 50 s
  // The time source signals the jump from its own thread; the node only
  // flags it, and the next message callback (executor thread) applies it.
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (!n->resetPending() && std::chrono::steady_clock::now() < end) {exec.spin_some(10ms);}
  ASSERT_TRUE(n->resetPending());
  EXPECT_EQ(n->core().estimateCount(), 10);
  n->imuCallback(imu(0));
  EXPECT_FALSE(n->resetPending());
  EXPECT_EQ(n->core().estimateCount(), 0);    // reset before this message was processed
  EXPECT_FALSE(n->core().accelerometerInitialized());
}

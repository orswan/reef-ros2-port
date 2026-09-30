// Event-replay driver for the fidelity comparison (P04).
//   reef_estimator_event_replay PARAMS EVENTS OUT.csv [--mode core|node]
// core (default): events go straight to the ROS-free VerticalEstimator.
// node: each event becomes a ROS 2 message (builtin_interfaces stamp,
//   float32 range, ...) delivered to an in-process SensorManager node through
//   its callbacks; the node gets the parameters as ROS parameters. The CSV
//   then also has the fields of the published messages (msg_*), so the check
//   can require them to equal the core state (wrapper equivalence).
// Reads the P02 harness formats and feeds each event to the ported vertical
// estimator in file order, with the ROS 1 node's subscription rules, then
// writes the vertical state after every event using the harness column names.
// Horizontal events (mocap_twist, rgbd) are counted as delivered when the
// original would have subscribed, but the vertical filter never uses them.
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "event_file.hpp"
#include "reef_estimator/sensor_manager.h"

namespace
{

void put(std::ostream & o, double x) {o << ',' << x;}

std::vector<double> flat(const Eigen::MatrixXd & m)
{
  std::vector<double> v;
  for (Eigen::Index i = 0; i < m.rows(); i++) {
    for (Eigen::Index j = 0; j < m.cols(); j++) {v.push_back(m(i, j));}
  }
  return v;
}

// EstimatorParameters -> ROS parameter overrides (what a YAML file would give).
std::vector<rclcpp::Parameter> as_ros_parameters(const reef_estimator::EstimatorParameters & p)
{
  return {
    {"debug_mode", p.debug_mode}, {"enable_xy", p.enable_xy}, {"enable_z", p.enable_z},
    {"enable_mocap_xy", p.enable_mocap_xy}, {"enable_rgbd", p.enable_rgbd},
    {"enable_mocap_z", p.enable_mocap_z}, {"enable_sonar", p.enable_sonar},
    {"enable_partial_update", p.enable_partial_update},
    {"enable_mocap_switch", p.enable_mocap_switch},
    {"mocap_override_channel", static_cast<int64_t>(p.mocap_override_channel)},
    {"enable_measurements", p.enable_measurements},
    {"mahalanobis_d_sonar", p.mahalanobis_d_sonar},
    {"mahalanobis_d_rgbd_velocity", p.mahalanobis_d_rgbd_velocity},
    {"mahalanobis_d_mocap_z", p.mahalanobis_d_mocap_z},
    {"mahalanobis_d_mocap_velocity", p.mahalanobis_d_mocap_velocity},
    {"estimator_dt", p.estimator_dt},
    {"xy_x0", flat(p.xy_x0)}, {"xy_P0", flat(p.xy_P0)}, {"xy_Q", flat(p.xy_Q)},
    {"xy_R0", flat(p.xy_R0)}, {"xy_beta", flat(p.xy_beta)}, {"z_x0", flat(p.z_x0)},
    {"z_P0", flat(p.z_P0)}, {"z_P0_flying", flat(p.z_P0_flying)}, {"z_Q", flat(p.z_Q)},
    {"z_R0", flat(p.z_R0)}, {"z_R_flying", flat(p.z_R_flying)}, {"z_beta", flat(p.z_beta)}};
}

builtin_interfaces::msg::Time ros_time(long long t_ns)
{
  const reef_estimator::Stamp s = reef_estimator::tools::stamp_of(t_ns);
  builtin_interfaces::msg::Time t;
  t.sec = s.sec;
  t.nanosec = s.nanosec;
  return t;
}

}  // namespace

int main(int argc, char ** argv)
{
  const bool node_mode = argc == 6 && std::string(argv[4]) == "--mode" && std::string(argv[5]) == "node";
  const bool core_mode = argc == 4 ||
    (argc == 6 && std::string(argv[4]) == "--mode" && std::string(argv[5]) == "core");
  if (!node_mode && !core_mode) {
    std::fprintf(stderr, "usage: %s PARAMS EVENTS OUT.csv [--mode core|node]\n", argv[0]);
    return 2;
  }
  using namespace reef_estimator;
  if (node_mode) {rclcpp::init(1, argv);}
  int status = 0;
  try {
    const EstimatorParameters params = tools::load_params(argv[1]);
    std::unique_ptr<VerticalEstimator> core;
    std::shared_ptr<SensorManager> node;
    if (node_mode) {
      node = std::make_shared<SensorManager>(
        rclcpp::NodeOptions().parameter_overrides(as_ros_parameters(params)));
    } else {
      core = std::make_unique<VerticalEstimator>(params);
    }
    auto est = [&]() -> const VerticalEstimator & {return node ? node->core() : *core;};
    const auto events = tools::load_events(argv[2]);

    std::ofstream out(argv[3]);
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "idx,type,t_ns,z_flag_before,z,zdot,zbias";
    for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {out << ",zP" << i << j;}}
    out << ",z_meas,zR,u,z_dt,g_init,acc_init,takeoff,z_flag,n_prop,maha2,z_gate,"
      "use_mocap_z,n_published,delivered";
    if (node_mode) {
      out << ",msg_published,msg_stamp_ns,msg_z,msg_zdot,msg_x_dot_isnan,dbg_bias,dbg_u";
      for (int k = 0; k < 9; ++k) {out << ",dbg_p" << k;}
      out << ",dbg_minus_z";
    }
    out << "\n";

    long idx = 0;
    for (const auto & ev : events) {
      const VerticalEstimator & e0 = est();
      const bool z_before = e0.pendingZMeasurement();
      const long gates_before = e0.zGateCount();
      const long published_before = node ? node->publishedCount() : 0;
      bool delivered = true;
      if (ev.type == "imu" || ev.type == "imu_nan") {
        if (node) {
          const ImuSample s = tools::imu_sample(ev);
          sensor_msgs::msg::Imu m;
          m.header.stamp = ros_time(ev.t_ns);
          m.linear_acceleration.x = s.ax; m.linear_acceleration.y = s.ay; m.linear_acceleration.z = s.az;
          m.orientation.x = s.qx; m.orientation.y = s.qy; m.orientation.z = s.qz; m.orientation.w = s.qw;
          node->imuCallback(m);
        } else {
          core->sensorUpdate(tools::imu_sample(ev));
        }
      } else if (ev.type == "range") {
        delivered = e0.subscribesRange();
        if (delivered && node) {
          sensor_msgs::msg::Range m;
          m.header.stamp = ros_time(ev.t_ns);
          m.range = static_cast<float>(ev.f.at(0));
          m.max_range = static_cast<float>(ev.f.at(1));
          node->altimeterCallback(m);
        } else if (delivered) {
          core->sensorUpdate(tools::range_sample(ev));
        }
      } else if (ev.type == "mocap_pose") {
        delivered = e0.subscribesMocapPose();
        if (delivered && node) {
          geometry_msgs::msg::PoseStamped m;
          m.header.stamp = ros_time(ev.t_ns);
          m.pose.position.z = ev.f.at(0);
          node->mocapPoseCallback(m);
        } else if (delivered) {
          core->mocapUpdate(tools::mocap_pose_sample(ev));
        }
      } else if (ev.type == "rc") {
        delivered = e0.subscribesRc();
        if (delivered && node) {
          rosflight_msgs::msg::RCRaw m;
          m.header.stamp = ros_time(ev.t_ns);
          for (std::size_t i = 0; i < m.values.size(); i++) {m.values[i] = static_cast<uint16_t>(ev.f.at(i));}
          node->rcRawCallback(m);
        } else if (delivered) {
          core->rcRawUpdate(tools::rc_sample(ev));
        }
      } else if (ev.type == "mocap_twist") {
        delivered = params.enable_mocap_xy;
      } else if (ev.type == "rgbd") {
        delivered = params.enable_rgbd;
      } else {
        throw std::runtime_error("unknown event type '" + ev.type + "'");
      }
      const VerticalEstimator & e = est();
      const ZEstimator & z = e.zFilter();
      out << idx++ << ',' << ev.type << ',' << ev.t_ns << ',' << z_before;
      for (int i = 0; i < 3; ++i) {put(out, z.xHat(i, 0));}
      for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {put(out, z.P(i, j));}}
      put(out, z.z(0)); put(out, z.R(0, 0)); put(out, z.u(0)); put(out, z.dt);
      put(out, e.accelerometerInitialized() ? e.initialGravity() :
        std::numeric_limits<double>::quiet_NaN());
      out << ',' << e.accelerometerInitialized() << ',' << e.isFlying() << ','
          << e.pendingZMeasurement() << ',' << e.propagationCount();
      put(out, e.lastMahalanobisSquared());
      out << ',' << (e.zGateCount() != gates_before) << ',' << e.usingMocapZ() << ','
          << e.estimateCount() << ',' << delivered;
      if (node) {
        const bool published = node->publishedCount() != published_before;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        out << ',' << published;
        if (published) {
          const auto & m = *node->lastEstimate();
          out << ',' << (static_cast<long long>(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec);
          put(out, m.z_plus.z); put(out, m.z_plus.z_dot);
          out << ',' << std::isnan(m.xy_plus.x_dot);
          if (node->lastDebugEstimate()) {
            const auto & d = *node->lastDebugEstimate();
            put(out, d.z_plus.bias); put(out, d.z_plus.u);
            for (double v : d.z_plus.p) {put(out, v);}
            put(out, d.z_minus.z);
          } else {
            for (int k = 0; k < 12; ++k) {put(out, nan);}
          }
        } else {
          out << ",-1";
          for (int k = 0; k < 15; ++k) {put(out, nan);}
        }
      }
      out << '\n';
    }
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "reef_estimator_event_replay: %s\n", ex.what());
    status = 2;
  }
  if (node_mode) {rclcpp::shutdown();}
  return status;
}

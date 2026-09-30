// Event-replay driver for the fidelity comparison (P04).
//   reef_estimator_event_replay PARAMS EVENTS OUT.csv [--mode core|node]
// core (default): events go straight to the ROS-free XYZEstimator.
// node: each event becomes a ROS 2 message (builtin_interfaces stamp,
//   float32 range, ...) delivered to an in-process SensorManager node through
//   its callbacks; the node gets the parameters as ROS parameters. The CSV
//   then also has the fields of the published messages (msg_*).
// PORT_PUBLISHED=<path>: one row per event that published an estimate, with
//   the message fields in the column order of the reference harness
//   (REF_PUBLISHED, baseline/harness/reef_ref_main.cpp). In core mode the
//   messages are built with ros_conversions.hpp from the core; in node mode
//   they are the messages the node published.
// Reads the P02 harness formats and feeds each event to the ported vertical
// estimator in file order, with the ROS 1 node's subscription rules, then
// writes the vertical state after every event using the harness column names.
// rgbd events are applied only if enable_measurements is true (SensorManager
// read that parameter at every RGB-D message).
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "event_file.hpp"
#include "reef_estimator/ros_conversions.hpp"
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
    {"correction_c1_clear_xy_flag", p.correction_c1},
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

template<class A> void put_array(std::ostream & o, const A & a) {for (double v : a) {put(o, v);}}

void put_z(std::ostream & o, const reef_msgs::msg::ZDebugEstimate & z)
{
  put(o, z.z); put(o, z.z_dot); put(o, z.bias); put(o, z.u);
  put_array(o, z.p); put_array(o, z.sigma_plus); put_array(o, z.sigma_minus);
}

void put_xy(std::ostream & o, const reef_msgs::msg::XYDebugEstimate & x)
{
  put(o, x.x_dot); put(o, x.y_dot); put(o, x.pitch_bias); put(o, x.roll_bias); put(o, x.xa_bias); put(o, x.ya_bias);
  put_array(o, x.sigma_plus); put_array(o, x.sigma_minus);
}

void published_header(std::ostream & o)
{
  o << "idx,t_ns,pub_z,pub_zdot,pub_x_dot,pub_y_dot,debug";
  for (const char * g : {"dz", "mz"}) {
    for (const char * n : {"z", "zdot", "bias", "u"}) {o << ',' << g << '_' << n;}
    for (int k = 0; k < 9; ++k) {o << ',' << g << "_p" << k;}
    for (int k = 0; k < 3; ++k) {o << ',' << g << "_sp" << k;}
    for (int k = 0; k < 3; ++k) {o << ',' << g << "_sm" << k;}
    const char * xy = g[0] == 'd' ? "dxy" : "mxy";
    for (const char * n : {"x_dot", "y_dot", "pitch_bias", "roll_bias", "xa_bias", "ya_bias"}) {
      o << ',' << xy << '_' << n;
    }
    for (int k = 0; k < 6; ++k) {o << ',' << xy << "_sp" << k;}
    for (int k = 0; k < 6; ++k) {o << ',' << xy << "_sm" << k;}
  }
  o << '\n';
}

void published_row(std::ostream & o, long idx, long long t_ns, const reef_msgs::msg::XYZEstimate & m,
  const reef_msgs::msg::XYZDebugEstimate * d)
{
  o << idx << ',' << t_ns;
  put(o, m.z_plus.z); put(o, m.z_plus.z_dot); put(o, m.xy_plus.x_dot); put(o, m.xy_plus.y_dot);
  o << ',' << (d != nullptr);
  // the harness writes a zero-initialized debug message when none was published
  const reef_msgs::msg::XYZDebugEstimate zero;
  const reef_msgs::msg::XYZDebugEstimate & dd = d ? *d : zero;
  put_z(o, dd.z_plus); put_xy(o, dd.xy_plus); put_z(o, dd.z_minus); put_xy(o, dd.xy_minus);
  o << '\n';
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
    std::unique_ptr<XYZEstimator> core;
    std::shared_ptr<SensorManager> node;
    if (node_mode) {
      node = std::make_shared<SensorManager>(
        rclcpp::NodeOptions().parameter_overrides(as_ros_parameters(params)));
    } else {
      core = std::make_unique<XYZEstimator>(params);
    }
    auto est = [&]() -> const XYZEstimator & {return node ? node->core() : *core;};
    const auto events = tools::load_events(argv[2]);

    std::ofstream out(argv[3]);
    std::ofstream published;
    if (const char * path = std::getenv("PORT_PUBLISHED")) {
      published.open(path);
      published << std::setprecision(std::numeric_limits<double>::max_digits10);
      published_header(published);
    }
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "idx,type,t_ns,z_flag_before,xy_flag_before,z,zdot,zbias";
    for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {out << ",zP" << i << j;}}
    out << ",vx,vy,pitch_bias,roll_bias,ax_bias,ay_bias";
    for (int i = 0; i < 6; ++i) {for (int j = 0; j < 6; ++j) {out << ",xyP" << i << j;}}
    out << ",z_meas,zR,xy_meas0,xy_meas1,xyR00,xyR11,u,z_dt,xy_dt,g_init,acc_init,takeoff,z_flag,xy_flag,"
      "n_prop,maha2,z_gate,xy_gate,use_mocap_xy,use_mocap_z,n_published,delivered,xy_accepted,xy_fusions";
    if (node_mode) {
      out << ",msg_published,msg_stamp_ns,msg_z,msg_zdot,msg_x_dot,msg_y_dot,dbg_bias,dbg_u";
      for (int k = 0; k < 9; ++k) {out << ",dbg_p" << k;}
      out << ",dbg_minus_z,dbg_pitch_bias,dbg_roll_bias,dbg_xa_bias,dbg_ya_bias,dbg_xy_sigma_plus0,dbg_minus_x_dot";
    }
    out << "\n";

    long idx = 0;
    for (const auto & ev : events) {
      const XYZEstimator & e0 = est();
      const bool z_before = e0.pendingZMeasurement();
      const bool xy_before = e0.pendingXYMeasurement();
      const long gates_before = e0.zGateCount();
      const long xy_gates_before = e0.xyGateCount();
      const long published_before = node ? node->publishedCount() : 0;
      const long estimates_before = e0.estimateCount();
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
      } else if (ev.type == "mocap_twist" || ev.type == "rgbd") {
        const bool mocap = ev.type == "mocap_twist";
        delivered = mocap ? e0.subscribesMocapTwist() : e0.subscribesRgbd();
        if (delivered && node) {
          geometry_msgs::msg::TwistWithCovarianceStamped tw;
          tw.header.stamp = ros_time(ev.t_ns);
          tw.twist.twist.linear.x = ev.f.at(0);
          tw.twist.twist.linear.y = ev.f.at(1);
          tw.twist.covariance[0] = ev.f.at(2);
          tw.twist.covariance[7] = ev.f.at(3);
          if (mocap) {
            node->mocapTwistCallback(tw);
          } else {
            reef_msgs::msg::DeltaToVel d;
            d.header = tw.header;
            d.vel = tw;
            node->rgbdTwistCallback(d);
          }
        } else if (delivered) {
          if (mocap) {
            core->mocapUpdate(tools::twist_sample(ev));
          } else if (params.enable_measurements) {
            core->rgbdUpdate(tools::twist_sample(ev));
          }
        }
      } else {
        throw std::runtime_error("unknown event type '" + ev.type + "'");
      }
      const XYZEstimator & e = est();
      if (published.is_open() && e.estimateCount() != estimates_before) {
        if (node) {
          published_row(published, idx, ev.t_ns, *node->lastEstimate(),
            params.debug_mode ? &*node->lastDebugEstimate() : nullptr);
        } else {
          const auto m = toEstimateMsg(e);
          const auto d = toDebugMsg(e);
          published_row(published, idx, ev.t_ns, m, params.debug_mode ? &d : nullptr);
        }
      }
      const ZEstimator & z = e.zFilter();
      const XYEstimator & xy = e.xyFilter();
      out << idx++ << ',' << ev.type << ',' << ev.t_ns << ',' << z_before << ',' << xy_before;
      for (int i = 0; i < 3; ++i) {put(out, z.xHat(i, 0));}
      for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {put(out, z.P(i, j));}}
      for (int i = 0; i < 6; ++i) {put(out, xy.xHat(i, 0));}
      for (int i = 0; i < 6; ++i) {for (int j = 0; j < 6; ++j) {put(out, xy.P(i, j));}}
      put(out, z.z(0)); put(out, z.R(0, 0));
      put(out, xy.z(0)); put(out, xy.z(1)); put(out, xy.R(0, 0)); put(out, xy.R(1, 1));
      put(out, z.u(0)); put(out, z.dt); put(out, xy.dt);
      put(out, e.accelerometerInitialized() ? e.initialGravity() :
        std::numeric_limits<double>::quiet_NaN());
      out << ',' << e.accelerometerInitialized() << ',' << e.isFlying() << ','
          << e.pendingZMeasurement() << ',' << e.pendingXYMeasurement() << ',' << e.propagationCount();
      put(out, e.lastMahalanobisSquared());
      out << ',' << (e.zGateCount() != gates_before) << ',' << (e.xyGateCount() != xy_gates_before) << ','
          << e.usingMocapXY() << ',' << e.usingMocapZ() << ','
          << e.estimateCount() << ',' << delivered << ',' << e.xyObservationsAccepted() << ',' << e.xyFusions();
      if (node) {
        const bool published = node->publishedCount() != published_before;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        out << ',' << published;
        if (published) {
          const auto & m = *node->lastEstimate();
          out << ',' << (static_cast<long long>(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec);
          put(out, m.z_plus.z); put(out, m.z_plus.z_dot);
          put(out, m.xy_plus.x_dot); put(out, m.xy_plus.y_dot);
          if (node->lastDebugEstimate()) {
            const auto & d = *node->lastDebugEstimate();
            put(out, d.z_plus.bias); put(out, d.z_plus.u);
            for (double v : d.z_plus.p) {put(out, v);}
            put(out, d.z_minus.z);
            put(out, d.xy_plus.pitch_bias); put(out, d.xy_plus.roll_bias);
            put(out, d.xy_plus.xa_bias); put(out, d.xy_plus.ya_bias);
            put(out, d.xy_plus.sigma_plus[0]); put(out, d.xy_minus.x_dot);
          } else {
            for (int k = 0; k < 18; ++k) {put(out, nan);}
          }
        } else {
          out << ",-1";
          for (int k = 0; k < 22; ++k) {put(out, nan);}
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

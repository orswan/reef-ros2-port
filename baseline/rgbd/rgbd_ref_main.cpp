// Reference harness (P08): drives the UNMODIFIED pinned rgbd_to_velocity
// (b7637198) with a deterministic odometry fixture and writes its complete
// state after every message.
//
//   rgbd_ref PARAMS EVENTS OUT.csv [--log]
//
// PARAMS: as the estimator harness: <name> bool|double|string|list <value...>
//         (flat names; the converter reads alpha, x_vel_covariance,
//         y_vel_covariance, body_to_camera_quat, body_to_camera_trans)
// EVENTS: odom sec nsec px py pz qx qy qz qw     nav_msgs/Odometry on cam_to_init
// Output columns: rgbd_columns.txt (shared with the port's replay tool).
//
// Adaptations (baseline/README.md): V1 stand-in headers (the estimator
// harness's roscpp and messages, plus nav_msgs/Odometry); V2 the converter
// object is constructed in zeroed storage, so members the original reads or
// prints before setting them are 0; V3 the published init-frame message is
// recorded through a record_published overload (rgbd_record.h).
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <new>
#include <sstream>

#include <boost/make_shared.hpp>

#include "rgbd_to_velocity.h"

namespace ros {
std::map<std::string, ParamValue>& param_store() { static std::map<std::string, ParamValue> s; return s; }
std::map<std::string, long>& publish_counts() { static std::map<std::string, long> c; return c; }
bool& log_enabled() { static bool on = false; return on; }
}  // namespace ros

namespace {
reef_msgs::DeltaToVel g_init_msg;   // V3: the init-frame message as published
}
namespace reef_msgs {
void record_published(const std::string& topic, const DeltaToVel& m) {
  if (topic == "rgbd_to_velocity/init_frame") g_init_msg = m;
}
}

namespace {

[[noreturn]] void die(const std::string& m) { std::fprintf(stderr, "%s\n", m.c_str()); std::exit(2); }

double num(std::istringstream& ss) {
  std::string tok;
  if (!(ss >> tok)) die("missing numeric field");
  char* end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') die("bad number '" + tok + "'");
  return v;
}

void load_params(const std::string& path) {
  std::ifstream in(path);
  if (!in) die("cannot open " + path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string name, kind;
    ss >> name >> kind;
    ros::ParamValue v;
    if (kind == "bool") { std::string b; ss >> b; v.kind = ros::ParamValue::Bool; v.b = (b == "true"); }
    else if (kind == "double") { v.kind = ros::ParamValue::Double; v.d = num(ss); }
    else if (kind == "string") { v.kind = ros::ParamValue::String; ss >> v.s; }
    else if (kind == "list") { v.kind = ros::ParamValue::List; std::string t; while (ss >> t) v.list.push_back(std::strtod(t.c_str(), nullptr)); }
    else die("bad param line: " + line);
    ros::param_store()[name] = v;
  }
}

void put(std::ostream& o, double x) {
  char b[40];
  if (std::isnan(x)) std::snprintf(b, sizeof b, "nan");
  else std::snprintf(b, sizeof b, "%.17g", x);
  o << ',' << b;
}
template <class V> void put_n(std::ostream& o, const V& v, int n) { for (int i = 0; i < n; ++i) put(o, v(i)); }
void put_m(std::ostream& o, const Eigen::Matrix3d& m) { for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) put(o, m(i, j)); }

void write_row(std::ostream& o, long idx, const rgbd_to_velocity::RgbdToVelocity& c) {
  auto& pc = ros::publish_counts();
  o << idx << ",odom," << pc["rgbd_to_velocity/init_frame"] << ',' << pc["rgbd_to_velocity/body_level_frame"];
  o << ',' << c.counterOfSamples;
  put(o, c.current_time_stamp); put(o, c.previous_time_stamp); put(o, c.DT);
  put(o, c.alpha); put(o, c.x_vel_covariance); put(o, c.y_vel_covariance);
  put(o, c.yaw); put(o, c.pitch); put(o, c.roll); put(o, c.beta_0);
  put_n(o, c.beta, 3);
  put_n(o, c.previous_position_init, 3); put_n(o, c.current_position_init, 3);
  put_n(o, c.estimated_velocity_init, 3); put_n(o, c.filtered_velocity_init, 3);
  put_n(o, c.previous_velocity_init, 3); put_n(o, c.filtered_velocity_body_leveled_frame, 3);
  put_m(o, c.C_from_init_to_camera_level_frame); put_m(o, c.covariance_matrix_in_body_level);
  put_n(o, c.quaternion_body_to_camera, 4); put_n(o, c.translation_body_to_camera, 3);
  const auto& v = c.vel_msg;
  o << ',' << v.vel.header.stamp.sec << ',' << v.vel.header.stamp.nsec;
  put(o, v.vel.twist.twist.linear.x); put(o, v.vel.twist.twist.linear.y); put(o, v.vel.twist.twist.linear.z);
  put(o, v.vel.twist.covariance[0]); put(o, v.vel.twist.covariance[7]); put(o, v.vel.twist.covariance[14]);
  for (int i = 0; i < 6; ++i) put(o, v.S_upper_bound[i]);
  for (int i = 0; i < 6; ++i) put(o, v.S_lower_bound[i]);
  const auto& w = g_init_msg;   // V3
  o << ',' << w.vel.header.stamp.sec << ',' << w.vel.header.stamp.nsec;
  put(o, w.vel.twist.twist.linear.x); put(o, w.vel.twist.twist.linear.y); put(o, w.vel.twist.twist.linear.z);
  put(o, w.vel.twist.covariance[0]); put(o, w.vel.twist.covariance[7]); put(o, w.vel.twist.covariance[14]);
  for (int i = 0; i < 3; ++i) put(o, w.S_upper_bound[i]);
  for (int i = 0; i < 3; ++i) put(o, w.S_lower_bound[i]);
  o << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) die("usage: rgbd_ref PARAMS EVENTS OUT.csv [--log]");
  ros::log_enabled() = (argc > 4 && std::string(argv[4]) == "--log");
  load_params(argv[1]);
  // V2: zeroed storage (static storage is zero-initialized; memset for clarity).
  alignas(rgbd_to_velocity::RgbdToVelocity) static unsigned char storage[sizeof(rgbd_to_velocity::RgbdToVelocity)];
  std::memset(storage, 0, sizeof storage);
  auto* conv = new (storage) rgbd_to_velocity::RgbdToVelocity();

  std::ifstream in(argv[2]);
  if (!in) die(std::string("cannot open ") + argv[2]);
  std::ofstream out(argv[3]);
  if (!out) die(std::string("cannot write ") + argv[3]);
  std::string line;
  long idx = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string kind;
    ss >> kind;
    if (kind != "odom") die("unknown event: " + line);
    auto m = boost::make_shared<nav_msgs::Odometry>();
    const double s = num(ss), n = num(ss);
    if (s < 0 || n < 0 || n >= 1e9) die("bad stamp");
    m->header.stamp.sec = static_cast<uint32_t>(s);
    m->header.stamp.nsec = static_cast<uint32_t>(n);
    m->pose.pose.position.x = num(ss); m->pose.pose.position.y = num(ss); m->pose.pose.position.z = num(ss);
    m->pose.pose.orientation.x = num(ss); m->pose.pose.orientation.y = num(ss);
    m->pose.pose.orientation.z = num(ss); m->pose.pose.orientation.w = num(ss);
    std::string extra;
    if (ss >> extra) die("trailing fields: " + line);
    nav_msgs::OdometryConstPtr cm = m;
    conv->poseCallback(cm);
    write_row(out, idx++, *conv);
  }
  return 0;
}

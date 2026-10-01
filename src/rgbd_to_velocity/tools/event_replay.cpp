// Event replay for the comparison with the reference harness
// (baseline/rgbd/rgbd_ref_main.cpp): same PARAMS and EVENTS formats, same
// columns (baseline/rgbd/rgbd_columns.txt).
//
//   rgbd_to_velocity_event_replay PARAMS EVENTS OUT.csv [--mode core|node]
//
// core: the ROS-free converter. node: the ROS 2 node (parameters as
// overrides; odometry converted by the node); the message columns come from
// the ROS 2 messages it published.
// Exit: 0 done, 2 bad input, 3 invalid parameters.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rgbd_to_velocity/rgbd_node.hpp"

using namespace rgbd_to_velocity;

namespace
{
[[noreturn]] void die(const std::string & m, int code = 2) {std::fprintf(stderr, "%s\n", m.c_str()); std::exit(code);}

double num(const std::string & tok)
{
  char * end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') {die("bad number '" + tok + "'");}
  return v;
}

struct Param { std::string kind; std::vector<double> v; };

std::map<std::string, Param> read_params(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {die("cannot open " + path);}
  std::map<std::string, Param> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    std::string name, kind, tok;
    ss >> name >> kind;
    Param p{kind, {}};
    while (ss >> tok) {p.v.push_back(num(tok));}
    if (kind != "double" && kind != "list") {die("bad param line: " + line);}
    out[name] = p;
  }
  return out;
}

ConverterParameters core_params(const std::map<std::string, Param> & ps)
{
  ConverterParameters c;
  auto get = [&](const char * n, double & v) {auto it = ps.find(n); if (it != ps.end() && !it->second.v.empty()) {v = it->second.v[0];}};
  get("alpha", c.alpha);
  get("x_vel_covariance", c.x_vel_covariance);
  get("y_vel_covariance", c.y_vel_covariance);
  if (ps.count("body_to_camera_quat")) {c.body_to_camera_quat = ps.at("body_to_camera_quat").v;}
  if (ps.count("body_to_camera_trans")) {c.body_to_camera_trans = ps.at("body_to_camera_trans").v;}
  const auto e = parameterErrors(c);
  if (!e.empty()) {die("invalid rgbd_to_velocity parameters: " + e.front(), 3);}
  return c;
}

void put(std::ostream & o, double x)
{
  char b[40];
  if (std::isnan(x)) {std::snprintf(b, sizeof b, "nan");} else {std::snprintf(b, sizeof b, "%.17g", x);}
  o << ',' << b;
}
template<class V> void put_n(std::ostream & o, const V & v, int n) {for (int i = 0; i < n; ++i) {put(o, v(i));}}
void put_m(std::ostream & o, const Eigen::Matrix3d & m) {for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {put(o, m(i, j));}}}

void put_msg(std::ostream & o, const reef_msgs::msg::DeltaToVel & v, int bounds)
{
  o << ',' << v.vel.header.stamp.sec << ',' << v.vel.header.stamp.nanosec;
  put(o, v.vel.twist.twist.linear.x); put(o, v.vel.twist.twist.linear.y); put(o, v.vel.twist.twist.linear.z);
  put(o, v.vel.twist.covariance[0]); put(o, v.vel.twist.covariance[7]); put(o, v.vel.twist.covariance[14]);
  for (int i = 0; i < bounds; ++i) {put(o, v.s_upper_bound[i]);}
  for (int i = 0; i < bounds; ++i) {put(o, v.s_lower_bound[i]);}
}

void write_row(std::ostream & o, long idx, const RgbdToVelocity & c, const reef_msgs::msg::DeltaToVel & body,
  const reef_msgs::msg::DeltaToVel & init)
{
  o << idx << ",odom," << c.initFramePublished << ',' << c.bodyLevelPublished << ',' << c.counterOfSamples;
  put(o, c.current_time_stamp); put(o, c.previous_time_stamp); put(o, c.DT);
  put(o, c.alpha); put(o, c.x_vel_covariance); put(o, c.y_vel_covariance);
  put(o, c.yaw); put(o, c.pitch); put(o, c.roll); put(o, c.beta_0);
  put_n(o, c.beta, 3);
  put_n(o, c.previous_position_init, 3); put_n(o, c.current_position_init, 3);
  put_n(o, c.estimated_velocity_init, 3); put_n(o, c.filtered_velocity_init, 3);
  put_n(o, c.previous_velocity_init, 3); put_n(o, c.filtered_velocity_body_leveled_frame, 3);
  put_m(o, c.C_from_init_to_camera_level_frame); put_m(o, c.covariance_matrix_in_body_level);
  put_n(o, c.quaternion_body_to_camera, 4); put_n(o, c.translation_body_to_camera, 3);
  put_msg(o, body, 6);
  put_msg(o, init, 3);
  o << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string mode = "core";
  std::vector<std::string> pos;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--mode" && i + 1 < argc) {mode = argv[++i];} else {pos.push_back(a);}
  }
  if (pos.size() != 3 || (mode != "core" && mode != "node")) {
    die("usage: rgbd_to_velocity_event_replay PARAMS EVENTS OUT.csv [--mode core|node]");
  }
  const auto ps = read_params(pos[0]);
  std::unique_ptr<RgbdToVelocity> core;
  std::shared_ptr<RgbdNode> node;
  reef_msgs::msg::DeltaToVel body, init;
  if (mode == "core") {
    core = std::make_unique<RgbdToVelocity>(core_params(ps));
    core->velocity_init_frame_publisher_ = [&init](const DeltaToVel & m) {init = toRos(m);};
    core->velocity_level_body_publisher_ = [&body](const DeltaToVel & m) {body = toRos(m);};
  } else {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> ov;
    for (const auto & [n, p] : ps) {
      if (p.kind == "list") {ov.emplace_back(n, p.v);} else {ov.emplace_back(n, p.v.at(0));}
    }
    opts.parameter_overrides(ov);
    opts.use_global_arguments(false);
    try {
      node = std::make_shared<RgbdNode>(opts);
    } catch (const ParameterError & e) {
      die(e.what(), 3);
    }
  }
  std::ifstream in(pos[1]);
  if (!in) {die("cannot open " + pos[1]);}
  std::ofstream out(pos[2]);
  if (!out) {die("cannot write " + pos[2]);}
  std::string line;
  long idx = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    std::string kind, tok;
    ss >> kind;
    std::vector<double> f;
    while (ss >> tok) {f.push_back(num(tok));}
    if (kind != "odom" || f.size() != 9) {die("bad event: " + line);}
    nav_msgs::msg::Odometry m;
    m.header.stamp.sec = static_cast<int32_t>(f[0]);
    m.header.stamp.nanosec = static_cast<uint32_t>(f[1]);
    m.pose.pose.position.x = f[2]; m.pose.pose.position.y = f[3]; m.pose.pose.position.z = f[4];
    m.pose.pose.orientation.x = f[5]; m.pose.pose.orientation.y = f[6];
    m.pose.pose.orientation.z = f[7]; m.pose.pose.orientation.w = f[8];
    if (core) {
      core->poseCallback(fromRos(m));
      write_row(out, idx++, *core, body, init);
    } else {
      node->onOdometry(m);
      write_row(out, idx++, node->converter(), node->lastBodyLevel(), node->lastInitFrame());
    }
  }
  if (node) {rclcpp::shutdown();}
  return 0;
}

// Reference harness: drives the UNMODIFIED pinned REEF estimator sources
// (SensorManager -> XYZEstimator -> XYEstimator/ZEstimator) with a
// deterministic event fixture and writes the complete filter state after
// every event.
//
//   reef_ref PARAMS EVENTS OUT.csv [--log]
//
// PARAMS: one parameter per line:  <name> bool|double|string|list <value...>
// EVENTS: one event per line, time in integer nanoseconds:
//   imu         t_ns ax ay az qx qy qz qw   specific force, body FRD (m/s^2);
//                                           attitude quaternion (x y z w)
//   imu_nan     t_ns                        IMU with NaN acceleration
//   range       t_ns range max_range        sonar (float32 fields, as ROS 1)
//   mocap_pose  t_ns z_ned
//   mocap_twist t_ns vx vy cov_xx cov_yy    body-level velocity
//   rgbd        t_ns vx vy cov_xx cov_yy    DeltaToVel.vel
//   rc          t_ns v0 ... v7              rosflight_msgs/RCRaw values
// Lines starting with '#' are comments.
//
// Access to private members relies on compiling every translation unit with
// -Dprivate=public -Dprotected=public (see baseline/README.md, adaptation A1).
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <boost/make_shared.hpp>

#include "sensor_manager.h"

namespace ros {
std::map<std::string, ParamValue>& param_store() { static std::map<std::string, ParamValue> s; return s; }
std::map<std::string, long>& publish_counts() { static std::map<std::string, long> c; return c; }
bool& log_enabled() { static bool on = false; return on; }
}  // namespace ros

namespace {

ros::Time stamp_of(long long t_ns) {
  ros::Time t;
  t.sec = static_cast<uint32_t>(t_ns / 1000000000LL);
  t.nsec = static_cast<uint32_t>(t_ns % 1000000000LL);
  return t;
}

void load_params(const std::string& path) {
  std::ifstream in(path);
  if (!in) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); std::exit(2); }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string name, kind;
    ss >> name >> kind;
    ros::ParamValue v;
    if (kind == "bool") { std::string b; ss >> b; v.kind = ros::ParamValue::Bool; v.b = (b == "true"); }
    else if (kind == "double") { v.kind = ros::ParamValue::Double; ss >> v.d; }
    else if (kind == "string") { v.kind = ros::ParamValue::String; ss >> v.s; }
    else if (kind == "list") { v.kind = ros::ParamValue::List; double x; while (ss >> x) v.list.push_back(x); }
    else { std::fprintf(stderr, "bad param line: %s\n", line.c_str()); std::exit(2); }
    ros::param_store()[name] = v;
  }
}

// Numeric fields are read as tokens and converted with strtod, which accepts
// "inf", "-inf" and "nan" (istream >> double does not, and silently yields 0).
double num(std::istringstream& ss) {
  std::string tok;
  if (!(ss >> tok)) { std::fprintf(stderr, "missing numeric field\n"); std::exit(2); }
  char* end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') { std::fprintf(stderr, "bad number '%s'\n", tok.c_str()); std::exit(2); }
  return v;
}

void put(std::ostream& o, double x) { o << ',' << x; }

void put_matrix(std::ostream& o, const Eigen::MatrixXd& m) {
  for (int i = 0; i < m.rows(); ++i)
    for (int j = 0; j < m.cols(); ++j) put(o, m(i, j));
}

void header(std::ostream& o) {
  o << "idx,type,t_ns,z_flag_before,xy_flag_before";
  for (const char* n : {"z", "zdot", "zbias"}) o << ',' << n;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) o << ",zP" << i << j;
  for (const char* n : {"vx", "vy", "pitch_bias", "roll_bias", "ax_bias", "ay_bias"}) o << ',' << n;
  for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) o << ",xyP" << i << j;
  o << ",z_meas,zR,xy_meas0,xy_meas1,xyR00,xyR11,u,z_dt,xy_dt,g_init,acc_init,takeoff,"
       "z_flag,xy_flag,n_prop,maha2,use_mocap_xy,use_mocap_z,n_published,delivered\n";
}

}  // namespace

// A6 (P05/R1): what the original published. With REF_PUBLISHED=<path> the
// harness writes one row per event that published xyz_estimate, with the
// fields of that message and of the xyz_debug_estimate published with it.
// The column order is shared with the port's driver (tools/event_replay.cpp).
namespace {
reef_msgs::XYZEstimate g_estimate;
reef_msgs::XYZDebugEstimate g_debug;
long g_debug_count = 0;
}
namespace reef_msgs {
void record_published(const std::string&, const XYZEstimate& m) { g_estimate = m; }
void record_published(const std::string&, const XYZDebugEstimate& m) { g_debug = m; ++g_debug_count; }
}
namespace {
template <class A> void put_array(std::ostream& o, const A& a) { for (double v : a) put(o, v); }
void put_z(std::ostream& o, const reef_msgs::ZDebugEstimate& z) {
  put(o, z.z); put(o, z.z_dot); put(o, z.bias); put(o, z.u);
  put_array(o, z.P); put_array(o, z.sigma_plus); put_array(o, z.sigma_minus);
}
void put_xy(std::ostream& o, const reef_msgs::XYDebugEstimate& x) {
  put(o, x.x_dot); put(o, x.y_dot); put(o, x.pitch_bias); put(o, x.roll_bias); put(o, x.xa_bias); put(o, x.ya_bias);
  put_array(o, x.sigma_plus); put_array(o, x.sigma_minus);
}
void published_header(std::ostream& o) {
  o << "idx,t_ns,pub_z,pub_zdot,pub_x_dot,pub_y_dot,debug";
  for (const char* g : {"dz", "mz"}) {
    for (const char* n : {"z", "zdot", "bias", "u"}) o << ',' << g << '_' << n;
    for (int k = 0; k < 9; ++k) o << ',' << g << "_p" << k;
    for (int k = 0; k < 3; ++k) o << ',' << g << "_sp" << k;
    for (int k = 0; k < 3; ++k) o << ',' << g << "_sm" << k;
    const char* xy = g[0] == 'd' ? "dxy" : "mxy";
    for (const char* n : {"x_dot", "y_dot", "pitch_bias", "roll_bias", "xa_bias", "ya_bias"}) o << ',' << xy << '_' << n;
    for (int k = 0; k < 6; ++k) o << ',' << xy << "_sp" << k;
    for (int k = 0; k < 6; ++k) o << ',' << xy << "_sm" << k;
  }
  o << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) { std::fprintf(stderr, "usage: %s PARAMS EVENTS OUT.csv [--log]\n", argv[0]); return 2; }
  ros::log_enabled() = (argc > 4 && std::strcmp(argv[4], "--log") == 0);
  load_params(argv[1]);

  reef_estimator::SensorManager sm;   // constructs the XYZEstimator exactly as the ROS node does
  reef_estimator::XYZEstimator& e = sm.xyzEst;
  // Harness-side initialization (adaptation A3): this member is uninitialized
  // in the original constructor and only written by a gate before being read,
  // so NaN marks "no gate evaluated yet" without affecting the estimator.
  e.Mahalanobis_D_hat_square.setConstant(std::numeric_limits<double>::quiet_NaN());

  // Deliver an event only if the ROS 1 node would have subscribed to its topic:
  // SensorManager subscribes to rc_raw only with enable_mocap_switch, mocap twist
  // only with enable_mocap_xy, mocap pose only with enable_mocap_z, rgbd only with
  // enable_rgbd, sonar only with enable_sonar (sensor_manager.cpp, constructor).
  bool rc_subscribed = false;
  ros::NodeHandle("~").param<bool>("enable_mocap_switch", rc_subscribed, false);
  auto subscribed = [&](const std::string& type) {
    if (type == "rc") return rc_subscribed;
    if (type == "mocap_twist") return e.enableMocapXY;
    if (type == "mocap_pose") return e.enableMocapZ;
    if (type == "rgbd") return e.enableRGBD;
    if (type == "range") return e.enableSonar;
    return true;   // imu, imu_nan
  };

  std::ifstream in(argv[2]);
  if (!in) { std::fprintf(stderr, "cannot open %s\n", argv[2]); return 2; }
  std::ofstream out(argv[3]);
  std::ofstream published;
  if (const char* path = std::getenv("REF_PUBLISHED")) {
    published.open(path);
    published << std::setprecision(std::numeric_limits<double>::max_digits10);
    published_header(published);
  }
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  header(out);

  std::string line;
  long idx = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string type; long long t_ns;
    ss >> type;
    t_ns = static_cast<long long>(std::stoll([&] { std::string t; ss >> t; return t; }()));
    const bool z_before = e.newSonarMeasurement, xy_before = e.newRgbdMeasurement;
    const long pub_before = ros::publish_counts()["xyz_estimate"], dbg_before = g_debug_count;

    if (!subscribed(type)) {
      // not delivered: the row records the unchanged state
    } else if (type == "imu" || type == "imu_nan") {
      auto m = boost::make_shared<sensor_msgs::Imu>();
      m->header.stamp = stamp_of(t_ns);
      if (type == "imu") {
        m->linear_acceleration.x = num(ss); m->linear_acceleration.y = num(ss); m->linear_acceleration.z = num(ss);
        m->orientation.x = num(ss); m->orientation.y = num(ss); m->orientation.z = num(ss); m->orientation.w = num(ss);
      } else {
        m->linear_acceleration.x = m->linear_acceleration.y = m->linear_acceleration.z =
            std::numeric_limits<double>::quiet_NaN();
      }
      sm.imuCallback(m);
    } else if (type == "range") {
      auto m = boost::make_shared<sensor_msgs::Range>();
      m->header.stamp = stamp_of(t_ns);
      m->range = static_cast<float>(num(ss));       // float32 fields, like the ROS 1 message
      m->max_range = static_cast<float>(num(ss));
      sm.altimeterCallback(m);
    } else if (type == "mocap_pose") {
      auto m = boost::make_shared<geometry_msgs::PoseStamped>();
      m->header.stamp = stamp_of(t_ns);
      m->pose.position.z = num(ss);
      sm.mocapPoseCallback(m);
    } else if (type == "mocap_twist" || type == "rgbd") {
      geometry_msgs::TwistWithCovarianceStamped tw;
      tw.header.stamp = stamp_of(t_ns);
      tw.twist.twist.linear.x = num(ss); tw.twist.twist.linear.y = num(ss);
      tw.twist.covariance[0] = num(ss); tw.twist.covariance[7] = num(ss);
      if (type == "mocap_twist") {
        sm.mocapTwistCallback(boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>(tw));
      } else {
        auto d = boost::make_shared<reef_msgs::DeltaToVel>();
        d->header = tw.header; d->vel = tw;
        sm.rgbdTwistCallback(d);
      }
    } else if (type == "rc") {
      auto m = boost::make_shared<rosflight_msgs::RCRaw>();
      m->header.stamp = stamp_of(t_ns);
      for (auto& v : m->values) v = static_cast<uint16_t>(num(ss));
      sm.rcRawCallback(m);
    } else {
      std::fprintf(stderr, "unknown event type '%s'\n", type.c_str());
      return 2;
    }

    if (published.is_open() && ros::publish_counts()["xyz_estimate"] != pub_before) {
      const bool dbg = g_debug_count != dbg_before;
      published << idx << ',' << t_ns;
      put(published, g_estimate.z_plus.z); put(published, g_estimate.z_plus.z_dot);
      put(published, g_estimate.xy_plus.x_dot); put(published, g_estimate.xy_plus.y_dot);
      published << ',' << dbg;
      const reef_msgs::XYZDebugEstimate d = dbg ? g_debug : reef_msgs::XYZDebugEstimate{};
      put_z(published, d.z_plus); put_xy(published, d.xy_plus);
      put_z(published, d.z_minus); put_xy(published, d.xy_minus);
      published << '\n';
    }
    out << idx++ << ',' << type << ',' << t_ns << ',' << z_before << ',' << xy_before;
    put_matrix(out, e.zEst.xHat); put_matrix(out, e.zEst.P);
    put_matrix(out, e.xyEst.xHat); put_matrix(out, e.xyEst.P);
    put(out, e.zEst.z(0)); put(out, e.zEst.R(0, 0));
    put(out, e.xyEst.z(0)); put(out, e.xyEst.z(1)); put(out, e.xyEst.R(0, 0)); put(out, e.xyEst.R(1, 1));
    put(out, e.zEst.u(0)); put(out, e.zEst.dt); put(out, e.xyEst.dt);
    put(out, e.accInitialized ? e.initialAccMagnitude : std::numeric_limits<double>::quiet_NaN());
    out << ',' << e.accInitialized << ',' << e.takeoffState.data << ',' << e.newSonarMeasurement << ','
        << e.newRgbdMeasurement << ',' << e.numberOfPropagations;
    put(out, e.Mahalanobis_D_hat_square(0));
    out << ',' << e.useMocapXY << ',' << e.useMocapZ << ',' << ros::publish_counts()["xyz_estimate"] << ','
        << subscribed(type) << '\n';
  }
  return 0;
}

// Reference harness: drives the UNMODIFIED pinned reef_control sources
// (12237b76: Controller + PIDController + SimplePID) with a deterministic
// event fixture and writes the complete controller state after every event.
//
//   control_ref PARAMS EVENTS OUT.csv [--log]
//
// PARAMS: one parameter per line:  <name> bool|int|double <value>
//         a leading '~' marks the node's private namespace (as in roscpp).
// EVENTS: one event per line (stamps as integer sec nsec):
//   est    sec nsec z z_dot x_dot y_dot         reef_msgs/XYZEstimate (xyz_estimate)
//   des    sec nsec av pv vv acv ao  px py pz pyaw  vx vy vz vyaw  ax ay az ayaw  tx ty tz tyaw
//                                               reef_msgs/DesiredState (desired_state); flags 0/1;
//                                               p* pose, v* velocity, a* acceleration, t* attitude
//   status armed                                rosflight_msgs/Status (status)
//   flying data                                 std_msgs/Bool (is_flying)
//   pose   x y qx qy qz qw                      geometry_msgs/PoseStamped (pose_stamped)
//   gain   name value                           runtime dynamic_reconfigure change of one field
// Lines starting with '#' are comments. Output columns: control_columns.txt
// (shared with the port's replay tool).
//
// Adaptations (baseline/README.md): C1 access keywords relaxed after all
// library headers (access_prelude.h); C2 dynamic_reconfigure stand-in;
// C3 the controller object is constructed in zeroed storage, so members the
// original leaves uninitialized (PIDController::theta, used by face_target
// before the first lookup) read as 0.
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <new>
#include <sstream>

#include "PID.h"

namespace ros {
std::map<std::string, ParamValue>& param_store() { static std::map<std::string, ParamValue> s; return s; }
std::map<std::string, long>& Publisher::publish_counts() { static std::map<std::string, long> c; return c; }
bool& log_enabled() { static bool on = false; return on; }
}  // namespace ros

namespace {
rosflight_msgs::Command g_cmd;
reef_msgs::DesiredState g_cs;
}  // namespace
namespace rosflight_msgs { void record_published(const std::string&, const Command& m) { g_cmd = m; } }
namespace reef_msgs { void record_published(const std::string&, const DesiredState& m) { g_cs = m; } }

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
long long integer(std::istringstream& ss) {
  std::string tok;
  if (!(ss >> tok)) die("missing integer field");
  char* end = nullptr;
  const long long v = std::strtoll(tok.c_str(), &end, 10);
  if (end == tok.c_str() || *end != '\0') die("bad integer '" + tok + "'");
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
    if (kind == "bool") { v.kind = ros::ParamValue::Bool; v.b = integer(ss) != 0; }
    else if (kind == "int") { v.kind = ros::ParamValue::Int; v.i = static_cast<int>(integer(ss)); }
    else if (kind == "double") { v.kind = ros::ParamValue::Double; v.d = num(ss); }
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
void put_f(std::ostream& o, float x) {
  char b[40];
  if (std::isnan(x)) std::snprintf(b, sizeof b, "nan");
  else std::snprintf(b, sizeof b, "%.9g", static_cast<double>(x));
  o << ',' << b;
}
void put_i(std::ostream& o, long long x) { o << ',' << x; }

void put_vec(std::ostream& o, const reef_msgs::DesiredVector& v) { put(o, v.x); put(o, v.y); put(o, v.z); put(o, v.yaw); }
void put_ds(std::ostream& o, const reef_msgs::DesiredState& d) {
  put_i(o, d.header.stamp.sec); put_i(o, d.header.stamp.nsec);
  put_vec(o, d.pose); put_vec(o, d.velocity); put_vec(o, d.acceleration); put_vec(o, d.attitude);
  put_i(o, d.attitude_valid); put_i(o, d.position_valid); put_i(o, d.velocity_valid);
  put_i(o, d.acceleration_valid); put_i(o, d.altitude_only);
}
void put_pid(std::ostream& o, const reef_control::SimplePID& p) {
  put(o, p.kp_); put(o, p.ki_); put(o, p.kd_); put(o, p.tau_); put(o, p.max_); put(o, p.min_);
  put(o, p.integrator_); put(o, p.differentiator_); put(o, p.last_error_); put(o, p.last_state_);
}

void write_row(std::ostream& o, long idx, const std::string& kind, const reef_control::PIDController& c) {
  auto& pc = ros::Publisher::publish_counts();
  o << idx << ',' << kind;
  put_i(o, pc["command"]); put_i(o, pc["controller_state"]);
  // last published command and controller_state
  put_i(o, g_cmd.header.stamp.sec); put_i(o, g_cmd.header.stamp.nsec);
  put_i(o, g_cmd.mode); put_i(o, g_cmd.ignore);
  put_f(o, g_cmd.x); put_f(o, g_cmd.y); put_f(o, g_cmd.z); put_f(o, g_cmd.F);
  put_ds(o, g_cs);
  // stored desired state and current state
  put_ds(o, c.desired_state_);
  const auto& s = c.current_state_;
  put_i(o, s.header.stamp.sec); put_i(o, s.header.stamp.nsec);
  put(o, s.pose.pose.position.x); put(o, s.pose.pose.position.y); put(o, s.pose.pose.position.z);
  put(o, s.pose.pose.orientation.x); put(o, s.pose.pose.orientation.y); put(o, s.pose.pose.orientation.z); put(o, s.pose.pose.orientation.w);
  put(o, s.twist.twist.linear.x); put(o, s.twist.twist.linear.y); put(o, s.twist.twist.linear.z);
  // controller internals
  put_i(o, c.armed_); put_i(o, c.initialized_); put_i(o, c.is_flying_);
  put_i(o, c.time_of_previous_control_.sec); put_i(o, c.time_of_previous_control_.nsec);
  put(o, c.dt); put(o, c.phi_desired); put(o, c.theta_desired); put(o, c.thrust);
  put(o, c.max_roll_); put(o, c.max_pitch_); put(o, c.max_yaw_rate_);
  put(o, c.current_yaw); put(o, c.theta);
  put(o, c.kp); put(o, c.deadzone); put(o, c.vel_max); put(o, c.x_0); put(o, c.alpha); put(o, c.sigma);
  put_i(o, c.face_target_); put_i(o, c.fly_fixed_wing_);
  put_pid(o, c.d_); put_pid(o, c.w_); put_pid(o, c.yaw_); put_pid(o, c.u_); put_pid(o, c.v_);
  o << '\n';
}

ros::Time stamp(std::istringstream& ss) {
  const long long s = integer(ss), n = integer(ss);
  if (s < 0 || n < 0 || n >= 1000000000LL) die("bad stamp");
  return ros::Time(static_cast<uint32_t>(s), static_cast<uint32_t>(n));
}
void vec(std::istringstream& ss, reef_msgs::DesiredVector& v) { v.x = num(ss); v.y = num(ss); v.z = num(ss); v.yaw = num(ss); }

template <class M> void deliver(const std::string& topic, const M& m) {
  auto& subs = ros::subscriptions<M>();
  auto it = subs.find(topic);
  if (it == subs.end()) die("no subscription on " + topic);
  it->second(m);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) die("usage: control_ref PARAMS EVENTS OUT.csv [--log]");
  ros::log_enabled() = (argc > 4 && std::string(argv[4]) == "--log");
  load_params(argv[1]);

  // C3: zeroed storage (static storage is zero-initialized).
  alignas(reef_control::PIDController) static unsigned char storage[sizeof(reef_control::PIDController)];
  std::memset(storage, 0, sizeof storage);
  auto* ctl = new (storage) reef_control::PIDController();

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
    if (kind == "est") {
      reef_msgs::XYZEstimate m;
      m.header.stamp = stamp(ss);
      m.z_plus.z = num(ss); m.z_plus.z_dot = num(ss); m.xy_plus.x_dot = num(ss); m.xy_plus.y_dot = num(ss);
      deliver("xyz_estimate", m);
    } else if (kind == "des") {
      reef_msgs::DesiredState m;
      m.header.stamp = stamp(ss);
      m.attitude_valid = integer(ss) != 0; m.position_valid = integer(ss) != 0; m.velocity_valid = integer(ss) != 0;
      m.acceleration_valid = integer(ss) != 0; m.altitude_only = integer(ss) != 0;
      vec(ss, m.pose); vec(ss, m.velocity); vec(ss, m.acceleration); vec(ss, m.attitude);
      deliver("desired_state", m);
    } else if (kind == "status") {
      rosflight_msgs::Status m; m.armed = integer(ss) != 0;
      deliver("status", m);
    } else if (kind == "flying") {
      std_msgs::Bool m; m.data = integer(ss) != 0;
      deliver("is_flying", m);
    } else if (kind == "pose") {
      geometry_msgs::PoseStamped m;
      m.pose.position.x = num(ss); m.pose.position.y = num(ss);
      m.pose.orientation.x = num(ss); m.pose.orientation.y = num(ss); m.pose.orientation.z = num(ss); m.pose.orientation.w = num(ss);
      deliver("pose_stamped", m);
    } else if (kind == "gain") {
      std::string name; ss >> name;
      const double v = num(ss);
      bool ok = true;
      ctl->server_.update([&](reef_control::GainsConfig& c) { ok = c.set(name, v); });
      if (!ok) die("unknown gain " + name);
    } else {
      die("unknown event: " + line);
    }
    std::string extra;
    if (ss >> extra) die("trailing fields: " + line);
    write_row(out, idx++, kind, *ctl);
  }
  return 0;
}

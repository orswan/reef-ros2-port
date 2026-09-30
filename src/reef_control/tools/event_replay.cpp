// Event replay for the fidelity comparison with the reference harness
// (baseline/control/control_ref_main.cpp): same PARAMS and EVENTS formats,
// same output columns (baseline/control/control_columns.txt).
//
//   reef_control_event_replay PARAMS EVENTS OUT.csv [--mode core|node]
//
// core: the ROS-free PIDController, called directly.
// node: the ROS 2 ControlNode (parameters as overrides, messages converted
//       by the node, gain events through set_parameter); the command columns
//       come from the ROS messages it published. In node mode cmd_sec and
//       cmd_nsec are the node's header stamp (the original left them zero).
// Parameter names: '~name' is the node's parameter; of the global names
// only face_target and fly_fixed_wing are read (the original read those
// globally, CONTROL_CHAIN.md K12). Unknown names are ignored, as the
// original ignored unused parameters.
// Exit: 0 done, 2 bad input, 3 invalid parameters.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "reef_control/control_node.hpp"

using namespace reef_control;

namespace
{

[[noreturn]] void die(const std::string & m, int code = 2)
{
  std::fprintf(stderr, "%s\n", m.c_str());
  std::exit(code);
}

double num(std::istringstream & ss)
{
  std::string tok;
  if (!(ss >> tok)) {die("missing numeric field");}
  char * end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') {die("bad number '" + tok + "'");}
  return v;
}

long long integer(std::istringstream & ss)
{
  std::string tok;
  if (!(ss >> tok)) {die("missing integer field");}
  char * end = nullptr;
  const long long v = std::strtoll(tok.c_str(), &end, 10);
  if (end == tok.c_str() || *end != '\0') {die("bad integer '" + tok + "'");}
  return v;
}

struct ParamLine { std::string kind; double d = 0; long long i = 0; };

// Parameters of the file that the port reads, keyed by the port's name.
std::map<std::string, ParamLine> read_params(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {die("cannot open " + path);}
  std::map<std::string, ParamLine> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    std::string name;
    ParamLine p;
    ss >> name >> p.kind;
    if (p.kind == "double") {p.d = num(ss);} else if (p.kind == "bool" || p.kind == "int") {
      p.i = integer(ss);
    } else {die("bad param line: " + line);}
    std::string port;
    if (name.size() > 1 && name[0] == '~') {
      port = name.substr(1);
      if (port == "face_target" || port == "fly_fixed_wing") {continue;}   // K12: ignored by the original
    } else if (name == "face_target" || name == "fly_fixed_wing") {
      port = name;
    } else {
      continue;
    }
    out[port] = p;
  }
  return out;
}

rclcpp::ParameterValue to_value(const ParamLine & p)
{
  if (p.kind == "double") {return rclcpp::ParameterValue(p.d);}
  if (p.kind == "bool") {return rclcpp::ParameterValue(p.i != 0);}
  return rclcpp::ParameterValue(static_cast<int64_t>(p.i));
}

// Core parameters from the file, with the node's rules (integers accepted
// for numbers; missing limits and invalid values are errors).
ControllerParameters core_params(const std::map<std::string, ParamLine> & ps)
{
  ControllerParameters c;
  std::string errors;
  auto number = [&](const std::string & n, double & v, bool required) {
      auto it = ps.find(n);
      if (it == ps.end()) {if (required) {errors += "\n  " + n + " is required";} return;}
      if (it->second.kind == "double") {v = it->second.d;} else if (it->second.kind == "int") {
        v = static_cast<double>(it->second.i);
      } else {errors += "\n  " + n + ": expected a number";}
    };
  auto flag = [&](const std::string & n, bool & v) {
      auto it = ps.find(n);
      if (it == ps.end()) {return;}
      if (it->second.kind == "bool") {v = it->second.i != 0;} else {errors += "\n  " + n + ": expected a bool";}
    };
  number("max_roll", c.max_roll, true);
  number("max_pitch", c.max_pitch, true);
  number("max_yaw_rate", c.max_yaw_rate, true);
  flag("face_target", c.face_target);
  flag("fly_fixed_wing", c.fly_fixed_wing);
  for (const auto & r : gainRanges()) {number(r.name, *gainField(c.gains, r.name), false);}
  for (const auto & b : gainBools()) {flag(b, *gainBoolField(c.gains, b));}
  for (const auto & e : parameterErrors(c)) {errors += "\n  " + e;}
  if (!errors.empty()) {die("invalid reef_control parameters:" + errors, 3);}
  return c;
}

void put(std::ostream & o, double x)
{
  char b[40];
  if (std::isnan(x)) {std::snprintf(b, sizeof b, "nan");} else {std::snprintf(b, sizeof b, "%.17g", x);}
  o << ',' << b;
}
void put_f(std::ostream & o, float x)
{
  char b[40];
  if (std::isnan(x)) {
    std::snprintf(b, sizeof b, "nan");
  } else {
    std::snprintf(b, sizeof b, "%.9g", static_cast<double>(x));
  }
  o << ',' << b;
}
void put_i(std::ostream & o, long long x) {o << ',' << x;}
void put_vec(std::ostream & o, const DesiredVector & v) {put(o, v.x); put(o, v.y); put(o, v.z); put(o, v.yaw);}
void put_ds(std::ostream & o, const DesiredState & d)
{
  put_i(o, d.header.stamp.sec); put_i(o, d.header.stamp.nsec);
  put_vec(o, d.pose); put_vec(o, d.velocity); put_vec(o, d.acceleration); put_vec(o, d.attitude);
  put_i(o, d.attitude_valid); put_i(o, d.position_valid); put_i(o, d.velocity_valid);
  put_i(o, d.acceleration_valid); put_i(o, d.altitude_only);
}
void put_pid(std::ostream & o, const SimplePID & p)
{
  put(o, p.kp()); put(o, p.ki()); put(o, p.kd()); put(o, p.tau()); put(o, p.max()); put(o, p.min());
  put(o, p.integrator()); put(o, p.differentiator()); put(o, p.lastError()); put(o, p.lastState());
}

// Last published outputs, in the core's types.
struct Published
{
  Stamp cmd_stamp;
  Command cmd;
  DesiredState cs;
};

void write_row(std::ostream & o, long idx, const std::string & kind, const PIDController & c, const Published & pub)
{
  o << idx << ',' << kind;
  put_i(o, c.commandCount()); put_i(o, c.controllerStateCount());
  put_i(o, pub.cmd_stamp.sec); put_i(o, pub.cmd_stamp.nsec);
  put_i(o, pub.cmd.mode); put_i(o, pub.cmd.ignore);
  put_f(o, pub.cmd.x); put_f(o, pub.cmd.y); put_f(o, pub.cmd.z); put_f(o, pub.cmd.F);
  put_ds(o, pub.cs);
  put_ds(o, c.desiredState());
  const Odometry & s = c.currentState();
  put_i(o, s.header.stamp.sec); put_i(o, s.header.stamp.nsec);
  put(o, s.pose.pose.position.x); put(o, s.pose.pose.position.y); put(o, s.pose.pose.position.z);
  put(o, s.pose.pose.orientation.x); put(o, s.pose.pose.orientation.y);
  put(o, s.pose.pose.orientation.z); put(o, s.pose.pose.orientation.w);
  put(o, s.twist.twist.linear.x); put(o, s.twist.twist.linear.y); put(o, s.twist.twist.linear.z);
  put_i(o, c.armed()); put_i(o, c.initialized_); put_i(o, c.isFlying());
  put_i(o, c.timeOfPreviousControl().sec); put_i(o, c.timeOfPreviousControl().nsec);
  put(o, c.lastDt()); put(o, c.phiDesired()); put(o, c.thetaDesired()); put(o, c.thrustDesired());
  put(o, c.maxRoll()); put(o, c.maxPitch()); put(o, c.maxYawRate());
  put(o, c.currentYaw()); put(o, c.headingToTarget());
  put(o, c.lookupKp()); put(o, c.lookupDeadzone()); put(o, c.lookupVelMax()); put(o, c.lookupCenter());
  put(o, c.lookupAlpha()); put(o, c.lookupSigma());
  put_i(o, c.faceTarget()); put_i(o, c.flyFixedWing());
  put_pid(o, c.dPID()); put_pid(o, c.wPID()); put_pid(o, c.yawPID()); put_pid(o, c.uPID()); put_pid(o, c.vPID());
  o << '\n';
}

Stamp stamp(std::istringstream & ss)
{
  const long long s = integer(ss), n = integer(ss);
  if (s < 0 || n < 0 || n >= 1000000000LL) {die("bad stamp");}
  Stamp t;
  t.sec = s;
  t.nsec = static_cast<uint32_t>(n);
  return t;
}
void vec(std::istringstream & ss, DesiredVector & v) {v.x = num(ss); v.y = num(ss); v.z = num(ss); v.yaw = num(ss);}

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
    die("usage: reef_control_event_replay PARAMS EVENTS OUT.csv [--mode core|node]");
  }
  const auto file_params = read_params(pos[0]);

  std::unique_ptr<PIDController> core;
  std::shared_ptr<ControlNode> node;
  Published pub;
  if (mode == "core") {
    core = std::make_unique<PIDController>(core_params(file_params));
    core->command_publisher_ = [&pub](const Command & c) {pub.cmd = c;};
    core->desired_state_pub_ = [&pub](const DesiredState & d) {pub.cs = d;};
  } else {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> overrides;
    for (const auto & [name, p] : file_params) {overrides.emplace_back(name, to_value(p));}
    opts.parameter_overrides(overrides);
    opts.use_global_arguments(false);
    try {
      node = std::make_shared<ControlNode>(opts);
    } catch (const ParameterError & e) {
      die(e.what(), 3);
    }
  }
  const PIDController & ctl = core ? *core : node->controller();

  std::ifstream in(pos[1]);
  if (!in) {die("cannot open " + pos[1]);}
  std::ofstream out(pos[2]);
  if (!out) {die("cannot write " + pos[2]);}
  std::string line;
  long idx = 0;
  std::map<std::string, double> gain_changes;   // core mode: runtime changes so far
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    std::string kind;
    ss >> kind;
    if (kind == "est") {
      XYZEstimate m;
      m.header.stamp = stamp(ss);
      m.z_plus.z = num(ss); m.z_plus.z_dot = num(ss); m.xy_plus.x_dot = num(ss); m.xy_plus.y_dot = num(ss);
      if (core) {
        core->currentStateCallback(m);
      } else {
        reef_msgs::msg::XYZEstimate r;
        r.header.stamp = toRos(m.header.stamp);
        r.z_plus.z = m.z_plus.z; r.z_plus.z_dot = m.z_plus.z_dot;
        r.xy_plus.x_dot = m.xy_plus.x_dot; r.xy_plus.y_dot = m.xy_plus.y_dot;
        node->onEstimate(r);
      }
    } else if (kind == "des") {
      DesiredState m;
      m.header.stamp = stamp(ss);
      m.attitude_valid = integer(ss) != 0; m.position_valid = integer(ss) != 0; m.velocity_valid = integer(ss) != 0;
      m.acceleration_valid = integer(ss) != 0; m.altitude_only = integer(ss) != 0;
      vec(ss, m.pose); vec(ss, m.velocity); vec(ss, m.acceleration); vec(ss, m.attitude);
      if (core) {core->desiredStateCallback(m);} else {node->onDesiredState(toRos(m));}
    } else if (kind == "status") {
      Status m;
      m.armed = integer(ss) != 0;
      if (core) {
        core->statusCallback(m);
      } else {
        rosflight_msgs::msg::Status r;
        r.armed = m.armed;
        node->onStatus(r);
      }
    } else if (kind == "flying") {
      Bool m;
      m.data = integer(ss) != 0;
      if (core) {
        core->isflyingCallback(m);
      } else {
        std_msgs::msg::Bool r;
        r.data = m.data;
        node->onIsFlying(r);
      }
    } else if (kind == "pose") {
      PoseStamped m;
      m.pose.position.x = num(ss); m.pose.position.y = num(ss);
      m.pose.orientation.x = num(ss); m.pose.orientation.y = num(ss);
      m.pose.orientation.z = num(ss); m.pose.orientation.w = num(ss);
      if (core) {
        core->poseCallback(m);
      } else {
        geometry_msgs::msg::PoseStamped r;
        r.pose.position.x = m.pose.position.x; r.pose.position.y = m.pose.position.y;
        r.pose.orientation.x = m.pose.orientation.x; r.pose.orientation.y = m.pose.orientation.y;
        r.pose.orientation.z = m.pose.orientation.z; r.pose.orientation.w = m.pose.orientation.w;
        node->onPose(r);
      }
    } else if (kind == "gain") {
      std::string name;
      ss >> name;
      const double v = num(ss);
      if (core) {
        GainsConfig g = core_params(file_params).gains;   // start value, then every change so far
        gain_changes[name] = v;
        for (const auto & [n, x] : gain_changes) {
          if (double * f = gainField(g, n)) {*f = x;} else if (bool * b = gainBoolField(g, n)) {
            *b = x != 0;
          } else {die("unknown gain " + n);}
        }
        const auto errors = gainErrors(g);
        if (!errors.empty()) {die("rejected gain change: " + errors.front(), 3);}
        core->gainsCallback(g);
      } else {
        GainsConfig probe;
        const rclcpp::Parameter p = gainBoolField(probe, name) ? rclcpp::Parameter(name, v != 0) : rclcpp::Parameter(name, v);
        const auto r = node->set_parameter(p);
        if (!r.successful) {die("rejected gain change: " + r.reason, 3);}
      }
    } else {
      die("unknown event: " + line);
    }
    std::string extra;
    if (ss >> extra) {die("trailing fields: " + line);}
    if (node) {
      const auto & c = node->lastCommand();
      pub.cmd_stamp = toStamp(c.header.stamp);
      pub.cmd.mode = c.mode;
      pub.cmd.ignore = static_cast<uint8_t>(c.ignore);
      pub.cmd.x = c.u[0]; pub.cmd.y = c.u[1]; pub.cmd.z = c.u[2]; pub.cmd.F = c.u[3];
      pub.cs = fromRos(node->lastControllerState());
    }
    write_row(out, idx++, kind, ctl, pub);
  }
  if (node) {rclcpp::shutdown();}
  return 0;
}

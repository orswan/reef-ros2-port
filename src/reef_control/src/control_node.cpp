#include "reef_control/control_node.hpp"

#include <functional>
#include <sstream>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/exceptions.hpp>

namespace reef_control
{

namespace
{

using rcl_interfaces::msg::ParameterDescriptor;

ParameterDescriptor describe(const std::string & text, bool read_only)
{
  ParameterDescriptor d;
  d.description = text;
  d.read_only = read_only;
  return d;
}

// A number parameter value (integers accepted, as roscpp getParam(double) did).
bool asNumber(const rclcpp::ParameterValue & v, double & out)
{
  if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {out = v.get<double>(); return true;}
  if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    out = static_cast<double>(v.get<int64_t>());
    return true;
  }
  return false;
}

}  // namespace

ControllerParameters loadParameters(rclcpp::Node & node)
{
  ControllerParameters p;
  std::vector<std::string> errors;
  auto attempt = [&](const std::string & name, const std::function<void()> & f) {
      try {
        f();
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        errors.push_back(name + ": wrong type (" + e.what() + ")");
      } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
        errors.push_back(name + ": invalid value (" + e.what() + ")");
      } catch (const ParameterError & e) {
        errors.push_back(e.what());
      }
    };
  // Number that must be given (the original: ROS_ASSERT(getParam(...))).
  auto required = [&](const std::string & name, double & value, const std::string & text) {
      attempt(name, [&] {
          ParameterDescriptor d = describe(text, true);
          d.dynamic_typing = true;
          const rclcpp::ParameterValue v = node.declare_parameter(name, rclcpp::ParameterValue(), d);
          if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            throw ParameterError(name + " is required (the original asserted it)");
          }
          if (!asNumber(v, value)) {
            throw ParameterError(name + " has type " + rclcpp::to_string(v.get_type()) + "; expected a number");
          }
        });
    };
  auto flag = [&](const std::string & name, bool & value, const std::string & text, bool read_only) {
      attempt(name, [&] {
          value = node.declare_parameter(name, rclcpp::ParameterValue(value), describe(text, read_only)).get<bool>();
        });
    };

  required("max_roll", p.max_roll, "roll command limit [rad] (command x clamp)");
  required("max_pitch", p.max_pitch, "pitch command limit [rad] (command y clamp)");
  required("max_yaw_rate", p.max_yaw_rate,
    "yaw rate command limit [rad/s] (command z clamp). Not the unused Gains.cfg field of the same name");
  flag("face_target", p.face_target,
    "position mode: heading follows the direction to the target (read from the global namespace by the original)", true);
  flag("fly_fixed_wing", p.fly_fixed_wing,
    "position mode: scale the velocity request down while yawing (read from the global namespace by the original)", true);

  for (const auto & r : gainRanges()) {
    double & value = *gainField(p.gains, r.name);
    attempt(r.name, [&] {
        std::ostringstream text;
        text << "reef_control gain (cfg/Gains.cfg), range [" << r.lo << ", " << r.hi << "]; changeable at runtime";
        ParameterDescriptor d = describe(text.str(), false);
        d.dynamic_typing = true;
        const rclcpp::ParameterValue v = node.declare_parameter(r.name, rclcpp::ParameterValue(value), d);
        if (!asNumber(v, value)) {
          throw ParameterError(r.name + " has type " + rclcpp::to_string(v.get_type()) + "; expected a number");
        }
      });
  }
  for (const auto & name : gainBools()) {
    flag(name, *gainBoolField(p.gains, name),
      "reef_control integrator switch (cfg/Gains.cfg); changeable at runtime", false);
  }

  if (errors.empty()) {
    errors = parameterErrors(p);
  }
  if (!errors.empty()) {
    std::string message = "invalid reef_control parameters:";
    for (const auto & e : errors) {
      message += "\n  " + e;
    }
    throw ParameterError(message);
  }
  return p;
}

Stamp toStamp(const builtin_interfaces::msg::Time & t)
{
  Stamp s;
  s.sec = t.sec;
  s.nsec = t.nanosec;
  return s;
}

builtin_interfaces::msg::Time toRos(const Stamp & s)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(s.sec);
  t.nanosec = s.nsec;
  return t;
}

namespace
{
DesiredVector fromRos(const reef_msgs::msg::DesiredVector & v) {return {v.x, v.y, v.z, v.yaw};}
reef_msgs::msg::DesiredVector toRos(const DesiredVector & v)
{
  reef_msgs::msg::DesiredVector m;
  m.x = v.x; m.y = v.y; m.z = v.z; m.yaw = v.yaw;
  return m;
}
}  // namespace

DesiredState fromRos(const reef_msgs::msg::DesiredState & m)
{
  DesiredState d;
  d.header.stamp = toStamp(m.header.stamp);
  d.node_id = m.node_id;
  d.pose = fromRos(m.pose);
  d.velocity = fromRos(m.velocity);
  d.acceleration = fromRos(m.acceleration);
  d.attitude = fromRos(m.attitude);
  d.attitude_valid = m.attitude_valid;
  d.position_valid = m.position_valid;
  d.velocity_valid = m.velocity_valid;
  d.acceleration_valid = m.acceleration_valid;
  d.altitude_only = m.altitude_only;
  return d;
}

reef_msgs::msg::DesiredState toRos(const DesiredState & d)
{
  reef_msgs::msg::DesiredState m;
  m.header.stamp = toRos(d.header.stamp);
  m.node_id = d.node_id;
  m.pose = toRos(d.pose);
  m.velocity = toRos(d.velocity);
  m.acceleration = toRos(d.acceleration);
  m.attitude = toRos(d.attitude);
  m.attitude_valid = d.attitude_valid;
  m.position_valid = d.position_valid;
  m.velocity_valid = d.velocity_valid;
  m.acceleration_valid = d.acceleration_valid;
  m.altitude_only = d.altitude_only;
  return m;
}

XYZEstimate fromRos(const reef_msgs::msg::XYZEstimate & m)
{
  XYZEstimate e;
  e.header.stamp = toStamp(m.header.stamp);
  e.xy_plus.x_dot = m.xy_plus.x_dot;
  e.xy_plus.y_dot = m.xy_plus.y_dot;
  e.z_plus.z = m.z_plus.z;
  e.z_plus.z_dot = m.z_plus.z_dot;
  return e;
}

PoseStamped fromRos(const geometry_msgs::msg::PoseStamped & m)
{
  PoseStamped p;
  p.pose.position.x = m.pose.position.x;
  p.pose.position.y = m.pose.position.y;
  p.pose.position.z = m.pose.position.z;
  p.pose.orientation.x = m.pose.orientation.x;
  p.pose.orientation.y = m.pose.orientation.y;
  p.pose.orientation.z = m.pose.orientation.z;
  p.pose.orientation.w = m.pose.orientation.w;
  return p;
}

rosflight_msgs::msg::Command toRos(const Command & c, const Stamp & stamp)
{
  rosflight_msgs::msg::Command m;
  m.header.stamp = toRos(stamp);
  m.mode = c.mode;
  m.ignore = c.ignore;
  m.u[0] = c.x;
  m.u[1] = c.y;
  m.u[2] = c.z;
  m.u[3] = c.F;
  return m;
}

ControlNode::ControlNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("reef_control_pid", options)
{
  const ControllerParameters params = loadParameters(*this);
  gains_ = params.gains;
  controller_ = std::make_unique<PIDController>(params);
  controller_->log = [this](const std::string & s) {RCLCPP_INFO(get_logger(), "%s", s.c_str());};
  controller_->gainsCallback(gains_);   // logs the gains, as setCallback did; same values

  // Queue depth 1 everywhere, as the original (ROS 1 queue size 1).
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  command_pub_ = create_publisher<rosflight_msgs::msg::Command>("command", qos);
  state_pub_ = create_publisher<reef_msgs::msg::DesiredState>("controller_state", qos);
  controller_->command_publisher_ = [this](const Command & c) {
      last_command_ = toRos(c, last_estimate_stamp_);
      command_pub_->publish(last_command_);
    };
  controller_->desired_state_pub_ = [this](const DesiredState & d) {
      last_state_ = toRos(d);
      state_pub_->publish(last_state_);
    };

  desired_sub_ = create_subscription<reef_msgs::msg::DesiredState>("desired_state", qos,
      [this](const reef_msgs::msg::DesiredState & m) {onDesiredState(m);});
  status_sub_ = create_subscription<rosflight_msgs::msg::Status>("status", qos,
      [this](const rosflight_msgs::msg::Status & m) {onStatus(m);});
  flying_sub_ = create_subscription<std_msgs::msg::Bool>("is_flying", qos,
      [this](const std_msgs::msg::Bool & m) {onIsFlying(m);});
  estimate_sub_ = create_subscription<reef_msgs::msg::XYZEstimate>("xyz_estimate", qos,
      [this](const reef_msgs::msg::XYZEstimate & m) {onEstimate(m);});
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("pose_stamped", qos,
      [this](const geometry_msgs::msg::PoseStamped & m) {onPose(m);});

  // Runtime gain changes (the original: dynamic_reconfigure). Out-of-range
  // values are rejected instead of clamped; accepted changes call
  // gainsCallback with the whole configuration, which keeps the integrators.
  validate_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      GainsConfig g = gains_;
      for (const auto & p : ps) {
        if (double * f = gainField(g, p.get_name())) {
          if (!asNumber(p.get_parameter_value(), *f)) {
            r.successful = false;
            r.reason = p.get_name() + ": expected a number";
          }
        } else if (bool * b = gainBoolField(g, p.get_name())) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            r.successful = false;
            r.reason = p.get_name() + ": expected a bool";
          } else {
            *b = p.as_bool();
          }
        }
      }
      if (r.successful) {
        const auto errors = gainErrors(g);
        if (!errors.empty()) {
          r.successful = false;
          r.reason = errors.front();
        }
      }
      return r;
    });
  apply_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & ps) {
      bool changed = false;
      for (const auto & p : ps) {
        if (double * f = gainField(gains_, p.get_name())) {
          asNumber(p.get_parameter_value(), *f);
          changed = true;
        } else if (bool * b = gainBoolField(gains_, p.get_name())) {
          *b = p.as_bool();
          changed = true;
        }
      }
      if (changed && controller_) {
        controller_->gainsCallback(gains_);
      }
    });
}

void ControlNode::onDesiredState(const reef_msgs::msg::DesiredState & m)
{
  controller_->desiredStateCallback(fromRos(m));
}

void ControlNode::onStatus(const rosflight_msgs::msg::Status & m)
{
  Status s;
  s.armed = m.armed;
  controller_->statusCallback(s);
}

void ControlNode::onIsFlying(const std_msgs::msg::Bool & m)
{
  Bool b;
  b.data = m.data;
  controller_->isflyingCallback(b);
}

void ControlNode::onEstimate(const reef_msgs::msg::XYZEstimate & m)
{
  const XYZEstimate e = fromRos(m);
  last_estimate_stamp_ = e.header.stamp;
  controller_->currentStateCallback(e);
}

void ControlNode::onPose(const geometry_msgs::msg::PoseStamped & m)
{
  controller_->poseCallback(fromRos(m));
}

}  // namespace reef_control

#include "reef_estimator/ros_parameters.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/parameter.hpp>

namespace reef_estimator
{

namespace
{

using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::node_interfaces::NodeParametersInterface;

ParameterDescriptor describe(const std::string & text, bool read_only = true)
{
  ParameterDescriptor d;
  d.description = text;
  d.read_only = read_only;
  return d;
}

class Loader
{
public:
  explicit Loader(NodeParametersInterface & params)
  : params_(params) {}

  // Runs one declaration/read; records an error instead of stopping.
  void attempt(const std::string & name, const std::function<void()> & f)
  {
    try {
      f();
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      errors_.push_back(name + ": wrong type (" + e.what() + ")");
    } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
      errors_.push_back(name + ": invalid value (" + e.what() + ")");
    } catch (const reef_msgs::ParameterError & e) {
      errors_.push_back(e.what());
    }
  }

  template<class T>
  void typed(const std::string & name, T & value, const ParameterDescriptor & d)
  {
    attempt(name, [&] {
        value = params_.declare_parameter(name, rclcpp::ParameterValue(value), d)
        .template get<T>();
      });
  }

  void integer(const std::string & name, int & value, ParameterDescriptor d, int lo, int hi)
  {
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = lo;
    range.to_value = hi;
    range.step = 1;
    d.integer_range.push_back(range);
    attempt(name, [&] {
        value = static_cast<int>(params_.declare_parameter(
          name, rclcpp::ParameterValue(static_cast<int64_t>(value)), d).get<int64_t>());
      });
  }

  // Double that also accepts an integer override (legacy roscpp did).
  void number(const std::string & name, double & value, ParameterDescriptor d)
  {
    d.dynamic_typing = true;
    attempt(name, [&] {
        const rclcpp::ParameterValue v =
        params_.declare_parameter(name, rclcpp::ParameterValue(value), d);
        switch (v.get_type()) {
          case rclcpp::ParameterType::PARAMETER_DOUBLE:
            value = v.get<double>();
            break;
          case rclcpp::ParameterType::PARAMETER_INTEGER:
            value = static_cast<double>(v.get<int64_t>());
            break;
          default:
            throw reef_msgs::ParameterError(name + " has type " + rclcpp::to_string(v.get_type()) +
                  "; expected a number");
        }
      });
  }

  template<class Derived>
  void matrix(const std::string & name, Eigen::MatrixBase<Derived> & m)
  {
    attempt(name, [&] {reef_msgs::importMatrixFromParameter(params_, m, name);});
  }

  std::vector<std::string> & errors() {return errors_;}

private:
  NodeParametersInterface & params_;
  std::vector<std::string> errors_;
};

}  // namespace

EstimatorParameters loadParameters(NodeParametersInterface & params)
{
  EstimatorParameters p;
  Loader l(params);

  l.typed("debug_mode", p.debug_mode, describe("publish xyz_debug_estimate and sonar_ned"));
  l.typed("enable_xy", p.enable_xy, describe("run the horizontal filter's landing reset and updates"));
  l.typed("enable_z", p.enable_z, describe("run the vertical filter's landing reset and updates"));
  l.typed("enable_mocap_xy", p.enable_mocap_xy, describe("subscribe to mocap body-level velocity"));
  l.typed("enable_rgbd", p.enable_rgbd, describe("subscribe to RGB-D velocity (DeltaToVel)"));
  l.typed("enable_mocap_z", p.enable_mocap_z, describe("subscribe to mocap NED pose (z only)"));
  l.typed("enable_sonar", p.enable_sonar, describe("subscribe to the range sensor"));
  l.typed("enable_partial_update", p.enable_partial_update,
    describe("partial (beta-weighted) updates instead of full Kalman updates"));
  l.typed("enable_mocap_switch", p.enable_mocap_switch,
    describe("subscribe to rc_raw and switch to mocap while the channel is > 1500 us; "
    "false in simulation"));
  l.integer("mocap_override_channel", p.mocap_override_channel,
    describe("index into rosflight_msgs/RCRaw.values (receiver channel = index + 1)"),
    0, kRcRawChannels - 1);
  l.typed("enable_measurements", p.enable_measurements,
    describe("runtime switch: apply RGB-D measurements (read on every RGB-D message)", false));

  l.number("mahalanobis_d_sonar", p.mahalanobis_d_sonar,
    describe("gate on squared Mahalanobis distance of range measurements"));
  l.number("mahalanobis_d_rgbd_velocity", p.mahalanobis_d_rgbd_velocity,
    describe("gate on squared Mahalanobis distance of RGB-D velocity"));
  l.number("mahalanobis_d_mocap_z", p.mahalanobis_d_mocap_z,
    describe("gate on squared Mahalanobis distance of mocap z"));
  l.number("mahalanobis_d_mocap_velocity", p.mahalanobis_d_mocap_velocity,
    describe("gate on squared Mahalanobis distance of mocap velocity"));
  l.number("estimator_dt", p.estimator_dt,
    describe("nominal IMU period [s]: initial dt and xy_Q scaling (xy_Q * dt^2)"));

  l.typed("mocap_twist_topic", p.mocap_twist_topic, describe("mocap velocity topic"));
  l.typed("mocap_pose_topic", p.mocap_pose_topic, describe("mocap pose topic"));
  l.typed("rgbd_twist_topic", p.rgbd_twist_topic, describe("RGB-D velocity topic"));

  l.matrix("xy_x0", p.xy_x0);
  l.matrix("xy_P0", p.xy_P0);
  l.matrix("xy_Q", p.xy_Q);
  l.matrix("xy_R0", p.xy_R0);
  l.matrix("xy_beta", p.xy_beta);
  l.matrix("z_x0", p.z_x0);
  l.matrix("z_P0", p.z_P0);
  l.matrix("z_P0_flying", p.z_P0_flying);
  l.matrix("z_Q", p.z_Q);
  l.matrix("z_R0", p.z_R0);
  l.matrix("z_R_flying", p.z_R_flying);
  l.matrix("z_beta", p.z_beta);

  std::vector<std::string> & errors = l.errors();
  if (errors.empty()) {
    // Only meaningful once every value was read.
    const std::vector<std::string> invalid = validate(p);
    errors.insert(errors.end(), invalid.begin(), invalid.end());
  }
  if (!errors.empty()) {
    std::string message = "invalid reef_estimator parameters:";
    for (const std::string & e : errors) {
      message += "\n  " + e;
    }
    throw reef_msgs::ParameterError(message);
  }
  return p;
}

}  // namespace reef_estimator

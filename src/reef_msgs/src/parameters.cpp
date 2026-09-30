#include "reef_msgs/parameters.hpp"

#include <cstdint>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

namespace reef_msgs
{

std::vector<double> getNumberArrayParameter(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const std::string & name, const std::string & description)
{
  if (!params.has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    descriptor.dynamic_typing = true;
    params.declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
  }
  const rclcpp::Parameter p = params.get_parameters({name}).at(0);

  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return p.as_double_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      constexpr std::int64_t kExact = std::int64_t{1} << 53;
      std::vector<double> out;
      for (const std::int64_t v : p.as_integer_array()) {
        if (v > kExact || v < -kExact) {
          throw ParameterError(name + " contains the integer " + std::to_string(v) +
                               ", which is not exactly representable as a double");
        }
        out.push_back(static_cast<double>(v));
      }
      return out;
    }
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      throw ParameterError(name + " is not set; expected a list of numbers (" + description + ")");
    default:
      throw ParameterError(name + " has type " + rclcpp::to_string(p.get_type()) +
                           "; expected a list of numbers (" + description + ")");
  }
}

}  // namespace reef_msgs

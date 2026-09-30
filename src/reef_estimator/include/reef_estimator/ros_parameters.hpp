// Declares and reads the reef_estimator parameters on a ROS 2 node.

#ifndef REEF_ESTIMATOR__ROS_PARAMETERS_HPP_
#define REEF_ESTIMATOR__ROS_PARAMETERS_HPP_

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "reef_estimator/parameters.hpp"
#include "reef_msgs/parameters.hpp"

namespace reef_estimator
{

// Declares every parameter of EstimatorParameters (with a description; all
// read-only except enable_measurements), reads the values, and validates
// them. Double parameters also accept integers (as roscpp did), converted
// exactly; bool, integer, and string parameters must have their exact type.
// Throws reef_msgs::ParameterError listing every problem, one per line.
EstimatorParameters loadParameters(rclcpp::node_interfaces::NodeParametersInterface & params);

}  // namespace reef_estimator

#endif  // REEF_ESTIMATOR__ROS_PARAMETERS_HPP_

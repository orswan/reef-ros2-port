// ROS 2 parameter reading for reef_msgs matrices (replaces the ROS 1
// importMatrixFromParamServer of reef_msgs 7fb63ff, see matrix_operation.h).

#ifndef REEF_MSGS__PARAMETERS_HPP_
#define REEF_MSGS__PARAMETERS_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "reef_msgs/matrix_operation.h"

namespace reef_msgs
{

class ParameterError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Declares `name` (read-only, dynamically typed, no default) if it is not
// declared yet, and returns its value as doubles. Accepts a double array or
// an integer array (converted exactly; |v| <= 2^53). Throws ParameterError if
// the parameter is unset or has any other type.
std::vector<double> getNumberArrayParameter(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const std::string & name, const std::string & description);

// Reads a matrix parameter with the legacy full/diagonal rules. Throws
// ParameterError (naming the parameter and the accepted sizes) if it is
// missing, of the wrong type, empty, non-finite, or of the wrong length. The
// matrix is modified only on success.
template<class Derived>
MatrixLayout importMatrixFromParameter(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  Eigen::MatrixBase<Derived> & mat, const std::string & name)
{
  const std::string accepted = acceptedSizes(mat.rows(), mat.cols());
  const std::vector<double> vec = getNumberArrayParameter(params, name, "matrix: " + accepted);
  const MatrixImport result = importMatrixFromVector(mat, vec, name);
  if (!result.ok) {
    throw ParameterError(result.error);
  }
  return result.layout;
}

}  // namespace reef_msgs

#endif  // REEF_MSGS__PARAMETERS_HPP_

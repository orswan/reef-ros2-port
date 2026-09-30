// Compiled by test_ros_independence.py with only this package's include
// directory and Eigen on the include path, in a clean environment: if any
// numerical helper header pulled in ROS, this would not compile.
#include <cstdio>
#include <vector>

#include "reef_msgs/dynamics.h"
#include "reef_msgs/matrix_operation.h"

int main()
{
  int failures = 0;
  Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Matrix3d C = reef_msgs::quaternion_to_rotation(q);
  if (!C.isApprox(q.toRotationMatrix().transpose(), 1e-15)) {
    std::puts("quaternion_to_rotation is not the transpose of R(q)");
    failures++;
  }
  double r, p, y;
  reef_msgs::roll_pitch_yaw_from_rotation321(C, r, p, y);
  if (std::abs(y - 0.3) > 1e-15 || r != 0.0 || p != 0.0) {
    std::printf("roll_pitch_yaw_from_rotation321: %g %g %g\n", r, p, y);
    failures++;
  }
  Eigen::Matrix2d m;
  if (!reef_msgs::importMatrixFromVector(m, std::vector<double>{0.03, 0.0001}, "z_Q").ok) {
    std::puts("importMatrixFromVector failed");
    failures++;
  }
  if (reef_msgs::importMatrixFromVector(m, std::vector<double>{1, 2, 3}, "z_Q").ok) {
    std::puts("importMatrixFromVector accepted a wrong length");
    failures++;
  }
  std::printf("ros_free_check: %d failures\n", failures);
  return failures ? 1 : 0;
}

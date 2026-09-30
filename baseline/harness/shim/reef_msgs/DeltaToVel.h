#pragma once
// Fields as in reef_msgs 7fb63ff9 msg/DeltaToVel.msg
#include <geometry_msgs/TwistWithCovarianceStamped.h>
namespace reef_msgs {
struct DeltaToVel {
  std_msgs::Header header; boost::array<double, 6> S_upper_bound{}, S_lower_bound{};
  boost::array<double, 3> scaled_std_xyz{}; geometry_msgs::TwistWithCovarianceStamped vel;
};
typedef boost::shared_ptr<const DeltaToVel> DeltaToVelConstPtr;
}

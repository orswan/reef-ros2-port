#pragma once
#include <geometry_msgs/TwistStamped.h>
namespace geometry_msgs {
struct TwistWithCovariance { Twist twist; boost::array<double, 36> covariance{}; };
struct TwistWithCovarianceStamped { std_msgs::Header header; TwistWithCovariance twist; };
typedef boost::shared_ptr<const TwistWithCovarianceStamped> TwistWithCovarianceStampedConstPtr;
}

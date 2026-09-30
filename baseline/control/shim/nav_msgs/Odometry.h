#pragma once
// Fields of nav_msgs/Odometry used by reef_control (pose, twist; covariances omitted).
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
namespace nav_msgs {
struct PoseWithCovariance { geometry_msgs::Pose pose; };
struct TwistWithCovariance { geometry_msgs::Twist twist; };
struct Odometry { std_msgs::Header header; std::string child_frame_id; PoseWithCovariance pose; TwistWithCovariance twist; };
}

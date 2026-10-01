#pragma once
// Adaptation V1 (baseline/README.md): fields of nav_msgs/Odometry that
// rgbd_to_velocity b7637198 uses (header, pose.pose), with the ConstPtr
// typedef of ROS 1. Covariances and twist are not read by the converter.
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
namespace nav_msgs {
struct PoseWithCovariance { geometry_msgs::Pose pose; boost::array<double, 36> covariance{}; };
struct TwistWithCovariance { geometry_msgs::Twist twist; boost::array<double, 36> covariance{}; };
struct Odometry { std_msgs::Header header; std::string child_frame_id; PoseWithCovariance pose; TwistWithCovariance twist; };
typedef boost::shared_ptr<const Odometry> OdometryConstPtr;
}

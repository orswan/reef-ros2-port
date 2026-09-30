#pragma once
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
namespace geometry_msgs {
struct Point { double x = 0, y = 0, z = 0; };
struct Pose { Point position; Quaternion orientation; };
struct PoseStamped { std_msgs::Header header; Pose pose; };
typedef boost::shared_ptr<const PoseStamped> PoseStampedConstPtr;
}

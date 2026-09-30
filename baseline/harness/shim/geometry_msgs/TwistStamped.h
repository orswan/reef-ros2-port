#pragma once
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
namespace geometry_msgs {
struct Twist { Vector3 linear, angular; };
struct TwistStamped { std_msgs::Header header; Twist twist; };
}

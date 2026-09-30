#pragma once
// ROS 1 geometry_msgs/Quaternion default-constructs to all zeros (w = 0).
namespace geometry_msgs { struct Quaternion { double x = 0, y = 0, z = 0, w = 0; }; }

#pragma once
// Fields as in reef_msgs 7fb63ff9 msg/{XYZEstimate,XYEstimate,ZEstimate}.msg
#include <std_msgs/Header.h>
#include <string>
namespace reef_msgs {
struct XYEstimate { double x_dot = 0, y_dot = 0; };
struct ZEstimate { double z = 0, z_dot = 0; };
struct XYZEstimate { std_msgs::Header header; uint32_t node_id = 0; XYEstimate xy_plus; ZEstimate z_plus; };
// A6: defined by the harness (records what the original published).
void record_published(const std::string& topic, const XYZEstimate& m);
}

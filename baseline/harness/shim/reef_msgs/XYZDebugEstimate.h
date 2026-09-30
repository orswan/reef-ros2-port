#pragma once
// Fields as in reef_msgs 7fb63ff9 msg/{XYZDebugEstimate,XYDebugEstimate,ZDebugEstimate}.msg
#include <std_msgs/Header.h>
#include <string>
namespace reef_msgs {
struct XYDebugEstimate {
  double x_dot = 0, y_dot = 0, pitch_bias = 0, roll_bias = 0, xa_bias = 0, ya_bias = 0;
  boost::array<double, 6> sigma_plus{}, sigma_minus{};
};
struct ZDebugEstimate {
  double z = 0, z_dot = 0, bias = 0, u = 0; boost::array<double, 9> P{};
  boost::array<double, 2> truth{}; double z_error = 0, z_dot_error = 0;
  boost::array<double, 3> sigma_plus{}, sigma_minus{};
};
struct XYZDebugEstimate {
  std_msgs::Header header; uint32_t node_id = 0;
  XYDebugEstimate xy_minus, xy_plus; ZDebugEstimate z_minus, z_plus;
};
// A6: defined by the harness (records what the original published).
void record_published(const std::string& topic, const XYZDebugEstimate& m);
}

#pragma once
// Fields as in reef_msgs 7fb63ff9 msg/DesiredState.msg and msg/DesiredVector.msg.
#include <std_msgs/Header.h>
#include <string>
namespace reef_msgs {
struct DesiredVector { double x = 0, y = 0, z = 0, yaw = 0; };
struct DesiredState {
  std_msgs::Header header; uint32_t node_id = 0;
  DesiredVector pose, velocity, acceleration, attitude;
  bool attitude_valid = false, position_valid = false, velocity_valid = false, acceleration_valid = false, altitude_only = false;
};
void record_published(const std::string& topic, const DesiredState& m);  // defined by the harness
}

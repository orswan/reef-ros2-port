#pragma once
// Fields as in rosflight_msgs (rosflight 44e5f37e) msg/Command.msg.
#include <std_msgs/Header.h>
#include <string>
namespace rosflight_msgs {
struct Command {
  static const uint8_t MODE_PASS_THROUGH = 0, MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1, MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2,
      MODE_ROLL_PITCH_YAWRATE_ALTITUDE = 3, MODE_XPOS_YPOS_YAW_ALTITUDE = 4, MODE_XVEL_YVEL_YAWRATE_ALTITUDE = 5, MODE_XACC_YACC_YAWRATE_AZ = 6;
  static const uint8_t IGNORE_NONE = 0, IGNORE_X = 1, IGNORE_Y = 2, IGNORE_Z = 4, IGNORE_F = 8;
  std_msgs::Header header; uint8_t mode = 0; uint8_t ignore = 0; float x = 0, y = 0, z = 0, F = 0;
};
void record_published(const std::string& topic, const Command& m);  // defined by the harness
}

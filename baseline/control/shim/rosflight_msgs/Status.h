#pragma once
// Fields as in rosflight_msgs (rosflight 44e5f37e) msg/Status.msg.
#include <std_msgs/Header.h>
namespace rosflight_msgs {
struct Status { std_msgs::Header header; bool armed = false, failsafe = false, rc_override = false, offboard = false;
                int8_t control_mode = 0, error_code = 0; int16_t num_errors = 0, loop_time_us = 0; };
}

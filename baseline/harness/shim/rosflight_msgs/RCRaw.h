#pragma once
// Fields as in rosflight_msgs (rosflight 44e5f37e) msg/RCRaw.msg: header, uint16[8] values
#include <std_msgs/Header.h>
namespace rosflight_msgs {
struct RCRaw { std_msgs::Header header; boost::array<uint16_t, 8> values{}; };
typedef boost::shared_ptr<const RCRaw> RCRawConstPtr;
}

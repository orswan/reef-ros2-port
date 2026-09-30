#pragma once
#include <std_msgs/Header.h>
namespace sensor_msgs {
struct Range {
  std_msgs::Header header; uint8_t radiation_type = 0;
  float field_of_view = 0, min_range = 0, max_range = 0, range = 0;   // float32 in the ROS 1 message
};
typedef boost::shared_ptr<const Range> RangeConstPtr;
}

// The pinned upstream rosflight_msgs/RCRaw matches what the estimator's RC
// switch assumes (legacy rosflight 44e5f37e: Header header, uint16[8] values).
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <type_traits>

#include <rosflight_msgs/msg/rc_raw.hpp>

#include "reef_estimator/parameters.hpp"

using RCRaw = rosflight_msgs::msg::RCRaw;

static_assert(std::is_same_v<decltype(RCRaw::values), std::array<std::uint16_t, 8>>);
static_assert(std::tuple_size_v<decltype(RCRaw::values)> == reef_estimator::kRcRawChannels);
static_assert(std::is_same_v<decltype(RCRaw::header), std_msgs::msg::Header>);

TEST(RcRaw, DefaultIsAllZeroWhichNeverSelectsMocap)
{
  // A default-constructed message reads as "switch low" (0 <= 1500).
  const RCRaw msg;
  for (auto v : msg.values) {
    EXPECT_EQ(v, 0u);
    EXPECT_FALSE(v > reef_estimator::kRcSwitchThresholdUs);
  }
}

TEST(RcRaw, SwitchThresholdIsTheLegacyValue)
{
  // sensor_manager.cpp: values[channel] > 1500 selects mocap, <= 1500 reverts.
  // Note for hardware integration: MAVLink marks an unused channel as
  // UINT16_MAX, which this rule reads as "high".
  EXPECT_EQ(reef_estimator::kRcSwitchThresholdUs, 1500);
}

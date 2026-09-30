#include "reef_control/firmware_mux.hpp"

namespace reef_control
{

void FirmwareMux::command(int64_t now_ms, uint8_t mode, uint16_t ignore, const std::array<float, 4> & xyzF)
{
  for (int i = 0; i < 4; ++i) {
    ch_[i].value = xyzF[i];
    ch_[i].active = !(ignore & (1u << i));
  }
  mode_ = mode;
  stamp_ms_ = now_ms;
  have_command_ = true;
}

bool FirmwareMux::tick(int64_t now_ms)
{
  if (!have_command_ || now_ms <= stamp_ms_ + timeout_ms_) {
    return false;
  }
  bool any = false;
  for (auto & c : ch_) {
    any = any || c.active;
    c.active = false;
  }
  if (any) {
    ++timeouts_;
  }
  return any;
}

std::string FirmwareMux::motors() const
{
  if (!armed_) {
    return "off (disarmed)";
  }
  if (!modeSupported()) {
    return "unsupported mode";
  }
  return ch_[3].active ? "offboard throttle" : "rc throttle";
}

}  // namespace reef_control

// Dry-run interpretation of offboard commands as the pinned ROSflight
// firmware (b77c3854) muxes them: comm_manager.cpp offboard_control_callback
// and command_manager.cpp run(). DEVELOPMENT/TRACE TOOL: it decides what the
// firmware would use for each channel; it drives nothing.
//   - a command marks each channel active unless its ignore bit is set
//     (x 1, y 2, z 4, F 8) and stamps the receipt time;
//   - once now > stamp + offboard_timeout, every channel is inactive;
//   - inactive channels fall back to RC (none in a dry run: "rc");
//   - disarmed: the mixer outputs nothing (motors off).
// Not modelled: RC stick deviation or switch override, failsafe, min
// throttle, equilibrium torques. No ROS types.
#ifndef REEF_CONTROL_FIRMWARE_MUX_HPP
#define REEF_CONTROL_FIRMWARE_MUX_HPP

#include <array>
#include <cstdint>
#include <string>

namespace reef_control
{

struct MuxChannel
{
  bool active = false;
  float value = 0;
};

class FirmwareMux
{
public:
  explicit FirmwareMux(int64_t offboard_timeout_ms = 100) : timeout_ms_(offboard_timeout_ms) {}

  // A command received at now_ms (mode as rosflight_msgs/Command; values x, y, z, F).
  void command(int64_t now_ms, uint8_t mode, uint16_t ignore, const std::array<float, 4> & xyzF);
  void setArmed(bool armed) {armed_ = armed;}
  // Applies the offboard timeout; returns true if the channels just went inactive.
  bool tick(int64_t now_ms);

  const std::array<MuxChannel, 4> & channels() const {return ch_;}
  bool armed() const {return armed_;}
  uint8_t mode() const {return mode_;}
  bool modeSupported() const {return mode_ == 2;}   // ROLL_PITCH_YAWRATE_THROTTLE, the only mode reef_control sends
  long timeouts() const {return timeouts_;}
  // "offboard" or "rc" per channel, and the motor state.
  std::string source(int i) const {return ch_[i].active ? "offboard" : "rc";}
  std::string motors() const;

private:
  int64_t timeout_ms_;
  int64_t stamp_ms_ = 0;
  bool have_command_ = false;
  bool armed_ = false;
  uint8_t mode_ = 0;
  long timeouts_ = 0;
  std::array<MuxChannel, 4> ch_{};
};

}  // namespace reef_control

#endif

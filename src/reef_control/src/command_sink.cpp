// reef_control_sink: DRY-RUN command sink (P06). Records every command of
// reef_control with the ROSflight firmware's interpretation (firmware_mux.hpp)
// to a CSV trace. It has no hardware output of any kind; the `hardware`
// parameter exists only to refuse (NOT IMPLEMENTED, P10).
//
// Parameters: trace_file (CSV path; empty = log only), offboard_timeout_ms
// (100, firmware OFFBOARD_TIMEOUT default), hardware (false; true -> exit 2).
// Topics in: command (rosflight_msgs/Command), status (rosflight_msgs/Status).
// Exit status: 0 after SIGINT/shutdown, 2 hardware requested, 1 trace file error.
#include <array>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>

#include "reef_control/firmware_mux.hpp"

namespace
{

struct Refused {int code; std::string what;};

class CommandSink : public rclcpp::Node
{
public:
  CommandSink()
  : rclcpp::Node("reef_control_sink")
  {
    if (declare_parameter("hardware", false)) {
      throw Refused{2, "hardware output is NOT IMPLEMENTED (P10): this sink is dry-run only"};
    }
    const auto timeout = declare_parameter<int64_t>("offboard_timeout_ms", 100);
    const auto path = declare_parameter<std::string>("trace_file", "");
    mux_ = reef_control::FirmwareMux(timeout);
    if (!path.empty()) {
      trace_.open(path);
      if (!trace_) {throw Refused{1, "cannot write trace_file " + path};}
      trace_ << "event,t_node_ns,stamp_sec,stamp_nsec,mode,ignore,x,y,z,F,armed,src_x,src_y,src_z,src_F,motors\n";
    }
    RCLCPP_INFO(get_logger(), "DRY RUN: commands are recorded, never sent to hardware");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    command_sub_ = create_subscription<rosflight_msgs::msg::Command>("command", qos,
        [this](const rosflight_msgs::msg::Command & m) {
          const int64_t now = now_ns();
          mux_.command(now / 1000000, m.mode, m.ignore, {m.u[0], m.u[1], m.u[2], m.u[3]});
          write("command", now, &m);
        });
    status_sub_ = create_subscription<rosflight_msgs::msg::Status>("status", qos,
        [this](const rosflight_msgs::msg::Status & m) {
          if (m.armed != mux_.armed()) {
            mux_.setArmed(m.armed);
            write(m.armed ? "armed" : "disarmed", now_ns(), nullptr);
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {
          const int64_t now = now_ns();
          if (mux_.tick(now / 1000000)) {
            write("offboard_timeout", now, nullptr);
            RCLCPP_WARN(get_logger(), "offboard timeout: all channels fall back to RC");
          }
        });
  }

private:
  int64_t now_ns() {return get_clock()->now().nanoseconds();}

  void write(const char * event, int64_t now, const rosflight_msgs::msg::Command * m)
  {
    if (!trace_) {return;}
    const auto & c = mux_.channels();
    char buf[512];
    std::snprintf(buf, sizeof buf, "%s,%lld,%d,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%d,%s,%s,%s,%s,%s\n", event,
      static_cast<long long>(now), m ? m->header.stamp.sec : 0, m ? m->header.stamp.nanosec : 0u,
      static_cast<unsigned>(mux_.mode()), m ? static_cast<unsigned>(m->ignore) : 0u,
      c[0].value, c[1].value, c[2].value, c[3].value, mux_.armed() ? 1 : 0,
      mux_.source(0).c_str(), mux_.source(1).c_str(), mux_.source(2).c_str(), mux_.source(3).c_str(),
      mux_.motors().c_str());
    trace_ << buf;
    trace_.flush();
  }

  reef_control::FirmwareMux mux_;
  std::ofstream trace_;
  rclcpp::Subscription<rosflight_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<CommandSink>());
  } catch (const Refused & r) {
    RCLCPP_FATAL(rclcpp::get_logger("reef_control_sink"), "%s", r.what.c_str());
    status = r.code;
  }
  rclcpp::shutdown();
  return status;
}

#include <rclcpp/rclcpp.hpp>
#include "reef_control/control_node.hpp"

// ROS 2 port (P06) of reef_control 12237b76 pid_node.cpp. Single-threaded
// executor: callbacks (including parameter changes) never overlap.
// Exit status: 0 after SIGINT/shutdown, 1 for invalid parameters.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto pid_object = std::make_shared<reef_control::ControlNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pid_object);
    executor.spin();
  } catch (const reef_control::ParameterError& e) {
    RCLCPP_FATAL(rclcpp::get_logger("reef_control"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

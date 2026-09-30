#include <rclcpp/rclcpp.hpp>

#include "reef_estimator/sensor_manager.h"
#include "reef_msgs/parameters.hpp"

// Single-threaded executor: callbacks never overlap, and all estimator state
// changes happen on this thread in delivery order (INTERFACES.md section 3.8).
// Exit status: 0 after SIGINT/shutdown, 1 for invalid parameters.
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        auto estimator_object = std::make_shared<reef_estimator::SensorManager>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(estimator_object);
        executor.spin();
    } catch (const reef_msgs::ParameterError& e) {
        RCLCPP_FATAL(rclcpp::get_logger("reef_estimator"), "%s", e.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}

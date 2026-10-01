//
// Created by humberto on 5/20/19.
//
// ROS 2 port (P08): single-threaded executor; exit 1 for invalid parameters.
#include <rclcpp/rclcpp.hpp>
#include "rgbd_to_velocity/rgbd_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(std::make_shared<rgbd_to_velocity::RgbdNode>());
    } catch (const rgbd_to_velocity::ParameterError& e) {
        RCLCPP_FATAL(rclcpp::get_logger("rgbd_to_velocity"), "%s", e.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}

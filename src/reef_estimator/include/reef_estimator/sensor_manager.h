/**
 * \class SensorManager
 *
 * \ingroup reef_estimator
 *
 * This class is meant to have all the ROS callbacks and recieve all the sensor readings
 * This class also publishes the sonar reading in the NED frame to verify it's measurements
 *
 * \author $Author: bv Humberto Ramos, William Warke, Prashant Ganesh
 *
 * \version $Revision: 1.0 $
 *
 * \date $Date: 2019/03/05 $
 *
 * Contact: prashant.ganesh@ufl.edu
 *
 * ROS 2 port (P04): an rclcpp node around the ROS-free VerticalEstimator.
 * Interface contract: docs/INTERFACES.md section 3. The horizontal inputs
 * are not subscribed until the horizontal filter is ported.
 */


#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <memory>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reef_msgs/msg/xyz_debug_estimate.hpp>
#include <reef_msgs/msg/xyz_estimate.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "reef_estimator/parameters.hpp"
#include "reef_estimator/vertical_estimator.h"

namespace reef_estimator {
    class SensorManager : public rclcpp::Node {
    public:
        // Throws reef_msgs::ParameterError for invalid parameters.
        explicit SensorManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        // Message callbacks (public so the wrapper tests can drive them
        // without transport; the executor calls them otherwise).
        void imuCallback(const sensor_msgs::msg::Imu& msg);
        void altimeterCallback(const sensor_msgs::msg::Range& msg);
        void rcRawCallback(const rosflight_msgs::msg::RCRaw& msg);
        void mocapPoseCallback(const geometry_msgs::msg::PoseStamped& msg);

        // Returns the estimator to its startup state (reset service, backward
        // ROS time jump).
        void reset(const std::string& reason);

        // Introspection for tests.
        const VerticalEstimator& core() const { return *xyzEst; }
        const std::optional<reef_msgs::msg::XYZEstimate>& lastEstimate() const { return last_estimate_; }
        const std::optional<reef_msgs::msg::XYZDebugEstimate>& lastDebugEstimate() const { return last_debug_; }
        long publishedCount() const { return published_; }
        long stampAnomalies() const { return stamp_anomalies_; }

    private:
        EstimatorParameters params_;
        std::unique_ptr<VerticalEstimator> xyzEst;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altimeter_subscriber_;
        rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_subscriber_;

        rclcpp::Publisher<reef_msgs::msg::XYZEstimate>::SharedPtr state_publisher_;
        rclcpp::Publisher<reef_msgs::msg::XYZDebugEstimate>::SharedPtr debug_state_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_flying_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_ned_publisher_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
        rclcpp::JumpHandler::SharedPtr jump_handler_;

        std::optional<reef_msgs::msg::XYZEstimate> last_estimate_;
        std::optional<reef_msgs::msg::XYZDebugEstimate> last_debug_;
        long published_ = 0;
        long stamp_anomalies_ = 0;
        std::optional<double> last_imu_stamp_;

        void makeEstimator();
        void publishFlying(bool flying);
    };
}
#endif

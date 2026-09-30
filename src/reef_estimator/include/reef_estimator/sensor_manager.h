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
 * ROS 2 port (P04/P05): an rclcpp node around the ROS-free XYZEstimator.
 * Interface contract: docs/INTERFACES.md section 3.
 */


#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <reef_msgs/msg/delta_to_vel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reef_msgs/msg/xyz_debug_estimate.hpp>
#include <reef_msgs/msg/xyz_estimate.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "reef_estimator/parameters.hpp"
#include "reef_estimator/xyz_estimator.h"

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
        void mocapTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped& msg);
        void rgbdTwistCallback(const reef_msgs::msg::DeltaToVel& msg);

        // Returns the estimator to its startup state (reset service; a backward
        // ROS time jump flags a reset that the next message callback applies).
        void reset(const std::string& reason);

        // Introspection for tests.
        const XYZEstimator& core() const { return *xyzEst; }
        const std::optional<reef_msgs::msg::XYZEstimate>& lastEstimate() const { return last_estimate_; }
        const std::optional<reef_msgs::msg::XYZDebugEstimate>& lastDebugEstimate() const { return last_debug_; }
        long publishedCount() const { return published_; }
        long stampAnomalies() const { return stamp_anomalies_; }
        bool resetPending() const { return jump_reset_pending_; }

    private:
        EstimatorParameters params_;
        std::unique_ptr<XYZEstimator> xyzEst;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altimeter_subscriber_;
        rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr mocap_twist_subscriber_;
        rclcpp::Subscription<reef_msgs::msg::DeltaToVel>::SharedPtr rgbd_twist_subscriber_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

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
        std::atomic<bool> jump_reset_pending_{false};

        // Callback wall-time statistics (diagnostics every 250 IMU callbacks).
        std::vector<double> callback_us_;
        long callbacks_total_ = 0;
        long callbacks_over_2ms_ = 0;
        double callback_us_max_ = 0;
        long imu_callbacks_since_diag_ = 0;
        long rgbd_ignored_ = 0;

        void makeEstimator();
        void recordCallback(std::chrono::steady_clock::time_point t0);
        void publishDiagnostics();
        void applyPendingReset();
        void publishFlying(bool flying);
    };
}
#endif

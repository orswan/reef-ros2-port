#include "reef_estimator/sensor_manager.h"

#include <chrono>
#include <functional>

#include "reef_estimator/ros_conversions.hpp"
#include "reef_estimator/ros_parameters.hpp"

namespace reef_estimator
{
    namespace
    {
        // Input QoS (INTERFACES.md section 3.4): best effort matches publishers
        // of either reliability; depths are the ROS 1 queue sizes.
        rclcpp::QoS inputQoS(size_t depth) { return rclcpp::QoS(depth).best_effort().durability_volatile(); }
        // Formerly latched outputs.
        rclcpp::QoS latchedQoS() { return rclcpp::QoS(1).reliable().transient_local(); }

        rclcpp::SubscriptionOptions overridable()
        {
            rclcpp::SubscriptionOptions o;
            o.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
            return o;
        }
    }

    SensorManager::SensorManager(const rclcpp::NodeOptions& options) : rclcpp::Node("reef_estimator", options)
    {
        params_ = loadParameters(*get_node_parameters_interface());
        for (const auto& w : warnings(params_))
            RCLCPP_WARN(get_logger(), "%s", w.c_str());
        makeEstimator();

        //Mocap override RC channel parameter
        if (params_.enable_mocap_switch) {
            RCLCPP_WARN(get_logger(), "Mocap override RC switch enabled on channel %d", params_.mocap_override_channel);
            rc_subscriber_ = create_subscription<rosflight_msgs::msg::RCRaw>("rc_raw", inputQoS(1),
                [this](const rosflight_msgs::msg::RCRaw& m) { rcRawCallback(m); }, overridable());
        }

        if (params_.enable_mocap_xy || params_.enable_rgbd)
        {
            RCLCPP_WARN(get_logger(), "Horizontal filter not ported yet (P04): '%s' and '%s' are not subscribed; "
                        "horizontal output fields are NaN", params_.mocap_twist_topic.c_str(),
                        params_.rgbd_twist_topic.c_str());
        }

        if (params_.enable_mocap_z)
        {
            mocap_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                params_.mocap_pose_topic, inputQoS(1),
                [this](const geometry_msgs::msg::PoseStamped& m) { mocapPoseCallback(m); }, overridable());
        }

        if (params_.enable_sonar)
        {
            altimeter_subscriber_ = create_subscription<sensor_msgs::msg::Range>("sonar", inputQoS(1),
                [this](const sensor_msgs::msg::Range& m) { altimeterCallback(m); }, overridable());
            if (params_.debug_mode)
                range_ned_publisher_ = create_publisher<sensor_msgs::msg::Range>("sonar_ned", rclcpp::QoS(1).reliable());
        }

        state_publisher_ = create_publisher<reef_msgs::msg::XYZEstimate>("xyz_estimate", latchedQoS());
        if (params_.debug_mode) {
            RCLCPP_WARN(get_logger(), "Debug Mode Enabled");
            debug_state_publisher_ = create_publisher<reef_msgs::msg::XYZDebugEstimate>("xyz_debug_estimate", latchedQoS());
        }
        is_flying_publisher_ = create_publisher<std_msgs::msg::Bool>("is_flying_reef", latchedQoS());

        reset_service_ = create_service<std_srvs::srv::Trigger>("~/reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                reset("reset service");
                res->success = true;
                res->message = "estimator returned to its startup state";
            });

        // A backward jump of ROS time (simulation reset, replay restarted)
        // would otherwise produce a negative dt for the next IMU message. The
        // time source may call this from its own thread, so it only flags the
        // reset; the executor thread applies it before the next message.
        rcl_jump_threshold_t threshold;
        threshold.on_clock_change = false;
        threshold.min_forward.nanoseconds = 0;     // 0 disables forward-jump callbacks
        threshold.min_backward.nanoseconds = -1;
        jump_handler_ = get_clock()->create_jump_callback(nullptr,
            [this](const rcl_time_jump_t& jump) {
                if (jump.delta.nanoseconds < 0) jump_reset_pending_ = true;
            }, threshold);

        //Initialize subscribers with corresponding callbacks.
        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", inputQoS(10),
            [this](const sensor_msgs::msg::Imu& m) { imuCallback(m); }, overridable());
    }

    void SensorManager::makeEstimator()
    {
        xyzEst = std::make_unique<VerticalEstimator>(params_);
        xyzEst->log = [this](LogLevel level, const std::string& text) {
            switch (level) {
                case LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", text.c_str()); break;
                case LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", text.c_str()); break;
                default:
                    // Rejections can arrive at the sensor rate.
                    if (text.find("rejected") != std::string::npos)
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", text.c_str());
                    else
                        RCLCPP_INFO(get_logger(), "%s", text.c_str());
            }
        };
    }

    void SensorManager::publishFlying(bool flying)
    {
        std_msgs::msg::Bool m;
        m.data = flying;
        is_flying_publisher_->publish(m);
    }

    void SensorManager::reset(const std::string& reason)
    {
        const bool was_flying = xyzEst->isFlying();
        makeEstimator();
        last_imu_stamp_.reset();
        RCLCPP_WARN(get_logger(), "Estimator reset (%s)", reason.c_str());
        if (was_flying)
            publishFlying(false);   // the fresh estimator starts on the ground
    }

    void SensorManager::applyPendingReset()
    {
        if (jump_reset_pending_.exchange(false))
            reset("ROS time jumped backwards");
    }

    void SensorManager::imuCallback(const sensor_msgs::msg::Imu& msg)
    {
        applyPendingReset();
        //Pass the imu message to estimator.
        const ImuSample s = fromMsg(msg);
        // Diagnostics only: the estimator processes anomalous stamps exactly
        // as the original did (BASELINE_DECISION.md D9, fixtures v05-v07).
        const double t = s.stamp.toSec();
        if (last_imu_stamp_) {
            const double dt = t - *last_imu_stamp_;
            if (dt <= 0 || dt > 10 * params_.estimator_dt) {
                stamp_anomalies_++;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "IMU stamp step %.6f s (nominal %.6f s); processed as in the original", dt, params_.estimator_dt);
            }
        }
        last_imu_stamp_ = t;

        const long transitions = xyzEst->takeoffTransitions();
        if (!xyzEst->sensorUpdate(s))
            return;
        last_estimate_ = toEstimateMsg(*xyzEst);
        state_publisher_->publish(*last_estimate_);
        published_++;
        if (params_.debug_mode) {
            last_debug_ = toDebugMsg(*xyzEst);
            debug_state_publisher_->publish(*last_debug_);
        }
        if (xyzEst->takeoffTransitions() != transitions)
            publishFlying(xyzEst->isFlying());
    }

    void SensorManager::rcRawCallback(const rosflight_msgs::msg::RCRaw& msg) {
        applyPendingReset();
        xyzEst->rcRawUpdate(fromMsg(msg));
    }

    void SensorManager::mocapPoseCallback(const geometry_msgs::msg::PoseStamped& msg)
    {
        applyPendingReset();
        xyzEst->mocapUpdate(fromMsg(msg));
    }

void SensorManager::altimeterCallback(const sensor_msgs::msg::Range& msg)
{
    applyPendingReset();
    xyzEst->sensorUpdate(fromMsg(msg));

    if (params_.debug_mode)
    {
        //Publish the negative range measurement
        sensor_msgs::msg::Range range_msg_ned = msg;
        range_msg_ned.range = -range_msg_ned.range;
        range_ned_publisher_->publish(range_msg_ned);
    }
}



}

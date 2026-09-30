// X3 -> REEF IMU adapter (C++; latency-critical path).
//
//   /x3/imu (FLU specific force, no orientation) + /x3/truth/odom (TRUTH)
//     -> /x3/reef/imu/data  sensor_msgs/Imu, body FRD, frame x3/base_link_frd,
//        orientation = TRUTH attitude of FRD in NED (IDEALIZED; see attitude.hpp)
//
// Each IMU message is converted and published immediately. IMU messages
// before the second truth sample are dropped. The Python reef_adapter
// publishes the simulated velocity observations and the input labels.
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "reef_x3_adapter/attitude.hpp"

namespace
{
int64_t ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}
}  // namespace

class X3ImuAdapter : public rclcpp::Node
{
public:
  X3ImuAdapter()
  : rclcpp::Node("x3_imu_adapter")
  {
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/x3/reef/imu/data", 50);
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("/x3/truth/odom", 50,
        [this](const nav_msgs::msg::Odometry & m) {
          const auto & o = m.pose.pose.orientation;
          truth_.add(ns(m.header.stamp), {o.w, o.x, o.y, o.z});
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/x3/imu", 50,
        [this](const sensor_msgs::msg::Imu & m) {onImu(m);});
    RCLCPP_WARN(get_logger(), "IDEALIZED: /x3/reef/imu/data orientation is the TRUTH attitude (/x3/truth/odom)");
  }

private:
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const auto q = truth_.at(ns(m.header.stamp));
    if (!q) {
      dropped_++;
      return;
    }
    sensor_msgs::msg::Imu out;
    out.header.stamp = m.header.stamp;
    out.header.frame_id = "x3/base_link_frd";
    out.linear_acceleration.x = m.linear_acceleration.x;       // FLU -> FRD: (x, -y, -z)
    out.linear_acceleration.y = -m.linear_acceleration.y;
    out.linear_acceleration.z = -m.linear_acceleration.z;
    out.angular_velocity.x = m.angular_velocity.x;
    out.angular_velocity.y = -m.angular_velocity.y;
    out.angular_velocity.z = -m.angular_velocity.z;
    out.linear_acceleration_covariance = m.linear_acceleration_covariance;
    out.angular_velocity_covariance = m.angular_velocity_covariance;
    out.orientation.w = (*q)[0];
    out.orientation.x = (*q)[1];
    out.orientation.y = (*q)[2];
    out.orientation.z = (*q)[3];
    out.orientation_covariance[0] = 0.0;   // idealized: truth
    pub_->publish(out);
  }

  reef_x3_adapter::TruthBuffer truth_;
  long dropped_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<X3ImuAdapter>());
  rclcpp::shutdown();
  return 0;
}

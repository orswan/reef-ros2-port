// reef_fc_standin: SIMPLIFIED LOW-LEVEL STAND-IN (DEVELOPMENT TOOL) between
// reef_control and Gazebo's MulticopterMotorModel (docs/CONTROL_CHAIN.md
// section 7). Not ROSflight; results are not ROSflight, hardware, or flight
// evidence.
//
// In:  command      rosflight_msgs/Command (reef_control, mode 2)
//      arm          std_msgs/Bool (the scenario arms and disarms)
//      gyro         sensor_msgs/Imu, Gazebo's NOISE-FREE IMU (FLU): body rates (TRUTH)
//      attitude     nav_msgs/Odometry, simulation TRUTH pose (ENU/FLU)
// Out: motor_speed  actuator_msgs/Actuators, velocity[4] in rad/s (bridged to
//                   the motor model's gz.msgs.Actuators command topic)
//      status       rosflight_msgs/Status (armed), 10 Hz and on change
//      debug        std_msgs/Float64MultiArray, one per step (layout in the label)
//      label        std_msgs/String, transient local
// One step per gyro sample once an attitude has arrived. Sim time throughout.
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <actuator_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "reef_fc_standin/standin.hpp"

namespace
{

const char * kDebugLayout =
  "t,armed,active,F,roll_c,pitch_c,yaw_rate_c,roll,pitch,yaw,p,q,r,T,tau_x,tau_y,tau_z,"
  "w0,w1,w2,w3,saturated,cmd_age,offboard_timeouts";

using Quat = std::array<double, 4>;   // w, x, y, z
Quat qmul(const Quat & a, const Quat & b)
{
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

class StandInNode : public rclcpp::Node
{
public:
  StandInNode()
  : rclcpp::Node("reef_fc_standin")
  {
    reef_fc_standin::Params p;
    const auto rx = declare_parameter<std::vector<double>>("rotor_x");
    const auto ry = declare_parameter<std::vector<double>>("rotor_y");
    const auto rd = declare_parameter<std::vector<int64_t>>("rotor_dir");
    if (rx.size() != 4 || ry.size() != 4 || rd.size() != 4) {
      throw std::runtime_error("rotor_x, rotor_y, rotor_dir need 4 values each");
    }
    for (int i = 0; i < 4; ++i) {
      p.rotors[i] = {rx[i], ry[i], static_cast<int>(rd[i])};
      if (rd[i] != 1 && rd[i] != -1) {throw std::runtime_error("rotor_dir values must be +1 or -1");}
    }
    p.motor_constant = declare_parameter<double>("motor_constant");
    p.moment_constant = declare_parameter<double>("moment_constant");
    p.max_rot_velocity = declare_parameter<double>("max_rot_velocity");
    p.ixx = declare_parameter<double>("ixx");
    p.iyy = declare_parameter<double>("iyy");
    p.izz = declare_parameter<double>("izz");
    p.kp_angle = declare_parameter<double>("kp_angle");
    p.kd_rate = declare_parameter<double>("kd_rate");
    p.kp_yaw_rate = declare_parameter<double>("kp_yaw_rate");
    p.offboard_timeout_ms = declare_parameter<int64_t>("offboard_timeout_ms", 100);
    for (double v : {p.motor_constant, p.moment_constant, p.max_rot_velocity, p.ixx, p.iyy, p.izz, p.kp_angle,
        p.kd_rate, p.kp_yaw_rate})
    {
      if (!std::isfinite(v) || v <= 0) {throw std::runtime_error("stand-in parameters must be finite and > 0");}
    }
    core_ = std::make_unique<reef_fc_standin::StandIn>(p);

    const auto rel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    motor_pub_ = create_publisher<actuator_msgs::msg::Actuators>("motor_speed", rel);
    status_pub_ = create_publisher<rosflight_msgs::msg::Status>("status", rel);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug", rclcpp::QoS(100).reliable());
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    label_pub_ = create_publisher<std_msgs::msg::String>("label", latched);
    std_msgs::msg::String label;
    label.data = std::string("STAND-IN LOW-LEVEL LOOP (DEVELOPMENT TOOL), not ROSflight: firmware command mux, "
      "angle-P/rate-D attitude loop on TRUTH attitude and noise-free gyro, linear thrust, geometric allocation "
      "to the X3 motor model. debug layout: ") + kDebugLayout;
    label_pub_->publish(label);

    command_sub_ = create_subscription<rosflight_msgs::msg::Command>("command", rel,
        [this](const rosflight_msgs::msg::Command & m) {
          core_->command(now_ms(), m.mode, m.ignore, {m.u[0], m.u[1], m.u[2], m.u[3]});
          cmd_stamp_ = rclcpp::Time(m.header.stamp, RCL_ROS_TIME);
          have_cmd_ = true;
        });
    arm_sub_ = create_subscription<std_msgs::msg::Bool>("arm", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool & m) {
          if (m.data != core_->armed()) {
            core_->setArmed(m.data);
            RCLCPP_INFO(get_logger(), "%s", m.data ? "ARMED" : "DISARMED");
            publishStatus();
          }
        });
    attitude_sub_ = create_subscription<nav_msgs::msg::Odometry>("attitude", rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry & m) {
          const auto & o = m.pose.pose.orientation;
          // q_NED<-FRD = q_T * q_ENU<-FLU * q_B (as reef_sim's reef_adapter)
          static const double s = std::sqrt(0.5);
          const Quat q = qmul(qmul({0.0, s, s, 0.0}, {o.w, o.x, o.y, o.z}), {0.0, 1.0, 0.0, 0.0});
          reef_fc_standin::eulerFromQuaternion(q[0], q[1], q[2], q[3], state_.roll, state_.pitch, state_.yaw);
          have_attitude_ = true;
        });
    gyro_sub_ = create_subscription<sensor_msgs::msg::Imu>("gyro", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::Imu & m) {
          state_.p = m.angular_velocity.x;      // FLU -> FRD
          state_.q = -m.angular_velocity.y;
          state_.r = -m.angular_velocity.z;
          if (have_attitude_) {step();}
        });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {publishStatus();});
    RCLCPP_WARN(get_logger(), "STAND-IN low-level loop (development tool, not ROSflight); attitude from TRUTH");
  }

private:
  int64_t now_ms() {return get_clock()->now().nanoseconds() / 1000000;}

  void publishStatus()
  {
    rosflight_msgs::msg::Status s;
    s.header.stamp = get_clock()->now();
    s.armed = core_->armed();
    s.offboard = true;
    status_pub_->publish(s);
  }

  void step()
  {
    const rclcpp::Time now = get_clock()->now();
    const auto o = core_->step(now.nanoseconds() / 1000000, state_);
    actuator_msgs::msg::Actuators a;
    a.header.stamp = now;
    a.velocity.assign(o.omega.begin(), o.omega.end());
    motor_pub_->publish(a);
    std_msgs::msg::Float64MultiArray d;
    const double age = have_cmd_ ? (now - cmd_stamp_).seconds() : std::nan("");
    d.data = {now.seconds(), double(o.armed), double(o.active), o.F, o.roll_c, o.pitch_c, o.yaw_rate_c,
      state_.roll, state_.pitch, state_.yaw, state_.p, state_.q, state_.r, o.T, o.tau_x, o.tau_y, o.tau_z,
      o.omega[0], o.omega[1], o.omega[2], o.omega[3], double(o.saturated), age, double(core_->offboardTimeouts())};
    debug_pub_->publish(d);
  }

  std::unique_ptr<reef_fc_standin::StandIn> core_;
  reef_fc_standin::State state_;
  bool have_attitude_ = false, have_cmd_ = false;
  rclcpp::Time cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::Publisher<rosflight_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr label_pub_;
  rclcpp::Subscription<rosflight_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr attitude_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<StandInNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("reef_fc_standin"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

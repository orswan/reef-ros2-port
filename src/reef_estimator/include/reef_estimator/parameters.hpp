// reef_estimator parameter contract (P03), independent of ROS.
//
// Names, types, and defaults are those of reef_estimator master e4179f48
// (xyz_estimator.cpp, sensor_manager.cpp); docs/INTERFACES.md section 3.4
// describes each one. The matrices have no legacy default that makes sense
// (a missing matrix was zero-filled), so they start as NaN and must be
// loaded; validate() rejects anything left unloaded.

#ifndef REEF_ESTIMATOR__PARAMETERS_HPP_
#define REEF_ESTIMATOR__PARAMETERS_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace reef_estimator
{

// rosflight_msgs/RCRaw: uint16 values[8], PWM pulse width in microseconds.
// values[i] is receiver channel i + 1.
constexpr int kRcRawChannels = 8;
// Legacy switch rule: values[mocap_override_channel] > 1500 selects mocap.
constexpr std::uint16_t kRcSwitchThresholdUs = 1500;

struct EstimatorParameters
{
  template<int R, int C>
  using Mat = Eigen::Matrix<double, R, C>;
  template<int R, int C>
  static Mat<R, C> unset() {return Mat<R, C>::Constant(std::numeric_limits<double>::quiet_NaN());}

  // Modes and measurement selection.
  bool debug_mode = false;
  bool enable_xy = true;
  bool enable_z = true;
  bool enable_mocap_xy = true;
  bool enable_rgbd = true;
  bool enable_mocap_z = true;
  bool enable_sonar = true;
  bool enable_partial_update = true;
  bool enable_mocap_switch = false;
  int mocap_override_channel = 4;       // index into RCRaw.values, 0..7
  bool enable_measurements = true;      // runtime switch for RGB-D updates

  // Chi-square gates on the squared Mahalanobis distance (+inf disables a gate).
  double mahalanobis_d_sonar = 20;
  double mahalanobis_d_rgbd_velocity = 20;
  double mahalanobis_d_mocap_z = 20;
  double mahalanobis_d_mocap_velocity = 20;

  // Nominal IMU period [s]: initial dt and the xy_Q scaling (xy_Q * dt^2).
  double estimator_dt = 0.002;

  // Correction candidate C1 (BASELINE_DECISION.md section 7): clear the XY
  // measurement flag after a partial update, so each observation is fused
  // once. NOT APPROVED (deferred to R1); false reproduces master (D1).
  bool correction_c1 = false;

  std::string mocap_twist_topic = "mocap_velocity/body_level_frame";
  std::string mocap_pose_topic = "mocap_ned";
  std::string rgbd_twist_topic = "rgbd_velocity_body_frame";

  // Horizontal filter, state [x_dot, y_dot, pitch_bias, roll_bias, xa_bias, ya_bias].
  Mat<6, 1> xy_x0 = unset<6, 1>();
  Mat<6, 6> xy_P0 = unset<6, 6>();
  Mat<6, 6> xy_Q = unset<6, 6>();
  Mat<2, 2> xy_R0 = unset<2, 2>();
  Mat<6, 1> xy_beta = unset<6, 1>();

  // Vertical filter, state [z, z_dot, bias]; process noise on [z_dot, bias].
  Mat<3, 1> z_x0 = unset<3, 1>();
  Mat<3, 3> z_P0 = unset<3, 3>();
  Mat<3, 3> z_P0_flying = unset<3, 3>();
  Mat<2, 2> z_Q = unset<2, 2>();
  Mat<1, 1> z_R0 = unset<1, 1>();
  Mat<1, 1> z_R_flying = unset<1, 1>();
  Mat<3, 1> z_beta = unset<3, 1>();
};

// Returns one message per violated rule; empty if the parameters are valid.
std::vector<std::string> validate(const EstimatorParameters & p);

// Legal but probably unintended combinations (for example, no measurement
// source for an enabled filter). Informational only.
std::vector<std::string> warnings(const EstimatorParameters & p);

}  // namespace reef_estimator

#endif  // REEF_ESTIMATOR__PARAMETERS_HPP_

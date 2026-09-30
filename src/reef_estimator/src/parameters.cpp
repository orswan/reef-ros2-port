#include "reef_estimator/parameters.hpp"

#include <cmath>

#include "reef_msgs/matrix_operation.h"

namespace reef_estimator
{

namespace
{

template<class M>
void finite(std::vector<std::string> & errors, const M & m, const std::string & name)
{
  for (Eigen::Index i = 0; i < m.size(); i++) {
    if (!std::isfinite(m.coeff(i))) {
      errors.push_back(name + "[" + std::to_string(i) + "] is not finite (not loaded?)");
      return;
    }
  }
}

template<class M>
void covariance(std::vector<std::string> & errors, const M & m, const std::string & name)
{
  const std::string e = reef_msgs::covarianceError(m, name);
  if (!e.empty()) {errors.push_back(e);}
}

template<class M>
void weights(std::vector<std::string> & errors, const M & m, const std::string & name)
{
  const std::string e = reef_msgs::rangeError(m, name, 0.0, 1.0);
  if (!e.empty()) {errors.push_back(e + " (partial-update weights)");}
}

void gate(std::vector<std::string> & errors, double d, const std::string & name)
{
  if (!(d > 0)) {
    errors.push_back(name + " = " + std::to_string(d) + " must be > 0 (+inf disables the gate)");
  }
}

}  // namespace

std::vector<std::string> validate(const EstimatorParameters & p)
{
  std::vector<std::string> e;
  if (p.mocap_override_channel < 0 || p.mocap_override_channel >= kRcRawChannels) {
    e.push_back("mocap_override_channel = " + std::to_string(p.mocap_override_channel) +
      " must be 0.." + std::to_string(kRcRawChannels - 1) + " (index into RCRaw.values)");
  }
  gate(e, p.mahalanobis_d_sonar, "mahalanobis_d_sonar");
  gate(e, p.mahalanobis_d_rgbd_velocity, "mahalanobis_d_rgbd_velocity");
  gate(e, p.mahalanobis_d_mocap_z, "mahalanobis_d_mocap_z");
  gate(e, p.mahalanobis_d_mocap_velocity, "mahalanobis_d_mocap_velocity");
  if (!(std::isfinite(p.estimator_dt) && p.estimator_dt > 0)) {
    e.push_back("estimator_dt = " + std::to_string(p.estimator_dt) + " must be finite and > 0");
  }
  for (const auto & [topic, name] : {std::pair{&p.mocap_twist_topic, "mocap_twist_topic"},
      std::pair{&p.mocap_pose_topic, "mocap_pose_topic"},
      std::pair{&p.rgbd_twist_topic, "rgbd_twist_topic"}})
  {
    if (topic->empty()) {e.push_back(std::string(name) + " is empty");}
  }

  finite(e, p.xy_x0, "xy_x0");
  covariance(e, p.xy_P0, "xy_P0");
  covariance(e, p.xy_Q, "xy_Q");
  covariance(e, p.xy_R0, "xy_R0");
  weights(e, p.xy_beta, "xy_beta");
  finite(e, p.z_x0, "z_x0");
  covariance(e, p.z_P0, "z_P0");
  covariance(e, p.z_P0_flying, "z_P0_flying");
  covariance(e, p.z_Q, "z_Q");
  covariance(e, p.z_R0, "z_R0");
  covariance(e, p.z_R_flying, "z_R_flying");
  weights(e, p.z_beta, "z_beta");
  return e;
}

std::vector<std::string> warnings(const EstimatorParameters & p)
{
  std::vector<std::string> w;
  if (p.enable_xy && !p.enable_mocap_xy && !p.enable_rgbd) {
    w.push_back("enable_xy is true but neither enable_mocap_xy nor enable_rgbd is: "
      "the horizontal filter only propagates");
  }
  if (p.enable_z && !p.enable_mocap_z && !p.enable_sonar) {
    w.push_back("enable_z is true but neither enable_mocap_z nor enable_sonar is: "
      "the vertical filter only propagates");
  }
  if (p.enable_mocap_switch && !(p.enable_mocap_xy || p.enable_mocap_z)) {
    w.push_back("enable_mocap_switch is true but no mocap input is enabled: the switch has no effect");
  }
  if (p.enable_rgbd && !p.enable_measurements) {
    w.push_back("enable_rgbd is true but enable_measurements is false: RGB-D messages are ignored");
  }
  return w;
}

}  // namespace reef_estimator

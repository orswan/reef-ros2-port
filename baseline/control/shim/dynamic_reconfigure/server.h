#pragma once
// Adaptation C2 (baseline/README.md): stand-in for dynamic_reconfigure's
// Server<ConfigType> (ROS Melodic server.h), reduced to what reef_control
// uses, with the same order of operations:
//   constructor (NodeHandle "~"): config = default; __fromServer__; __clamp__
//   setCallback(cb):              cb(config, ~0) immediately
//   runtime update (harness):     new = config with the changed fields;
//                                 __clamp__; cb(new, level); config = new
#include <boost/function.hpp>
#include <ros/ros.h>
#include <cstdint>

namespace dynamic_reconfigure {
template <class ConfigType>
class Server {
 public:
  typedef boost::function<void(ConfigType&, uint32_t)> CallbackType;
  explicit Server(const ros::NodeHandle& nh = ros::NodeHandle("~")) : nh_(nh) {
    config_ = ConfigType::__getDefault__();
    config_.__fromServer__(nh_);
    config_.__clamp__();
  }
  void setCallback(const CallbackType& cb) { callback_ = cb; if (callback_) callback_(config_, ~0u); }
  // Harness only: apply a runtime reconfigure request.
  template <class F> void update(F change) {
    ConfigType n = config_; change(n); n.__clamp__();
    if (callback_) callback_(n, ~0u);
    config_ = n;
  }
  const ConfigType& config() const { return config_; }
 private:
  ros::NodeHandle nh_;
  ConfigType config_;
  CallbackType callback_;
};
}  // namespace dynamic_reconfigure

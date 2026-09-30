// Minimal stand-in for the roscpp API used by the pinned REEF estimator
// sources (reef_estimator e4179f48 / 95987b51, reef_msgs 7fb63ff9). It
// provides only what those files call. Behaviour follows ROS 1 where it
// matters to the estimator:
//   ros::Time::toSec()       = (double)sec + 1e-9 * (double)nsec
//   NodeHandle::param<T>()   = parameter value if present, else the default
//   getParam(double)         = accepts integer-valued parameters (roscpp does)
//   ROS_ASSERT               = enabled (catkin default build defines no NDEBUG)
// Publishers record what was published; subscribers are inert (the harness
// calls the callbacks directly, in fixture order).
#pragma once
#include <cmath>
#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

namespace ros {

struct Time {
  uint32_t sec = 0, nsec = 0;
  double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); }
};

// Flat parameter store shared by all NodeHandles (the sources use distinct names).
struct ParamValue {
  enum Kind { Bool, Double, String, List } kind = Double;
  bool b = false; double d = 0; std::string s; std::vector<double> list;
};
std::map<std::string, ParamValue>& param_store();

struct Publisher {
  std::string topic;
  template <class M> void publish(const M&) const;
};
std::map<std::string, long>& publish_counts();
template <class M> void Publisher::publish(const M&) const { ++publish_counts()[topic]; }

struct Subscriber {};

class NodeHandle {
 public:
  explicit NodeHandle(const std::string& ns = "") : ns_(ns) {}
  const std::string& getNamespace() const { return ns_; }

  bool getParam(const std::string& k, std::vector<double>& v) const {
    auto it = param_store().find(k);
    if (it == param_store().end() || it->second.kind != ParamValue::List) return false;
    v = it->second.list; return true;
  }
  bool getParam(const std::string& k, double& v) const {
    auto it = param_store().find(k);
    if (it == param_store().end() || it->second.kind != ParamValue::Double) return false;
    v = it->second.d; return true;
  }
  template <class T> void param(const std::string& k, T& v, const T& def) const;

  template <class M> Publisher advertise(const std::string& topic, int, bool = false) const { return Publisher{topic}; }
  template <class M, class C> Subscriber subscribe(const std::string&, int, void (C::*)(M), C*) const { return {}; }

 private:
  std::string ns_;
};

template <> inline void NodeHandle::param<bool>(const std::string& k, bool& v, const bool& def) const {
  auto it = param_store().find(k);
  v = (it != param_store().end() && it->second.kind == ParamValue::Bool) ? it->second.b : def;
}
template <> inline void NodeHandle::param<double>(const std::string& k, double& v, const double& def) const {
  auto it = param_store().find(k);
  v = (it != param_store().end() && it->second.kind == ParamValue::Double) ? it->second.d : def;
}
template <> inline void NodeHandle::param<int>(const std::string& k, int& v, const int& def) const {
  auto it = param_store().find(k);
  v = (it != param_store().end() && it->second.kind == ParamValue::Double) ? static_cast<int>(it->second.d) : def;
}
template <> inline void NodeHandle::param<std::string>(const std::string& k, std::string& v, const std::string& def) const {
  auto it = param_store().find(k);
  v = (it != param_store().end() && it->second.kind == ParamValue::String) ? it->second.s : def;
}

bool& log_enabled();
}  // namespace ros

#define REEF_SHIM_LOG(level, ...) do { if (ros::log_enabled()) { std::fprintf(stderr, "[" level "] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } } while (0)
#define REEF_SHIM_LOG_STREAM(level, args) do { if (ros::log_enabled()) { std::ostringstream o_; o_ << args; std::cerr << "[" level "] " << o_.str() << "\n"; } } while (0)
#define ROS_INFO(...) REEF_SHIM_LOG("INFO", __VA_ARGS__)
#define ROS_WARN(...) REEF_SHIM_LOG("WARN", __VA_ARGS__)
#define ROS_ERROR(...) REEF_SHIM_LOG("ERROR", __VA_ARGS__)
#define ROS_INFO_STREAM(args) REEF_SHIM_LOG_STREAM("INFO", args)
#define ROS_WARN_STREAM(args) REEF_SHIM_LOG_STREAM("WARN", args)
#define ROS_ERROR_STREAM(args) REEF_SHIM_LOG_STREAM("ERROR", args)
#define ROS_ASSERT(cond) do { if (!(cond)) { std::fprintf(stderr, "ROS_ASSERT failed: %s\n", #cond); std::abort(); } } while (0)
#define ROS_ASSERT_MSG(cond, ...) do { if (!(cond)) { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); std::abort(); } } while (0)

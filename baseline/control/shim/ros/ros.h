// Minimal stand-in for the roscpp API used by the pinned reef_control
// sources (12237b76). It provides only what those files call, with ROS 1
// semantics where they matter to the controller:
//   Time - Time              = Duration normalized as roscpp_core
//                              (sec floor-normalized, nsec in [0, 1e9))
//   Duration::toSec()        = (double)sec + 1e-9 * (double)nsec
//   NodeHandle("~")          = private namespace: keys are "~name"
//   getParam/param (double)  = accepts integer-valued parameters (roscpp does)
//   getParam/param (bool)    = accepts only boolean parameters (roscpp does)
//   ROS_ASSERT               = enabled (catkin default build defines no NDEBUG)
// Subscribers store the callback; the harness calls them in fixture order.
// Publishers record what was published (count and last message).
#pragma once
#include <cmath>
#include <math.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

namespace ros {

struct Duration {
  int32_t sec = 0, nsec = 0;
  Duration() = default;
  Duration(int32_t s, int32_t n) : sec(s), nsec(n) {
    int64_t nsec_part = static_cast<int64_t>(n) % 1000000000LL;
    int64_t sec_part = static_cast<int64_t>(s) + static_cast<int64_t>(n) / 1000000000LL;
    if (nsec_part < 0) { nsec_part += 1000000000LL; --sec_part; }
    sec = static_cast<int32_t>(sec_part); nsec = static_cast<int32_t>(nsec_part);
  }
  double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); }
};

struct Time {
  uint32_t sec = 0, nsec = 0;
  Time() = default;
  explicit Time(double t) {  // ros::Time(0) in the sources
    sec = static_cast<uint32_t>(std::floor(t));
    nsec = static_cast<uint32_t>(std::lround((t - sec) * 1e9));
  }
  Time(uint32_t s, uint32_t n) : sec(s), nsec(n) {}
  double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); }
  // roscpp_core: D(int64 difference of toNSec()) normalized.
  Duration operator-(const Time& rhs) const {
    const int64_t d = (static_cast<int64_t>(sec) * 1000000000LL + nsec) - (static_cast<int64_t>(rhs.sec) * 1000000000LL + rhs.nsec);
    return Duration(static_cast<int32_t>(d / 1000000000LL), static_cast<int32_t>(d % 1000000000LL));
  }
};

struct ParamValue {
  enum Kind { Bool, Int, Double } kind = Double;
  bool b = false; int i = 0; double d = 0;
};
std::map<std::string, ParamValue>& param_store();

// Publishers: count per topic and hand the message to record_published
// (overloads for the controller's message types are defined by the harness
// and found by argument-dependent lookup).
template <class M> inline void record_published(const std::string&, const M&) {}
struct Publisher {
  std::string topic;
  template <class M> void publish(const M& m) const { ++publish_counts()[topic]; record_published(topic, m); }
  static std::map<std::string, long>& publish_counts();
};

// Subscribers: callbacks by topic, type-erased on the message type.
struct Subscriber {};
template <class M> std::map<std::string, std::function<void(const M&)>>& subscriptions() {
  static std::map<std::string, std::function<void(const M&)>> s; return s;
}

class NodeHandle {
 public:
  explicit NodeHandle(const std::string& ns = "") : ns_(ns) {}
  std::string key(const std::string& k) const { return ns_ == "~" ? "~" + k : k; }

  bool getParam(const std::string& k, double& v) const {
    auto it = param_store().find(key(k));
    if (it == param_store().end()) return false;
    if (it->second.kind == ParamValue::Double) { v = it->second.d; return true; }
    if (it->second.kind == ParamValue::Int) { v = it->second.i; return true; }
    return false;
  }
  bool getParam(const std::string& k, bool& v) const {
    auto it = param_store().find(key(k));
    if (it == param_store().end() || it->second.kind != ParamValue::Bool) return false;
    v = it->second.b; return true;
  }
  template <class T> void param(const std::string& k, T& v, const T& def) const { if (!getParam(k, v)) v = def; }

  template <class M> Publisher advertise(const std::string& topic, int, bool = false) const { return Publisher{topic}; }
  template <class M, class C> Subscriber subscribe(const std::string& topic, int, void (C::*f)(const M&), C* obj) const {
    subscriptions<M>()[topic] = [f, obj](const M& m) { (obj->*f)(m); };
    return {};
  }

 private:
  std::string ns_;
};

bool& log_enabled();
}  // namespace ros

#define REEF_SHIM_LOG(level, ...) do { if (ros::log_enabled()) { std::fprintf(stderr, "[" level "] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } } while (0)
#define ROS_INFO(...) REEF_SHIM_LOG("INFO", __VA_ARGS__)
#define ROS_WARN(...) REEF_SHIM_LOG("WARN", __VA_ARGS__)
#define ROS_ERROR(...) REEF_SHIM_LOG("ERROR", __VA_ARGS__)
#define ROS_ASSERT(cond) do { if (!(cond)) { std::fprintf(stderr, "ROS_ASSERT failed: %s\n", #cond); std::exit(3); } } while (0)
#define ROS_ASSERT_MSG(cond, ...) do { if (!(cond)) { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); std::exit(3); } } while (0)

// Readers for the P02 reference-harness formats (baseline/harness/reef_ref_main.cpp):
//   parameters: "name bool|double|string|list values..."
//   events:     "type t_ns fields..." (imu, imu_nan, range, mocap_pose, mocap_twist, rgbd, rc)
// Used by the event-replay driver and the wrapper tests, so both feed
// exactly what the reference harness feeds the original.
#ifndef REEF_ESTIMATOR__TOOLS__EVENT_FILE_HPP_
#define REEF_ESTIMATOR__TOOLS__EVENT_FILE_HPP_

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "reef_estimator/parameters.hpp"
#include "reef_estimator/vertical_estimator.h"
#include "reef_msgs/matrix_operation.h"

namespace reef_estimator::tools
{

inline double number(const std::string & tok)
{
  // strtod reads "inf", "-inf", "nan" (as the harness does).
  char * end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') {
    throw std::runtime_error("bad number '" + tok + "'");
  }
  return v;
}

// Harness stamp_of(): integer nanoseconds -> (sec, nsec).
inline Stamp stamp_of(long long t_ns)
{
  Stamp s;
  s.sec = static_cast<int32_t>(t_ns / 1000000000LL);
  s.nanosec = static_cast<uint32_t>(t_ns % 1000000000LL);
  return s;
}

template<class M>
void matrix(M & m, const std::vector<double> & v, const std::string & name)
{
  const auto r = reef_msgs::importMatrixFromVector(m, v, name);
  if (!r.ok) {throw std::runtime_error(r.error);}
}

inline EstimatorParameters load_params(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("cannot open " + path);}
  EstimatorParameters p;
  std::map<std::string, bool *> bools{
    {"debug_mode", &p.debug_mode}, {"enable_xy", &p.enable_xy}, {"enable_z", &p.enable_z},
    {"enable_mocap_xy", &p.enable_mocap_xy}, {"enable_rgbd", &p.enable_rgbd},
    {"enable_mocap_z", &p.enable_mocap_z}, {"enable_sonar", &p.enable_sonar},
    {"enable_partial_update", &p.enable_partial_update},
    {"enable_mocap_switch", &p.enable_mocap_switch},
    {"enable_measurements", &p.enable_measurements}};
  std::map<std::string, double *> doubles{
    {"mahalanobis_d_sonar", &p.mahalanobis_d_sonar},
    {"mahalanobis_d_rgbd_velocity", &p.mahalanobis_d_rgbd_velocity},
    {"mahalanobis_d_mocap_z", &p.mahalanobis_d_mocap_z},
    {"mahalanobis_d_mocap_velocity", &p.mahalanobis_d_mocap_velocity},
    {"estimator_dt", &p.estimator_dt}};
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    std::string name, kind, tok;
    ss >> name >> kind;
    std::vector<double> vals;
    std::string str;
    while (ss >> tok) {
      if (kind == "string") {str = tok;} else if (kind != "bool") {vals.push_back(number(tok));} else {
        str = tok;
      }
    }
    if (kind == "bool" && bools.count(name)) {
      *bools[name] = (str == "true");
    } else if (kind == "double" && doubles.count(name)) {
      *doubles[name] = vals.at(0);
    } else if (kind == "double" && name == "mocap_override_channel") {
      // The harness writes every number as a double; roscpp param<int> read
      // the YAML integer.
      p.mocap_override_channel = static_cast<int>(vals.at(0));
    } else if (kind == "string" && name == "mocap_twist_topic") {
      p.mocap_twist_topic = str;
    } else if (kind == "string" && name == "mocap_pose_topic") {
      p.mocap_pose_topic = str;
    } else if (kind == "string" && name == "rgbd_twist_topic") {
      p.rgbd_twist_topic = str;
    } else if (kind == "list") {
      if (name == "xy_x0") {matrix(p.xy_x0, vals, name);} else if (name == "xy_P0") {
        matrix(p.xy_P0, vals, name);
      } else if (name == "xy_Q") {matrix(p.xy_Q, vals, name);} else if (name == "xy_R0") {
        matrix(p.xy_R0, vals, name);
      } else if (name == "xy_beta") {matrix(p.xy_beta, vals, name);} else if (name == "z_x0") {
        matrix(p.z_x0, vals, name);
      } else if (name == "z_P0") {matrix(p.z_P0, vals, name);} else if (name == "z_P0_flying") {
        matrix(p.z_P0_flying, vals, name);
      } else if (name == "z_Q") {matrix(p.z_Q, vals, name);} else if (name == "z_R0") {
        matrix(p.z_R0, vals, name);
      } else if (name == "z_R_flying") {matrix(p.z_R_flying, vals, name);} else if (name == "z_beta") {
        matrix(p.z_beta, vals, name);
      } else {
        throw std::runtime_error("unknown matrix parameter " + name);
      }
    } else {
      throw std::runtime_error("unknown parameter line: " + line);
    }
  }
  const auto errors = validate(p);
  if (!errors.empty()) {throw std::runtime_error("invalid parameters: " + errors.front());}
  return p;
}

struct Event
{
  std::string type;
  long long t_ns = 0;
  std::vector<double> f;   // numeric fields after the stamp
};

inline std::vector<Event> load_events(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("cannot open " + path);}
  std::vector<Event> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    Event e;
    std::string t, tok;
    ss >> e.type >> t;
    e.t_ns = std::stoll(t);
    while (ss >> tok) {e.f.push_back(number(tok));}
    out.push_back(e);
  }
  return out;
}

// Core-level samples for one event (as the harness builds its messages).
inline ImuSample imu_sample(const Event & e)
{
  ImuSample s;
  s.stamp = stamp_of(e.t_ns);
  if (e.type == "imu_nan") {
    s.ax = s.ay = s.az = std::numeric_limits<double>::quiet_NaN();
  } else {
    s.ax = e.f.at(0); s.ay = e.f.at(1); s.az = e.f.at(2);
    s.qx = e.f.at(3); s.qy = e.f.at(4); s.qz = e.f.at(5); s.qw = e.f.at(6);
  }
  return s;
}

inline RangeSample range_sample(const Event & e)
{
  RangeSample s;
  s.stamp = stamp_of(e.t_ns);
  s.range = static_cast<float>(e.f.at(0));
  s.max_range = static_cast<float>(e.f.at(1));
  return s;
}

inline MocapPoseSample mocap_pose_sample(const Event & e)
{
  MocapPoseSample s;
  s.stamp = stamp_of(e.t_ns);
  s.z = e.f.at(0);
  return s;
}

inline RcSample rc_sample(const Event & e)
{
  RcSample s;
  s.stamp = stamp_of(e.t_ns);
  for (std::size_t i = 0; i < s.values.size(); i++) {
    s.values[i] = static_cast<uint16_t>(e.f.at(i));
  }
  return s;
}

}  // namespace reef_estimator::tools

#endif  // REEF_ESTIMATOR__TOOLS__EVENT_FILE_HPP_

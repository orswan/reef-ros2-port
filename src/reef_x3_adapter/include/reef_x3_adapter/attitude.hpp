// Attitude conversion for the X3 -> REEF IMU adapter, without ROS.
//
// Same algorithm as reef_sim/reef_adapter.py (used by the offline replay):
// q_NED<-FRD = q_T * q_ENU<-FLU * q_B with T = [[0,1,0],[1,0,0],[0,0,-1]]
// (180 deg about (1,1,0)/sqrt 2) and B = diag(1,-1,-1) (180 deg about x),
// canonical sign w >= 0; slerp between the truth samples around the IMU
// stamp, else extrapolated from the last two (at most 20 ms), else the
// last sample. IDEALIZED: the attitude is simulation truth.
#ifndef REEF_X3_ADAPTER__ATTITUDE_HPP_
#define REEF_X3_ADAPTER__ATTITUDE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

namespace reef_x3_adapter
{

using Quat = std::array<double, 4>;   // w, x, y, z

constexpr int64_t kMaxExtrapolationNs = 20000000;

inline Quat qmul(const Quat & a, const Quat & b)
{
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline Quat ned_frd_from_enu_flu(const Quat & q)
{
  const double s = std::sqrt(0.5);
  const Quat q_t{0.0, s, s, 0.0};
  const Quat q_b{0.0, 1.0, 0.0, 0.0};
  Quat r = qmul(qmul(q_t, q), q_b);
  if (r[0] < 0) {
    for (double & c : r) {c = -c;}
  }
  return r;
}

inline Quat slerp(const Quat & a, Quat b, double s)
{
  double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0) {
    for (double & c : b) {c = -c;}
    dot = -dot;
  }
  Quat q;
  if (dot > 0.9995) {
    for (int i = 0; i < 4; i++) {q[i] = a[i] + s * (b[i] - a[i]);}
  } else {
    const double th = std::acos(dot);
    const double sa = std::sin((1 - s) * th) / std::sin(th), sb = std::sin(s * th) / std::sin(th);
    for (int i = 0; i < 4; i++) {q[i] = sa * a[i] + sb * b[i];}
  }
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (double & c : q) {c /= n;}
  return q;
}

class TruthBuffer
{
public:
  explicit TruthBuffer(std::size_t keep = 16) : keep_(keep) {}

  void add(int64_t t_ns, const Quat & q_enu_flu)
  {
    if (!samples_.empty() && t_ns <= samples_.back().first) {return;}
    samples_.emplace_back(t_ns, q_enu_flu);
    while (samples_.size() > keep_) {samples_.pop_front();}
  }

  // q_NED<-FRD at t_ns, or nothing before the second truth sample.
  std::optional<Quat> at(int64_t t_ns) const
  {
    if (samples_.size() < 2 || t_ns < samples_.front().first) {return std::nullopt;}
    if (t_ns > samples_.back().first) {
      const auto & p0 = samples_[samples_.size() - 2];
      const auto & p1 = samples_.back();
      if (t_ns - p1.first > kMaxExtrapolationNs) {return ned_frd_from_enu_flu(p1.second);}
      return ned_frd_from_enu_flu(slerp(p0.second, p1.second,
               static_cast<double>(t_ns - p0.first) / static_cast<double>(p1.first - p0.first)));
    }
    for (std::size_t k = samples_.size() - 1; k > 0; k--) {
      const auto & prev = samples_[k - 1];
      const auto & cur = samples_[k];
      if (prev.first <= t_ns && t_ns <= cur.first) {
        if (t_ns == cur.first) {return ned_frd_from_enu_flu(cur.second);}
        return ned_frd_from_enu_flu(slerp(prev.second, cur.second,
                 static_cast<double>(t_ns - prev.first) / static_cast<double>(cur.first - prev.first)));
      }
    }
    return ned_frd_from_enu_flu(samples_.front().second);
  }

private:
  std::size_t keep_;
  std::deque<std::pair<int64_t, Quat>> samples_;
};

}  // namespace reef_x3_adapter

#endif  // REEF_X3_ADAPTER__ATTITUDE_HPP_

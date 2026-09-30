#include "reef_control/gains.hpp"

#include <cmath>
#include <sstream>

namespace reef_control
{

const std::vector<GainRange>& gainRanges()
{
#define REEF_CONTROL_RANGE(n, d, lo, hi) {#n, d, lo, hi},
  static const std::vector<GainRange> r = {REEF_CONTROL_GAIN_DOUBLES(REEF_CONTROL_RANGE)};
#undef REEF_CONTROL_RANGE
  return r;
}

const std::vector<std::string>& gainBools()
{
#define REEF_CONTROL_BOOL_NAME(n, d) #n,
  static const std::vector<std::string> b = {REEF_CONTROL_GAIN_BOOLS(REEF_CONTROL_BOOL_NAME)};
#undef REEF_CONTROL_BOOL_NAME
  return b;
}

double* gainField(GainsConfig& g, const std::string& name)
{
#define REEF_CONTROL_FIELD(n, d, lo, hi) if (name == #n) return &g.n;
  REEF_CONTROL_GAIN_DOUBLES(REEF_CONTROL_FIELD)
#undef REEF_CONTROL_FIELD
  return nullptr;
}

bool* gainBoolField(GainsConfig& g, const std::string& name)
{
#define REEF_CONTROL_BFIELD(n, d) if (name == #n) return &g.n;
  REEF_CONTROL_GAIN_BOOLS(REEF_CONTROL_BFIELD)
#undef REEF_CONTROL_BFIELD
  return nullptr;
}

std::vector<std::string> gainErrors(const GainsConfig& g)
{
  std::vector<std::string> e;
  GainsConfig copy = g;
  for (const auto& r : gainRanges()) {
    const double v = *gainField(copy, r.name);
    if (!std::isfinite(v) || v < r.lo || v > r.hi) {
      std::ostringstream o;
      o << r.name << " = " << v << " is outside [" << r.lo << ", " << r.hi << "] (reef_control cfg/Gains.cfg)";
      e.push_back(o.str());
    }
  }
  return e;
}

std::vector<std::string> parameterErrors(const ControllerParameters& p)
{
  std::vector<std::string> e = gainErrors(p.gains);
  const std::pair<const char*, double> limits[] = {
    {"max_roll", p.max_roll}, {"max_pitch", p.max_pitch}, {"max_yaw_rate", p.max_yaw_rate}};
  for (const auto& l : limits) {
    if (!std::isfinite(l.second) || l.second < 0) {
      std::ostringstream o;
      o << l.first << " = " << l.second << " must be finite and >= 0";
      e.push_back(o.str());
    }
  }
  return e;
}

}  // namespace reef_control

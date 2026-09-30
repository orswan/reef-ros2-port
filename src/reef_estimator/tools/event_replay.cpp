// Event-replay driver for the fidelity comparison (P04).
//   reef_estimator_event_replay PARAMS EVENTS OUT.csv
// Reads the P02 harness formats and feeds each event to the ported vertical
// estimator in file order, with the ROS 1 node's subscription rules, then
// writes the vertical state after every event using the harness column names.
// Horizontal events (mocap_twist, rgbd) are counted as delivered when the
// original would have subscribed, but the vertical filter never uses them.
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include "event_file.hpp"

namespace
{

void put(std::ostream & o, double x) {o << ',' << x;}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s PARAMS EVENTS OUT.csv\n", argv[0]);
    return 2;
  }
  using namespace reef_estimator;
  try {
    const EstimatorParameters params = tools::load_params(argv[1]);
    VerticalEstimator e(params);
    const auto events = tools::load_events(argv[2]);

    std::ofstream out(argv[3]);
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "idx,type,t_ns,z_flag_before,z,zdot,zbias";
    for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {out << ",zP" << i << j;}}
    out << ",z_meas,zR,u,z_dt,g_init,acc_init,takeoff,z_flag,n_prop,maha2,z_gate,"
      "use_mocap_z,n_published,delivered\n";

    long idx = 0;
    for (const auto & ev : events) {
      const bool z_before = e.pendingZMeasurement();
      const long gates_before = e.zGateCount();
      bool delivered = true;
      if (ev.type == "imu" || ev.type == "imu_nan") {
        e.sensorUpdate(tools::imu_sample(ev));
      } else if (ev.type == "range") {
        delivered = e.subscribesRange();
        if (delivered) {e.sensorUpdate(tools::range_sample(ev));}
      } else if (ev.type == "mocap_pose") {
        delivered = e.subscribesMocapPose();
        if (delivered) {e.mocapUpdate(tools::mocap_pose_sample(ev));}
      } else if (ev.type == "rc") {
        delivered = e.subscribesRc();
        if (delivered) {e.rcRawUpdate(tools::rc_sample(ev));}
      } else if (ev.type == "mocap_twist") {
        delivered = params.enable_mocap_xy;
      } else if (ev.type == "rgbd") {
        delivered = params.enable_rgbd;
      } else {
        std::fprintf(stderr, "unknown event type '%s'\n", ev.type.c_str());
        return 2;
      }
      const ZEstimator & z = e.zFilter();
      out << idx++ << ',' << ev.type << ',' << ev.t_ns << ',' << z_before;
      for (int i = 0; i < 3; ++i) {put(out, z.xHat(i, 0));}
      for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {put(out, z.P(i, j));}}
      put(out, z.z(0)); put(out, z.R(0, 0)); put(out, z.u(0)); put(out, z.dt);
      put(out, e.accelerometerInitialized() ? e.initialGravity() :
        std::numeric_limits<double>::quiet_NaN());
      out << ',' << e.accelerometerInitialized() << ',' << e.isFlying() << ','
          << e.pendingZMeasurement() << ',' << e.propagationCount();
      put(out, e.lastMahalanobisSquared());
      out << ',' << (e.zGateCount() != gates_before) << ',' << e.usingMocapZ() << ','
          << e.estimateCount() << ',' << delivered << '\n';
    }
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "reef_estimator_event_replay: %s\n", ex.what());
    return 2;
  }
  return 0;
}

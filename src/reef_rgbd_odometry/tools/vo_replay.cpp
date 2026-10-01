// Offline replay of the odometry core (P08 development tool, not a test):
//   vo_replay FRAMES_DIR [key=value ...] > out.csv
// FRAMES_DIR (tools/export_frames.py): meta.txt "N width height fx fy cx cy",
// stamps.txt (ns, one per frame), grey.u8 (N*w*h), depth.f32 (N*w*h).
// key=value overrides Params fields (see odometry.hpp). Prints one CSV row per
// frame: stamp_ns, state, publish, tracks, depth_tracks, inliers, rms, p (3),
// R (row-major 9), in the init optical frame.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "reef_rgbd_odometry/odometry.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {std::cerr << "usage: vo_replay FRAMES_DIR [key=value ...]\n"; return 2;}
  const std::string dir = argv[1];
  reef_rgbd_odometry::Params p;
  std::map<std::string, double *> d{{"quality", &p.quality}, {"min_distance", &p.min_distance},
    {"fb_max", &p.fb_max}, {"ransac_distance", &p.ransac_distance},
    {"max_step_translation", &p.max_step_translation}, {"max_step_rotation", &p.max_step_rotation},
    {"pixel_offset", &p.pixel_offset}, {"near_clip", &p.near_clip}, {"depth_discontinuity", &p.depth_discontinuity}, {"far_clip", &p.far_clip}};
  std::map<std::string, int *> n{{"max_features", &p.max_features}, {"min_features", &p.min_features},
    {"lk_window", &p.lk_window}, {"lk_levels", &p.lk_levels}, {"min_depth_tracks", &p.min_depth_tracks},
    {"min_inliers", &p.min_inliers}, {"ransac_iterations", &p.ransac_iterations},
    {"recover_frames", &p.recover_frames}};
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const auto eq = a.find('=');
    const std::string k = a.substr(0, eq), v = a.substr(eq + 1);
    if (d.count(k)) {*d[k] = std::stod(v);} else if (n.count(k)) {*n[k] = std::stoi(v);} else {
      std::cerr << "unknown parameter " << k << "\n";
      return 2;
    }
  }
  std::ifstream meta(dir + "/meta.txt"), stamps(dir + "/stamps.txt");
  std::ifstream grey(dir + "/grey.u8", std::ios::binary), depth(dir + "/depth.f32", std::ios::binary);
  long count;
  int w, h;
  reef_rgbd_odometry::Intrinsics k;
  if (!(meta >> count >> w >> h >> k.fx >> k.fy >> k.cx >> k.cy) || !grey || !depth) {
    std::cerr << "cannot read " << dir << "\n";
    return 2;
  }
  reef_rgbd_odometry::RgbdOdometry vo(p);
  cv::Mat g(h, w, CV_8UC1), z(h, w, CV_32FC1);
  std::printf("stamp_ns,state,publish,tracks,depth_tracks,inliers,rms,px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22\n");
  for (long i = 0; i < count; ++i) {
    long long t;
    stamps >> t;
    grey.read(reinterpret_cast<char *>(g.data), static_cast<std::streamsize>(w) * h);
    depth.read(reinterpret_cast<char *>(z.data), static_cast<std::streamsize>(w) * h * 4);
    if (!grey || !depth) {std::cerr << "short read at frame " << i << "\n"; return 2;}
    const auto r = vo.process(g, z, k);
    std::printf("%lld,%s,%d,%d,%d,%d,%.4f,%.9f,%.9f,%.9f", t, reef_rgbd_odometry::toString(r.state).c_str(),
      r.publish ? 1 : 0, r.tracks, r.depth_tracks, r.inliers, r.reprojection_rms, r.p.x(), r.p.y(), r.p.z());
    for (int a = 0; a < 3; ++a) {for (int b = 0; b < 3; ++b) {std::printf(",%.9f", r.R(a, b));}}
    std::printf("\n");
  }
  return 0;
}

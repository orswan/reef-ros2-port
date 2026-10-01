// Compact RGB-D visual odometry (P08): a REPLACEMENT for demo_rgbd (USER,
// option B), not a port. Design: docs/VISION.md section 6. ROS-free (OpenCV 4,
// Eigen).
#ifndef REEF_RGBD_ODOMETRY_ODOMETRY_HPP
#define REEF_RGBD_ODOMETRY_ODOMETRY_HPP

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace reef_rgbd_odometry
{

struct Params
{
  int max_features = 300;
  int min_features = 120;       // replenish below this many tracks
  double quality = 0.01;
  double min_distance = 8.0;    // px
  int lk_window = 21;
  int lk_levels = 3;
  double fb_max = 1.0;          // forward-backward error [px]
  double near_clip = 0.3, far_clip = 8.0;
  double depth_discontinuity = 0.05;  // reject depth if the 4 neighbours differ by more (relative)
  int min_depth_tracks = 40;    // depth loss below this
  int min_inliers = 25;         // weak texture below this
  // inlier: 3-D residual <= this x depth (0.1 %: 3.9 mm at 3.9 m, 0.28 px laterally). Below the
  // per-frame motion, as needed to separate translation from rotation; it relies on the simulated
  // depth being exact (a real sensor needs a threshold matched to its depth noise).
  double ransac_distance = 0.001;
  int ransac_iterations = 200;
  double max_step_translation = 0.5;  // m per frame
  double max_step_rotation = 0.3;     // rad per frame
  int recover_frames = 2;
  double pixel_offset = 0.5;    // Gazebo pixel-centre convention (VISION.md section 5)
};

struct Intrinsics { double fx = 0, fy = 0, cx = 0, cy = 0; };

enum class State { Init, Ok, Lost };
std::string toString(State s);

struct Result
{
  State state = State::Init;
  bool publish = false;          // a pose for cam_to_init
  std::string reason;            // why Lost (or "")
  int tracks = 0, depth_tracks = 0, inliers = 0;
  double reprojection_rms = 0;   // px, inliers
  // Pose of the camera in the init (first) optical frame.
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

class RgbdOdometry
{
public:
  explicit RgbdOdometry(const Params & p = Params()) : p_(p) {}
  // grey: CV_8UC1; depth: CV_32FC1 planar metres (same size).
  Result process(const cv::Mat & grey, const cv::Mat & depth, const Intrinsics & k);
  long lostEvents() const {return lost_events_;}

private:
  void detect(const cv::Mat & grey);
  Params p_;
  cv::Mat prev_grey_, prev_depth_;
  std::vector<cv::Point2f> prev_pts_;
  bool have_prev_ = false;
  State state_ = State::Init;
  int ok_streak_ = 0;
  long lost_events_ = 0;
  cv::RNG rng_{0x5eefULL};   // RANSAC samples (seeded: deterministic replay)
  // Published pose (the chain continues from it after a loss).
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_ = Eigen::Vector3d::Zero();
};

// DEMO's camera convention (x left, y up, z forward) from the optical frame.
void toDemo(const Eigen::Matrix3d & R_opt, const Eigen::Vector3d & p_opt, Eigen::Matrix3d & R_demo,
  Eigen::Vector3d & p_demo);

}  // namespace reef_rgbd_odometry

#endif

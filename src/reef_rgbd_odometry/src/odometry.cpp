#include "reef_rgbd_odometry/odometry.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace reef_rgbd_odometry
{

std::string toString(State s)
{
  switch (s) {
    case State::Init: return "INIT";
    case State::Ok: return "OK";
    default: return "LOST";
  }
}

void toDemo(const Eigen::Matrix3d & R_opt, const Eigen::Vector3d & p_opt, Eigen::Matrix3d & R_demo,
  Eigen::Vector3d & p_demo)
{
  const Eigen::Matrix3d F = Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal();
  R_demo = F * R_opt * F;
  p_demo = F * p_opt;
}

void RgbdOdometry::detect(const cv::Mat & grey)
{
  cv::Mat mask(grey.size(), CV_8UC1, cv::Scalar(255));
  for (const auto & q : prev_pts_) {
    cv::circle(mask, q, static_cast<int>(p_.min_distance), cv::Scalar(0), -1);
  }
  const int want = p_.max_features - static_cast<int>(prev_pts_.size());
  if (want <= 0) {return;}
  std::vector<cv::Point2f> fresh;
  cv::goodFeaturesToTrack(grey, fresh, want, p_.quality, p_.min_distance, mask);
  prev_pts_.insert(prev_pts_.end(), fresh.begin(), fresh.end());
}

Result RgbdOdometry::process(const cv::Mat & grey, const cv::Mat & depth, const Intrinsics & k)
{
  Result r;
  auto lose = [&](const std::string & why) {
      if (state_ != State::Lost) {++lost_events_;}
      state_ = State::Lost;
      ok_streak_ = 0;
      r.state = State::Lost;
      r.reason = why;
      r.R = R_;
      r.p = t_;
      prev_pts_.clear();
      prev_grey_ = grey.clone();
      prev_depth_ = depth.clone();
      detect(grey);
      have_prev_ = true;
      return r;
    };
  if (!have_prev_) {
    prev_grey_ = grey.clone();
    prev_depth_ = depth.clone();
    prev_pts_.clear();
    detect(grey);
    have_prev_ = true;
    state_ = State::Ok;   // the first frame anchors the init frame
    r.state = State::Ok;
    r.publish = true;
    r.tracks = static_cast<int>(prev_pts_.size());
    return r;
  }
  if (prev_pts_.empty()) {return lose("no features");}
  std::vector<cv::Point2f> cur, back;
  std::vector<uchar> st, st2;
  std::vector<float> err;
  const cv::Size win(p_.lk_window, p_.lk_window);
  cv::calcOpticalFlowPyrLK(prev_grey_, grey, prev_pts_, cur, st, err, win, p_.lk_levels);
  cv::calcOpticalFlowPyrLK(grey, prev_grey_, cur, back, st2, err, win, p_.lk_levels);
  // Back-project each track in both frames (planar depth, Gazebo pixel-centre convention).
  // Depth: bilinear over the 4 neighbouring pixel centres (exact on planes); rejected if any of them
  // is invalid (outside the clip range: -inf/+inf/NaN never used) or they straddle a discontinuity.
  auto backproject = [&](const cv::Mat & d, const cv::Point2f & q, Eigen::Vector3d & X) {
      const int u = static_cast<int>(std::floor(q.x)), v = static_cast<int>(std::floor(q.y));
      if (u < 0 || v < 0 || u + 1 >= d.cols || v + 1 >= d.rows) {return false;}
      const float z00 = d.at<float>(v, u), z01 = d.at<float>(v, u + 1);
      const float z10 = d.at<float>(v + 1, u), z11 = d.at<float>(v + 1, u + 1);
      float lo = z00, hi = z00;
      for (float zz : {z00, z01, z10, z11}) {
        if (!std::isfinite(zz) || zz < p_.near_clip || zz > p_.far_clip) {return false;}
        lo = std::min(lo, zz);
        hi = std::max(hi, zz);
      }
      if (hi > lo * (1.0 + p_.depth_discontinuity)) {return false;}
      const double fu = q.x - u, fv = q.y - v;
      const double z = (1 - fv) * ((1 - fu) * z00 + fu * z01) + fv * ((1 - fu) * z10 + fu * z11);
      X = Eigen::Vector3d((q.x + p_.pixel_offset - k.cx) / k.fx * z, (q.y + p_.pixel_offset - k.cy) / k.fy * z, z);
      return true;
    };
  std::vector<Eigen::Vector3d> Xp, Xc;
  std::vector<cv::Point2f> kept;
  int tracks = 0;
  for (size_t i = 0; i < prev_pts_.size(); ++i) {
    if (!st[i] || !st2[i] || cv::norm(back[i] - prev_pts_[i]) > p_.fb_max) {continue;}
    if (cur[i].x < 0 || cur[i].y < 0 || cur[i].x > grey.cols - 1 || cur[i].y > grey.rows - 1) {continue;}
    ++tracks;
    kept.push_back(cur[i]);
    Eigen::Vector3d a, b;
    if (backproject(prev_depth_, prev_pts_[i], a) && backproject(depth, cur[i], b)) {
      Xp.push_back(a);
      Xc.push_back(b);
    }
  }
  r.tracks = tracks;
  r.depth_tracks = static_cast<int>(Xp.size());
  if (r.depth_tracks < p_.min_depth_tracks) {
    Result l = lose("depth: " + std::to_string(r.depth_tracks) + " depth-valid tracks");
    l.tracks = tracks;
    l.depth_tracks = r.depth_tracks;
    return l;
  }
  // 3-D to 3-D rigid motion X_cur = Rcp X_prev + tcp: RANSAC over 3-point samples (Kabsch), then
  // Kabsch on all inliers. (3-D to 2-D PnP confused translation parallel to the image plane with
  // rotation on the planar scene; the current depth separates them. VISION.md section 6.)
  const int n = r.depth_tracks;
  auto fit = [&](const std::vector<int> & idx, Eigen::Matrix3d & Rm, Eigen::Vector3d & tm) {
      Eigen::Matrix3Xd A(3, idx.size()), B(3, idx.size());
      for (size_t j = 0; j < idx.size(); ++j) {A.col(j) = Xp[idx[j]]; B.col(j) = Xc[idx[j]];}
      const Eigen::Matrix4d T = Eigen::umeyama(A, B, false);
      Rm = T.topLeftCorner<3, 3>();
      tm = T.topRightCorner<3, 1>();
    };
  auto inliers_of = [&](const Eigen::Matrix3d & Rm, const Eigen::Vector3d & tm) {
      std::vector<int> in;
      for (int j = 0; j < n; ++j) {
        if ((Rm * Xp[j] + tm - Xc[j]).norm() <= p_.ransac_distance * Xc[j].z()) {in.push_back(j);}
      }
      return in;
    };
  std::vector<int> best;
  Eigen::Matrix3d Rcp;
  Eigen::Vector3d tcp;
  for (int it = 0; it < p_.ransac_iterations; ++it) {
    std::vector<int> s3;
    while (s3.size() < 3) {
      const int j = static_cast<int>(rng_.uniform(0, n));
      if (std::find(s3.begin(), s3.end(), j) == s3.end()) {s3.push_back(j);}
    }
    Eigen::Matrix3d Rm;
    Eigen::Vector3d tm;
    fit(s3, Rm, tm);
    auto in = inliers_of(Rm, tm);
    if (in.size() > best.size()) {best.swap(in);}
  }
  r.inliers = static_cast<int>(best.size());
  if (r.inliers < p_.min_inliers) {
    Result l = lose("texture: " + std::to_string(r.inliers) + " inliers");
    l.tracks = tracks;
    l.depth_tracks = r.depth_tracks;
    l.inliers = r.inliers;
    return l;
  }
  fit(best, Rcp, tcp);
  double ss = 0;
  for (int j : best) {
    const Eigen::Vector3d q = Rcp * Xp[j] + tcp;
    const double du = k.fx * (q.x() / q.z() - Xc[j].x() / Xc[j].z());
    const double dv = k.fy * (q.y() / q.z() - Xc[j].y() / Xc[j].z());
    ss += du * du + dv * dv;
  }
  r.reprojection_rms = std::sqrt(ss / static_cast<double>(best.size()));
  const Eigen::Matrix3d Rpc = Rcp.transpose();
  const Eigen::Vector3d tpc = -Rpc * tcp;   // current camera in the previous frame
  const double angle = Eigen::AngleAxisd(Rpc).angle();
  if (tpc.norm() > p_.max_step_translation || angle > p_.max_step_rotation) {
    Result l = lose("step: " + std::to_string(tpc.norm()) + " m, " + std::to_string(angle) + " rad");
    l.tracks = tracks;
    l.depth_tracks = r.depth_tracks;
    l.inliers = r.inliers;
    return l;
  }
  // Chain (the chain continues from the last published pose after a loss).
  t_ = t_ + R_ * tpc;
  R_ = R_ * Rpc;
  r.R = R_;
  r.p = t_;
  if (state_ == State::Lost) {
    if (++ok_streak_ >= p_.recover_frames) {state_ = State::Ok;}
  }
  r.state = state_;
  r.publish = state_ == State::Ok;
  // Next frame: keep the tracked points, replenish.
  prev_grey_ = grey.clone();
  prev_depth_ = depth.clone();
  prev_pts_ = kept;
  if (static_cast<int>(prev_pts_.size()) < p_.min_features) {detect(grey);}
  return r;
}

}  // namespace reef_rgbd_odometry

// reef_rgbd_odometry node (P08): REPLACEMENT RGB-D visual odometry for the
// REEF chain (not a port of demo_rgbd). docs/VISION.md section 6.
//
// In:  image (sensor_msgs/Image rgb8), depth (32FC1 planar m), camera_info
// Out: cam_to_init (nav_msgs/Odometry, DEMO's camera convention: x left,
//      y up, z forward; published only while tracking is OK)
//      vo/health (diagnostic_msgs/DiagnosticArray, every processed frame)
// RGB and depth are paired by identical stamps. Test hooks (labelled, off by
// default): fault_schedule entries "delay PHASE OFFSET SPAN D" / "drop PHASE
// OFFSET SPAN": frames stamped from OFFSET to OFFSET + SPAN sim seconds after
// the start of scenario phase PHASE (/x3/scenario/phase) are held until
// stamp + D, or discarded.
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "reef_rgbd_odometry/odometry.hpp"

namespace
{

int64_t ns(const builtin_interfaces::msg::Time & t) {return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;}

struct Fault { std::string kind, phase; double offset = 0, span = 0, delay = 0; double t0 = -1; };

class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode()
  : rclcpp::Node("reef_rgbd_odometry")
  {
    reef_rgbd_odometry::Params p;
    p.max_features = declare_parameter("max_features", p.max_features);
    p.min_features = declare_parameter("min_features", p.min_features);
    p.min_depth_tracks = declare_parameter("min_depth_tracks", p.min_depth_tracks);
    p.min_inliers = declare_parameter("min_inliers", p.min_inliers);
    p.near_clip = declare_parameter("near_clip", p.near_clip);
    p.far_clip = declare_parameter("far_clip", p.far_clip);
    p.recover_frames = declare_parameter("recover_frames", p.recover_frames);
    p.ransac_distance = declare_parameter("ransac_distance", p.ransac_distance);
    p.depth_discontinuity = declare_parameter("depth_discontinuity", p.depth_discontinuity);
    if (p.max_features <= 0 || p.min_features < 0 || p.min_inliers < 4 || p.min_depth_tracks < 4 ||
      !(p.near_clip > 0) || !(p.far_clip > p.near_clip) || p.recover_frames < 1 || !(p.ransac_distance > 0) ||
      !(p.depth_discontinuity > 0))
    {
      throw std::runtime_error("invalid reef_rgbd_odometry parameters");
    }
    vo_ = std::make_unique<reef_rgbd_odometry::RgbdOdometry>(p);
    for (const auto & f : declare_parameter<std::vector<std::string>>("fault_schedule", std::vector<std::string>{})) {
      std::istringstream in(f);
      Fault x;
      in >> x.kind >> x.phase >> x.offset >> x.span;
      if (x.kind == "delay") {in >> x.delay;}
      if ((x.kind != "delay" && x.kind != "drop") || x.phase.empty() || !(x.span > 0) ||
        (x.kind == "delay" && !(x.delay > 0)))
      {
        throw std::runtime_error("invalid fault_schedule entry: " + f);
      }
      faults_.push_back(x);
      RCLCPP_WARN(get_logger(), "TEST HOOK: %s", f.c_str());
    }
    if (!faults_.empty()) {
      auto latched = rclcpp::QoS(1).reliable().transient_local();
      phase_sub_ = create_subscription<std_msgs::msg::String>("/x3/scenario/phase", latched,
          [this](const std_msgs::msg::String & m) {
            for (auto & f : faults_) {
              if (f.phase == m.data) {f.t0 = get_clock()->now().seconds() + f.offset;}
            }
          });
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("cam_to_init", rclcpp::QoS(10).reliable());
    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("vo/health", rclcpp::QoS(10).reliable());
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", qos,
        [this](const sensor_msgs::msg::CameraInfo & m) {
          k_.fx = m.k[0]; k_.fy = m.k[4]; k_.cx = m.k[2]; k_.cy = m.k[5];
          have_k_ = true;
        });
    img_sub_ = create_subscription<sensor_msgs::msg::Image>("image", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {rgb_[ns(m->header.stamp)] = m; pair();});
    dep_sub_ = create_subscription<sensor_msgs::msg::Image>("depth", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {dep_[ns(m->header.stamp)] = m; pair();});
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] {release();});
    RCLCPP_WARN(get_logger(), "REPLACEMENT RGB-D odometry (not demo_rgbd); output in DEMO's camera convention");
  }

private:
  void pair()
  {
    while (!rgb_.empty()) {
      auto it = dep_.find(rgb_.begin()->first);
      if (it == dep_.end()) {break;}
      const int64_t t = rgb_.begin()->first;
      enqueue(t, rgb_.begin()->second, it->second);
      rgb_.erase(rgb_.begin());
      dep_.erase(it);
    }
    while (rgb_.size() > 5) {rgb_.erase(rgb_.begin());}   // unpaired leftovers
    while (dep_.size() > 5) {dep_.erase(dep_.begin());}
  }

  void enqueue(int64_t t, sensor_msgs::msg::Image::ConstSharedPtr rgb, sensor_msgs::msg::Image::ConstSharedPtr dep)
  {
    const double ts = t * 1e-9;
    double hold = 0;
    for (const auto & f : faults_) {
      if (f.t0 >= 0 && ts >= f.t0 && ts < f.t0 + f.span) {
        if (f.kind == "drop") {++dropped_; return;}
        hold = std::max(hold, f.delay);
      }
    }
    queue_.push_back({t + static_cast<int64_t>(hold * 1e9), rgb, dep});
    release();
  }

  void release()
  {
    const int64_t now = get_clock()->now().nanoseconds();
    while (!queue_.empty() && queue_.front().due <= now) {
      auto f = queue_.front();
      queue_.pop_front();
      process(f.rgb, f.dep);
    }
  }

  void process(sensor_msgs::msg::Image::ConstSharedPtr rgb, sensor_msgs::msg::Image::ConstSharedPtr dep)
  {
    if (!have_k_) {return;}
    const auto w0 = std::chrono::steady_clock::now();
    cv::Mat grey;
    cv::cvtColor(cv_bridge::toCvShare(rgb, "rgb8")->image, grey, cv::COLOR_RGB2GRAY);
    const cv::Mat depth = cv_bridge::toCvShare(dep, "32FC1")->image;
    const auto r = vo_->process(grey, depth, k_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
    ++frames_;
    if (r.publish) {
      Eigen::Matrix3d Rd;
      Eigen::Vector3d pd;
      reef_rgbd_odometry::toDemo(r.R, r.p, Rd, pd);
      const Eigen::Quaterniond q(Rd);
      nav_msgs::msg::Odometry o;
      o.header.stamp = rgb->header.stamp;
      o.header.frame_id = "cam_init_demo";
      o.child_frame_id = "camera_demo";
      o.pose.pose.position.x = pd.x(); o.pose.pose.position.y = pd.y(); o.pose.pose.position.z = pd.z();
      o.pose.pose.orientation.x = q.x(); o.pose.pose.orientation.y = q.y();
      o.pose.pose.orientation.z = q.z(); o.pose.pose.orientation.w = q.w();
      odom_pub_->publish(o);
      ++published_;
    }
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = rgb->header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "reef_rgbd_odometry";
    s.hardware_id = "replacement RGB-D odometry (not demo_rgbd)";
    s.level = r.state == reef_rgbd_odometry::State::Lost ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    s.message = reef_rgbd_odometry::toString(r.state) + (r.reason.empty() ? "" : ": " + r.reason);
    auto kv = [&](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue x;
        x.key = k;
        x.value = v;
        s.values.push_back(x);
      };
    const double age = (get_clock()->now().nanoseconds() - ns(rgb->header.stamp)) * 1e-6;
    kv("state", reef_rgbd_odometry::toString(r.state));
    kv("published", r.publish ? "1" : "0");
    kv("tracks", std::to_string(r.tracks));
    kv("depth_tracks", std::to_string(r.depth_tracks));
    kv("inliers", std::to_string(r.inliers));
    kv("reprojection_rms_px", std::to_string(r.reprojection_rms));
    kv("processing_ms_wall", std::to_string(ms));
    kv("frame_age_ms_sim", std::to_string(age));
    kv("wall_time_s", std::to_string(
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()));
    kv("frames", std::to_string(frames_));
    kv("frames_published", std::to_string(published_));
    kv("frames_dropped_by_hook", std::to_string(dropped_));
    kv("lost_events", std::to_string(vo_->lostEvents()));
    a.status.push_back(s);
    health_pub_->publish(a);
  }

  struct Pending { int64_t due; sensor_msgs::msg::Image::ConstSharedPtr rgb, dep; };
  std::unique_ptr<reef_rgbd_odometry::RgbdOdometry> vo_;
  reef_rgbd_odometry::Intrinsics k_;
  bool have_k_ = false;
  std::vector<Fault> faults_;
  std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> rgb_, dep_;
  std::deque<Pending> queue_;
  long frames_ = 0, published_ = 0, dropped_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_, dep_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<OdometryNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("reef_rgbd_odometry"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

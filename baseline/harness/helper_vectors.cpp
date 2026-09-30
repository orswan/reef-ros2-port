// Records input/output vectors of the pinned legacy reef_msgs helpers
// (7fb63ff dynamics.cpp, matrix_operation.h) for the P03 port tests.
// Compiled UNMODIFIED against the P02 ROS stand-ins by
// baseline/helper_vectors.sh. Output: one case per line, all doubles as C99
// hex floats (exact), so the port test can require bit-identical results.
//
//   Q2R qx qy qz qw | C00 C01 ... C22          quaternion_to_rotation(geometry_msgs::Quaternion)
//   RPY C00 ... C22 | roll pitch yaw           roll_pitch_yaw_from_rotation321(C, r, p, y)
//   MAT name rows cols n v1..vn | S m00 ...    importMatrixFromParamServer; S = F full, D diagonal,
//                                              Z missing -> zero-filled, U wrong size -> left unchanged
//                                              (matrix pre-filled with -1.5 to make "unchanged" visible)
//   ARR rows cols m00 ... | a0 a1 ...          matrixToArray (row-major)
#include <ros/ros.h>
#include <reef_msgs/dynamics.h>
#include <reef_msgs/matrix_operation.h>

#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace ros {
std::map<std::string, ParamValue>& param_store() { static std::map<std::string, ParamValue> s; return s; }
std::map<std::string, long>& publish_counts() { static std::map<std::string, long> c; return c; }
bool& log_enabled() { static bool on = false; return on; }
}  // namespace ros

static FILE* out = stdout;

static void put(double v) { std::fprintf(out, " %a", v); }

static void q2r(double x, double y, double z, double w) {
  geometry_msgs::Quaternion q;
  q.x = x; q.y = y; q.z = z; q.w = w;
  Eigen::Matrix3d C = reef_msgs::quaternion_to_rotation(q);
  std::fprintf(out, "Q2R");
  put(x); put(y); put(z); put(w);
  std::fprintf(out, " |");
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) put(C(i, j));
  std::fprintf(out, "\n");
}

static void rpy(const Eigen::Matrix3d& C) {
  double r, p, y;
  reef_msgs::roll_pitch_yaw_from_rotation321(C, r, p, y);
  std::fprintf(out, "RPY");
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) put(C(i, j));
  std::fprintf(out, " |");
  put(r); put(p); put(y);
  std::fprintf(out, "\n");
}

static void mat(const std::string& name, int rows, int cols, const std::vector<double>* vals) {
  ros::param_store().clear();
  if (vals) {
    ros::ParamValue pv;
    pv.kind = ros::ParamValue::List;
    pv.list = *vals;
    ros::param_store()[name] = pv;
  }
  Eigen::MatrixXd m = Eigen::MatrixXd::Constant(rows, cols, -1.5);
  ros::NodeHandle nh("~");
  reef_msgs::importMatrixFromParamServer(nh, m, name);
  char status;
  if (!vals) status = 'Z';
  else if (vals->size() == static_cast<size_t>(rows * cols)) status = 'F';
  else if (vals->size() == static_cast<size_t>(rows)) status = 'D';
  else status = 'U';
  std::fprintf(out, "MAT %s %d %d %zu", name.c_str(), rows, cols, vals ? vals->size() : 0);
  if (vals) for (double v : *vals) put(v);
  std::fprintf(out, " | %c", status);
  for (int i = 0; i < rows; i++) for (int j = 0; j < cols; j++) put(m(i, j));
  std::fprintf(out, "\n");
}

int main(int argc, char** argv) {
  if (argc > 1 && !(out = std::fopen(argv[1], "w"))) { std::perror(argv[1]); return 2; }
  std::fprintf(out, "# legacy reef_msgs 7fb63ff helper vectors (baseline/harness/helper_vectors.cpp)\n");

  std::mt19937_64 rng(20260930);
  std::uniform_real_distribution<double> u(-1.0, 1.0), scale(0.25, 4.0), small(-0.3, 0.3);
  std::normal_distribution<double> n01(0.0, 1.0);
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Matrix3d> rotations;

  // quaternion_to_rotation: special cases, then random unit, IMU-like, and non-unit.
  const double h = std::sqrt(0.5);
  const double specials[][4] = {
    {0, 0, 0, 1}, {0, 0, 0, -1}, {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0},
    {h, 0, 0, h}, {0, h, 0, h}, {0, 0, h, h}, {-h, 0, 0, h}, {0, -h, 0, h},
    {0.5, 0.5, 0.5, 0.5}, {-0.0, -0.0, -0.0, 1}, {0, 0, 0, 0}, {1e-300, 0, 0, 1},
    {1e200, 0, 0, 1}, {nan, 0, 0, 1}, {0, 0, 0, nan}, {inf, 0, 0, 1}, {0, 0, 0, inf},
    {0.1, 0.2, 0.3, 0.4},
  };
  for (const auto& s : specials) q2r(s[0], s[1], s[2], s[3]);
  for (int k = 0; k < 1500; k++) {
    Eigen::Quaterniond q;
    if (k < 700) {           // uniform random unit quaternion
      q = Eigen::Quaterniond(n01(rng), n01(rng), n01(rng), n01(rng)).normalized();
    } else if (k < 1200) {   // IMU-like: small roll/pitch, any yaw
      q = Eigen::AngleAxisd(3.2 * u(rng), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(small(rng), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(small(rng), Eigen::Vector3d::UnitX());
    } else {                 // not normalized (the helper does not normalize)
      q = Eigen::Quaterniond(u(rng), u(rng), u(rng), u(rng));
      q.coeffs() *= scale(rng);
    }
    q2r(q.x(), q.y(), q.z(), q.w());
    geometry_msgs::Quaternion gq;
    gq.x = q.x(); gq.y = q.y(); gq.z = q.z(); gq.w = q.w();
    if (k < 1200) rotations.push_back(reef_msgs::quaternion_to_rotation(gq));
  }

  // roll_pitch_yaw_from_rotation321: rotations above, then edge cases.
  for (const auto& C : rotations) rpy(C);
  const double edge02[] = {1.0, -1.0, 1.0 + 1e-15, -1.0 - 1e-15, 0.0, -0.0, nan, inf};
  for (double c02 : edge02) {
    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
    C(0, 2) = c02;
    rpy(C);
  }
  const double zeros[][4] = {{0.0, 0.0, 0.0, 0.0}, {-0.0, -0.0, -0.0, -0.0},
                             {0.0, -0.0, -0.0, 0.0}, {-0.0, 0.0, 0.0, -0.0}};
  for (const auto& z : zeros) {   // atan2 of signed zeros
    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
    C(1, 2) = z[0]; C(2, 2) = z[1]; C(0, 1) = z[2]; C(0, 0) = z[3];
    rpy(C);
  }
  for (int k = 0; k < 100; k++) {   // arbitrary (non-orthogonal) matrices
    Eigen::Matrix3d C;
    for (int i = 0; i < 9; i++) C(i / 3, i % 3) = 1.2 * u(rng);
    rpy(C);
  }

  // importMatrixFromParamServer: the shapes reef_estimator master reads.
  struct Shape { const char* name; int rows, cols; };
  const Shape shapes[] = {
    {"xy_x0", 6, 1}, {"xy_P0", 6, 6}, {"xy_Q", 6, 6}, {"xy_R0", 2, 2}, {"xy_beta", 6, 1},
    {"z_x0", 3, 1}, {"z_P0", 3, 3}, {"z_P0_flying", 3, 3}, {"z_Q", 2, 2},
    {"z_R0", 1, 1}, {"z_R_flying", 1, 1}, {"z_beta", 3, 1},
  };
  for (const auto& s : shapes) {
    const int full = s.rows * s.cols;
    for (int rep = 0; rep < 3; rep++) {
      std::vector<double> v(full);
      for (double& x : v) x = rep == 0 ? 0.0 : u(rng) * std::pow(10.0, 3 * u(rng));
      mat(s.name, s.rows, s.cols, &v);                                  // full
      if (s.rows == s.cols && s.rows > 1) {
        std::vector<double> d(s.rows);
        for (double& x : d) x = std::fabs(u(rng));
        mat(s.name, s.rows, s.cols, &d);                                // diagonal
      }
    }
    mat(s.name, s.rows, s.cols, nullptr);                               // missing
    std::vector<double> wrong(full + 1, 0.5);
    mat(s.name, s.rows, s.cols, &wrong);                                // wrong size
  }

  // matrixToArray, as used for ZDebugEstimate.P (3x3).
  for (int k = 0; k < 20; k++) {
    Eigen::MatrixXd P(3, 3);
    for (int i = 0; i < 9; i++) P(i / 3, i % 3) = u(rng);
    boost::array<double, 9> a;
    a.fill(0);
    reef_msgs::matrixToArray(P, a);
    std::fprintf(out, "ARR 3 3");
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) put(P(i, j));
    std::fprintf(out, " |");
    for (double x : a) put(x);
    std::fprintf(out, "\n");
  }
  if (out != stdout) std::fclose(out);
  return 0;
}

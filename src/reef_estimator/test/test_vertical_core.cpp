// XYZEstimator unit behaviour without ROS. Fidelity against the
// original is checked separately (baseline/tools/check_vertical.py); these
// tests pin the documented behaviour of individual steps.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "reef_estimator/xyz_estimator.h"

using namespace reef_estimator;

namespace
{

EstimatorParameters master()
{
  EstimatorParameters p;
  p.xy_x0.setZero();
  p.xy_P0 = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  p.xy_Q.setZero();
  p.xy_R0 = Eigen::Matrix2d::Identity() * 0.02;
  p.xy_beta.setOnes();
  p.z_x0 << -0.25, 0, 0;
  p.z_P0 = Eigen::Vector3d(0.025, 1.0, 0.01).asDiagonal();
  p.z_P0_flying = Eigen::Vector3d(0.25, 1.0, 0.01).asDiagonal();
  p.z_Q = Eigen::Vector2d(0.03, 0.0001).asDiagonal();
  p.z_R0 << 0.04;
  p.z_R_flying << 0.00016;
  p.z_beta << 1, 1, 0.5;
  p.enable_rgbd = false;
  return p;
}

ImuSample level(long long t_ns, double az = -9.81)
{
  ImuSample s;
  s.stamp.sec = static_cast<int32_t>(t_ns / 1000000000LL);
  s.stamp.nanosec = static_cast<uint32_t>(t_ns % 1000000000LL);
  s.az = az;
  return s;
}

RangeSample range(long long t_ns, float r, float max_range = 7.65f)
{
  RangeSample s;
  s.stamp = level(t_ns).stamp;
  s.range = r;
  s.max_range = max_range;
  return s;
}

constexpr long long T0 = 1000000000LL, DT = 2000000LL;

// Feeds n level IMU samples starting at index k0; returns the next index.
int feed(XYZEstimator & e, int k0, int n)
{
  for (int k = k0; k < k0 + n; k++) {e.sensorUpdate(level(T0 + k * DT));}
  return k0 + n;
}

}  // namespace

TEST(VerticalCore, StartsAtInitialStateWithNominalDt)
{
  XYZEstimator e(master());
  const ZEstimator & z = e.zFilter();
  EXPECT_EQ(z.xHat(0, 0), -0.25);
  EXPECT_EQ(z.P(0, 0), 0.025);
  EXPECT_EQ(z.R(0, 0), 0.04);
  EXPECT_EQ(z.dt, 0.002);
  EXPECT_EQ(z.Q(0, 0), 0.03 * 0.002);   // Q = z_Q * dt
  EXPECT_FALSE(e.isFlying());
  EXPECT_TRUE(std::isnan(e.lastMahalanobisSquared()));
}

TEST(VerticalCore, TwentySamplesInitializeThenEveryImuProducesAnEstimate)
{
  XYZEstimator e(master());
  for (int k = 0; k < 20; k++) {EXPECT_FALSE(e.sensorUpdate(level(T0 + k * DT))) << k;}
  EXPECT_TRUE(e.accelerometerInitialized());
  EXPECT_EQ(e.initialGravity(), 9.81);   // hard-coded in the original (C3)
  EXPECT_TRUE(e.sensorUpdate(level(T0 + 20 * DT)));
  EXPECT_EQ(e.estimateCount(), 1);
  // dt = toSec(t20) - toSec(t19) with ROS 1's formula (not exactly 0.002).
  EXPECT_EQ(e.zFilter().dt, level(T0 + 20 * DT).stamp.toSec() - level(T0 + 19 * DT).stamp.toSec());
  EXPECT_EQ(e.stamp().nanosec, 40000000u);
}

TEST(VerticalCore, DtUsesTheRos1SecondsFormula)
{
  Stamp s;
  s.sec = 12;
  s.nanosec = 345678901u;
  EXPECT_EQ(s.toSec(), static_cast<double>(12) + 1e-9 * static_cast<double>(345678901u));
}

TEST(VerticalCore, NanSampleIsDroppedAndTheNextDtSpansTheGap)
{
  XYZEstimator e(master());
  int k = feed(e, 0, 25);
  ImuSample bad = level(T0 + k * DT);
  bad.ax = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(e.sensorUpdate(bad));
  k++;
  EXPECT_TRUE(e.sensorUpdate(level(T0 + k * DT)));
  EXPECT_NEAR(e.zFilter().dt, 0.004, 1e-15);   // D9
}

TEST(VerticalCore, RangeAboveMaxOrNanIsIgnoredWithoutAGate)
{
  XYZEstimator e(master());
  feed(e, 0, 25);
  e.sensorUpdate(range(T0, 8.0f));
  e.sensorUpdate(range(T0, std::numeric_limits<float>::quiet_NaN()));
  EXPECT_EQ(e.zGateCount(), 0);
  EXPECT_FALSE(e.pendingZMeasurement());
}

TEST(VerticalCore, AcceptedRangeIsStoredNegatedAndFusedAtTheNextImu)
{
  XYZEstimator e(master());
  int k = feed(e, 0, 25);
  e.sensorUpdate(range(T0 + k * DT, 0.3f));
  EXPECT_EQ(e.zGateCount(), 1);
  EXPECT_TRUE(e.pendingZMeasurement());
  EXPECT_EQ(e.zFilter().z(0), static_cast<double>(-0.3f));   // float32 negation, as in ROS 1
  feed(e, k, 1);
  EXPECT_FALSE(e.pendingZMeasurement());
}

TEST(VerticalCore, OutlierIsRejectedByTheGate)
{
  XYZEstimator e(master());
  int k = feed(e, 0, 25);
  e.sensorUpdate(range(T0 + k * DT, 5.0f));
  EXPECT_EQ(e.zGateCount(), 1);
  EXPECT_GT(e.lastMahalanobisSquared(), 20.0);
  EXPECT_FALSE(e.pendingZMeasurement());
}

TEST(VerticalCore, DisabledZKeepsTheMeasurementPending)
{
  EstimatorParameters p = master();
  p.enable_z = false;
  XYZEstimator e(p);
  int k = feed(e, 0, 25);
  e.sensorUpdate(range(T0 + k * DT, 0.3f));
  feed(e, k, 5);
  EXPECT_TRUE(e.pendingZMeasurement());   // never cleared without the update
}

TEST(VerticalCore, LandingResetRestoresCovarianceEveryTenPropagations)
{
  XYZEstimator e(master());
  feed(e, 0, 20);
  feed(e, 20, 9);
  EXPECT_EQ(e.propagationCount(), 9);
  EXPECT_NE(e.zFilter().P(1, 1), 1.0);
  feed(e, 29, 1);
  EXPECT_EQ(e.propagationCount(), 0);
  EXPECT_EQ(e.zFilter().P(1, 1), 1.0);
}

TEST(VerticalCore, TakeoffNeedsAccelVarianceAndAltitude)
{
  XYZEstimator e(master());
  int k = feed(e, 0, 20);
  for (int n = 0; n < 60; n++, k++) {
    if (n % 10 == 0) {e.sensorUpdate(range(T0 + k * DT, 0.3f));}
    e.sensorUpdate(level(T0 + k * DT, -9.81 + ((k % 2) ? 1.5 : -1.5)));
  }
  EXPECT_TRUE(e.isFlying());
  EXPECT_EQ(e.takeoffTransitions(), 1);
  EXPECT_EQ(e.zFilter().R(0, 0), 0.00016);
}

TEST(VerticalCore, RcSwitchSelectsMocapZOnlyWhenEnabled)
{
  EstimatorParameters p = master();
  p.enable_mocap_switch = true;
  p.enable_mocap_z = true;
  p.mocap_override_channel = 6;
  XYZEstimator e(p);
  EXPECT_TRUE(e.subscribesRc());
  EXPECT_FALSE(e.usingMocapZ());   // sonar enabled: mocap z starts off
  RcSample rc;
  rc.values[6] = 1500;
  e.rcRawUpdate(rc);
  EXPECT_FALSE(e.usingMocapZ());   // > 1500 is required
  rc.values[6] = 1501;
  e.rcRawUpdate(rc);
  EXPECT_TRUE(e.usingMocapZ());
  rc.values[6] = 1000;
  e.rcRawUpdate(rc);
  EXPECT_FALSE(e.usingMocapZ());
}

TEST(VerticalCore, MocapZIsIgnoredUnlessSelected)
{
  EstimatorParameters p = master();
  p.enable_mocap_z = true;   // sonar also enabled: mocap not selected
  XYZEstimator e(p);
  feed(e, 0, 25);
  MocapPoseSample m;
  m.z = -0.3;
  e.mocapUpdate(m);
  EXPECT_EQ(e.zGateCount(), 0);
  EXPECT_FALSE(e.subscribesRc());
}

//
// Created by humberto on 6/4/18.
//
// ROS 2 port (P04 vertical, P05 combined): XYZEstimator of reef_estimator
// master e4179f48 plus the RC switch of SensorManager, without ROS. Messages
// are replaced by plain structs that keep the ROS 1 field types (float32
// range, sec/nanosec stamps), so the arithmetic is that of the original.
// See docs/INTERFACES.md section 3.
#ifndef REEF_ESTIMATOR_XYZ_ESTIMATOR_H
#define REEF_ESTIMATOR_XYZ_ESTIMATOR_H

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "reef_estimator/parameters.hpp"
#include "reef_estimator/xy_estimator.h"
#include "reef_estimator/z_estimator.h"

#define ACC_SAMPLE_SIZE 20
#define ACC_TAKEOFF_VARIANCE 0.5

namespace reef_estimator
{
    // Message stamp. toSec() is ROS 1's ros::Time::toSec(), which the
    // original used for dt (clock policy, INTERFACES.md section 3.4).
    struct Stamp
    {
        int32_t sec = 0;
        uint32_t nanosec = 0;
        double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nanosec); }
    };

    struct ImuSample            // sensor_msgs/Imu: specific force (FRD), attitude quaternion
    {
        Stamp stamp;
        double ax = 0, ay = 0, az = 0;
        double qx = 0, qy = 0, qz = 0, qw = 1;
    };

    struct RangeSample          // sensor_msgs/Range: float32 fields
    {
        Stamp stamp;
        float range = 0;
        float max_range = 0;
    };

    struct MocapPoseSample      // geometry_msgs/PoseStamped: pose.position.z (NED)
    {
        Stamp stamp;
        double z = 0;
    };

    // geometry_msgs/TwistWithCovarianceStamped (mocap velocity) or the vel
    // field of reef_msgs/DeltaToVel (RGB-D): body-level linear x/y and the
    // covariance entries [0] and [7].
    struct TwistSample
    {
        Stamp stamp;
        double vx = 0, vy = 0;
        double cov_xx = 0, cov_yy = 0;
    };

    struct RcSample             // rosflight_msgs/RCRaw
    {
        Stamp stamp;
        std::array<uint16_t, 8> values{};
    };

    struct ZState               // one ZDebugEstimate worth of state
    {
        double z = 0, z_dot = 0, bias = 0, u = 0;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    };

    struct XYState              // one XYDebugEstimate worth of state
    {
        Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();   // x_dot, y_dot, pitch_bias, roll_bias, xa_bias, ya_bias
        Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
    };

    enum class LogLevel { Info, Warn, Error };

    class XYZEstimator
    {
    public:
        explicit XYZEstimator(const EstimatorParameters& params);

        // Returns true when an estimate was produced (the original published
        // xyz_estimate); false while initializing or for a NaN sample.
        bool sensorUpdate(const ImuSample& imu);
        void sensorUpdate(const RangeSample& range_msg);
        void mocapUpdate(const MocapPoseSample& pose_msg);
        void mocapUpdate(const TwistSample& twist_msg);
        // RGB-D velocity. The caller applies SensorManager's enable_measurements
        // switch (a runtime parameter) before calling.
        void rgbdUpdate(const TwistSample& twist_msg);
        void rcRawUpdate(const RcSample& msg);

        // Which inputs the original node subscribed to (SensorManager constructor).
        bool subscribesRange() const { return enableSonar; }
        bool subscribesMocapPose() const { return enableMocapZ; }
        bool subscribesMocapTwist() const { return enableMocapXY; }
        bool subscribesRgbd() const { return enableRGBD; }
        bool subscribesRc() const { return enableMocapSwitch; }

        // Latest estimate (valid after sensorUpdate(imu) returned true).
        Stamp stamp() const { return stamp_; }
        ZState plusState() const;
        const ZState& minusState() const { return zMinus; }   // only with debug_mode
        XYState xyPlusState() const;
        const XYState& xyMinusState() const { return xyMinus; }   // only with debug_mode
        bool debugMode() const { return debug_mode_; }

        // Takeoff state and its changes (is_flying_reef is published on change).
        bool isFlying() const { return takeoffState; }
        long takeoffTransitions() const { return numTakeoffTransitions; }

        // Introspection for the fidelity comparison (names as in the harness CSV).
        const ZEstimator& zFilter() const { return zEst; }
        const XYEstimator& xyFilter() const { return xyEst; }
        bool accelerometerInitialized() const { return accInitialized; }
        double initialGravity() const { return initialAccMagnitude; }
        bool pendingZMeasurement() const { return newSonarMeasurement; }
        bool pendingXYMeasurement() const { return newRgbdMeasurement; }
        int propagationCount() const { return numberOfPropagations; }
        double lastMahalanobisSquared() const { return Mahalanobis_D_hat_square(0); }
        long zGateCount() const { return numZGates; }
        long xyGateCount() const { return numXYGates; }
        long estimateCount() const { return numEstimates; }
        bool usingMocapZ() const { return useMocapZ; }
        bool usingMocapXY() const { return useMocapXY; }

        // Observation accounting (D1): accepted XY observations and XY
        // filter updates. With the original partial-update path an accepted
        // observation is fused again at every IMU step until the next one
        // arrives (D1); correction C1 (opt-in, NOT APPROVED) fuses it once.
        long xyObservationsAccepted() const { return numXYAccepted; }
        long xyFusions() const { return numXYFusions; }
        bool correctionC1() const { return correction_c1; }

        // Optional log sink (the original used ROS_INFO/WARN/ERROR).
        std::function<void(LogLevel, const std::string&)> log;

    private:
        //Estimator enable/disable variables
        bool enableXY;
        bool enableZ;

        //Instantiate a ZEstimator
        ZEstimator zEst;

        //Instantiate an XYEstimator
        XYEstimator xyEst;

        int numberOfPropagations;

        //Accelerometer calibration variables
        bool accInitialized;
        int accInitSampleCount;
        double initialAccMagnitude;
        Eigen::Vector3d accSampleAverage;
        double last_time_stamp;
        double mahalanobis_distance_sonar;
        double mahalanobis_distance_rgbd_xy_;
        double mahalanobis_distance_mocap_z;
        double mahalanobis_distance_mocap_xy_;
        double dt;
        bool enable_partial_update;
        bool debug_mode_;
        bool correction_c1;

        //Mocap override settings (SensorManager)
        bool enableMocapXY, enableMocapZ;
        bool enableRGBD, enableSonar;
        bool useMocapXY, useMocapZ;
        bool enableMocapSwitch;
        int mocapOverrideChannel;
        bool mocapSwitchOn;

        //Takeoff detection variables
        bool takeoffState;
        bool accTakeoffState, sonarTakeoffState;
        int numAccSamples;
        double accSamples[ACC_SAMPLE_SIZE];
        double accMean, accVariance;
        bool newRgbdMeasurement, newSonarMeasurement;

        Eigen::Matrix3d C_NED_to_body_frame;
        Eigen::Vector3d accelxyz_in_body_frame ;
        Eigen::Vector3d accelxyz_in_NED_frame ;
        Eigen::VectorXd range;
        Eigen::VectorXd expected_measurement;
        Eigen::Vector3d Mahalanobis_D_hat_square;
        Eigen::Vector3d Mahalanobis_D_hat;
        Eigen::Vector2d measurement;
        Eigen::Vector2d expected_rgbd;

        Stamp stamp_;
        ZState zMinus;
        XYState xyMinus;
        long numZGates = 0;
        long numXYGates = 0;
        long numEstimates = 0;
        long numTakeoffTransitions = 0;
        long numXYAccepted = 0;
        long numXYFusions = 0;

        void checkTakeoffState(double accMagnitude);
        void saveMinusState();
        void initializeAcc(const ImuSample& imu);
        bool chi2Accept(float range_measurement);
        bool chi2AcceptRgbd(const TwistSample& twist_msg);
        bool chi2AcceptMocapXY(const TwistSample& twist_msg);
        bool chi2AcceptMocapZ(float z_mocap_ned);
        void report(LogLevel level, const std::string& text) const;
    };

    double getVectorMagnitude(double x, double y, double z);
}

#endif //REEF_ESTIMATOR_XYZ_ESTIMATOR_H

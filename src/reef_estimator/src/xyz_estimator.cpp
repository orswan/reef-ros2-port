//
// Created by humberto on 6/4/18.
//
// ROS 2 port (P04 vertical, P05 combined): XYZEstimator and the RC switch of
// SensorManager (reef_estimator master e4179f48). Statement order, types
// (including the float32 range, mocap-z gate, and z-bias arguments), and Eigen
// expressions are kept from the original so that results are identical.
// Removed: ROS parameters (see parameters.hpp) and publishers (see
// sensor_manager.cpp). Added: observation counters, and correction C1
// (approved at R1, default on; off reproduces master; BASELINE_DECISION.md 7).

#include "reef_estimator/xyz_estimator.h"

#include <cmath>
#include <limits>
#include <sstream>

#include "reef_msgs/dynamics.h"

namespace reef_estimator
{
    XYZEstimator::XYZEstimator(const EstimatorParameters& params) :
    numberOfPropagations(0),
    accInitialized(false),
    accInitSampleCount(0),
    initialAccMagnitude(0),
    last_time_stamp(0),
    mocapSwitchOn(false),
    takeoffState(false),
    accTakeoffState(false),
    sonarTakeoffState(false),
    numAccSamples(0),
    accMean(0),
    accVariance(0),
    newRgbdMeasurement(false),
    newSonarMeasurement(false)
    {
        debug_mode_ = params.debug_mode;
        correction_c1 = params.correction_c1;
        enableXY = params.enable_xy;
        enableZ = params.enable_z;

        //Mocap override settings
        enableMocapXY = params.enable_mocap_xy;
        enableRGBD = params.enable_rgbd;
        useMocapXY = enableMocapXY && !enableRGBD;

        enableMocapZ = params.enable_mocap_z;
        enableSonar = params.enable_sonar;
        useMocapZ = enableMocapZ && !enableSonar;

        enableMocapSwitch = params.enable_mocap_switch;
        mocapOverrideChannel = params.mocap_override_channel;

        mahalanobis_distance_sonar = params.mahalanobis_d_sonar;
        mahalanobis_distance_rgbd_xy_ = params.mahalanobis_d_rgbd_velocity;
        mahalanobis_distance_mocap_z = params.mahalanobis_d_mocap_z;
        mahalanobis_distance_mocap_xy_ = params.mahalanobis_d_mocap_velocity;
        enable_partial_update = params.enable_partial_update;

        // Initialize dt
        dt = params.estimator_dt;
        xyEst.dt = zEst.dt = dt;

        //Initialize estimators with parameters (validated by validate())
        xyEst.xHat0 = params.xy_x0;
        xyEst.P0 = params.xy_P0;
        xyEst.Q = params.xy_Q;
        xyEst.R0 = params.xy_R0;
        xyEst.betaVector = params.xy_beta;
        xyEst.Q *= (xyEst.dt*xyEst.dt);

        xyEst.initialize();//Initialize P,R and beta.

        zEst.xHat0 = params.z_x0;
        zEst.P0 = params.z_P0;
        zEst.P0forFlying = params.z_P0_flying;
        zEst.Q0 = params.z_Q;
        zEst.R0 = params.z_R0;
        zEst.RforFlying = params.z_R_flying;
        zEst.betaVector = params.z_beta;
        zEst.Q = zEst.Q0*(zEst.dt);
        zEst.updateLinearModel();
        zEst.initialize();
        zEst.setTakeoffState(false);

        //Initialize member variables
        accSampleAverage.setZero();
        // The original left this uninitialized until the first gate; NaN
        // marks "no gate evaluated yet" (as the P02 harness does, A3).
        Mahalanobis_D_hat_square.setConstant(std::numeric_limits<double>::quiet_NaN());
        Mahalanobis_D_hat.setConstant(std::numeric_limits<double>::quiet_NaN());
        for (double& s : accSamples) s = 0;
    }

    void XYZEstimator::report(LogLevel level, const std::string& text) const
    {
        if (log) log(level, text);
    }

/** This function is used to calculate the mean accelerometer bias . */
    void XYZEstimator::initializeAcc(const ImuSample& imu)
    {
        //Sum ACC_SAMPLE_SIZE accelerometer readings
        accSampleAverage(0) += imu.ax;
        accSampleAverage(1) += imu.ay;
        accSampleAverage(2) += imu.az;
        accInitSampleCount++;

        if (accInitSampleCount == ACC_SAMPLE_SIZE)
        {
            accSampleAverage(0) /= ACC_SAMPLE_SIZE;
            accSampleAverage(1) /= ACC_SAMPLE_SIZE;
            accSampleAverage(2) /= ACC_SAMPLE_SIZE;

            initialAccMagnitude = getVectorMagnitude(accSampleAverage(0), accSampleAverage(1), accSampleAverage(2));
            initialAccMagnitude = 9.81;
            std::ostringstream o;
            o << "initial g: " << initialAccMagnitude;
            report(LogLevel::Info, o.str());

            accInitialized = true;
        }
    }
/** Sensor update for the IMU. */
    bool XYZEstimator::sensorUpdate(const ImuSample& imu)
    {
        //Save the stamp. This is very important for good book-keeping.
        stamp_ = imu.stamp;

        if (std::isnan(getVectorMagnitude(imu.ax, imu.ay, imu.az))){
            report(LogLevel::Error, "IMU is giving NaNs");
            return false;
        }

        //Make sure accelerometer is initialized
        if (!accInitialized) {
            initializeAcc(imu);
            last_time_stamp = imu.stamp.toSec();
            return false;
        }
        // Compute new DT and pass it to estimators.
        xyEst.dt = imu.stamp.toSec() - last_time_stamp;
        zEst.dt = imu.stamp.toSec() - last_time_stamp;
        last_time_stamp = imu.stamp.toSec();

        Eigen::Quaterniond q_temp;
        q_temp.x() = imu.qx;
        q_temp.y() = imu.qy;
        q_temp.z() = imu.qz;
        q_temp.w() = imu.qw;
        C_NED_to_body_frame = reef_msgs::quaternion_to_rotation(q_temp);
        //TODO incorporate rotation into range measurement.
        //zEst.H(0,0) = (-1/C_NED_to_body_frame(2,2));

        //Transform accel measurements from body to NED frame.
        accelxyz_in_body_frame << imu.ax, imu.ay, imu.az; //This is a column vector.
        accelxyz_in_NED_frame = C_NED_to_body_frame.transpose() * accelxyz_in_body_frame;

        /*The specific force model is the following s = a_measured - bias_accel - noise_accel - gravity
         * The bias that is being estimated is in the inertial frame (NED)
         * */
        zEst.u(0) = accelxyz_in_NED_frame(2) + initialAccMagnitude; //We need to take avg from csv file to get a better g.

        //Finally propagate.
        xyEst.nonlinearPropagation(C_NED_to_body_frame, initialAccMagnitude, accelxyz_in_body_frame, zEst.xHat(2));
        zEst.updateLinearModel();
        zEst.propagate();

        //Z estimator publisher block------------------------------------------
        if (debug_mode_)
        {
            saveMinusState();
        }

        /*We let the previous block to keep running for approximately 0.1 seconds,
            * if we haven't taken off, we reset the covariance P to its initial value and keep R big enough to
            * prevent our covariance from totally shrinking
           */

        numberOfPropagations++;
        //Reset the z estimator to its landing state every 10 propagations
        if (!takeoffState && numberOfPropagations >= 10)
        {
            if (enableXY)
                xyEst.resetLandingState();
            
            if (enableZ)
                zEst.resetLandingState();

            //Reset number of propagations
            numberOfPropagations = 0;
        }

        if (enableXY && newRgbdMeasurement) {
            if(enable_partial_update) {
                xyEst.partialUpdate();
                numXYFusions++;
                if (correction_c1)
                    newRgbdMeasurement = false;   // C1 (approved at R1): fuse each observation once
            }
                else{
                    xyEst.update();
                    numXYFusions++;
                    newRgbdMeasurement = false;
                }
            }

        if (enableZ && newSonarMeasurement) {
            //TODO adjust estimator to perform partialUpdate on z as well.
            if(enable_partial_update)
                zEst.partialUpdate();
            else
                zEst.update();
            newSonarMeasurement = false;
        }

        // publishEstimates(): the published values are taken here, before the
        // takeoff check below can change the filters (R1 finding 1).
        zPublished = plusState();
        xyPublished = xyPlusState();
        numEstimates++;

        checkTakeoffState(accelxyz_in_body_frame.norm());
        return true;
    }

    void XYZEstimator::rgbdUpdate(const TwistSample& twist_msg)
    {
        if (!useMocapXY)
        {
            if (chi2AcceptRgbd(twist_msg))
            {
                xyEst.R(0, 0) = twist_msg.cov_xx;
                xyEst.R(1, 1) = twist_msg.cov_yy;
                xyEst.z(0) = twist_msg.vx;
                xyEst.z(1) = twist_msg.vy;
                newRgbdMeasurement = true;
                numXYAccepted++;
            }
        }
    }

    //Sonar update
    void XYZEstimator::sensorUpdate(const RangeSample& range_msg)
    {
        if (!useMocapZ)
        {
            if (range_msg.range <= range_msg.max_range)
            {
                if (chi2Accept(range_msg.range))
                {
                    zEst.z(0) = -range_msg.range; // negative size to convert to NED
                    newSonarMeasurement = true;
                }
            }
        }
    }

    //Mocap XY update
    void XYZEstimator::mocapUpdate(const TwistSample& twist_msg)
    {
        if (useMocapXY)
        {
            if (chi2AcceptMocapXY(twist_msg))
            {
                //z is the measurement.
                xyEst.R(0, 0) = twist_msg.cov_xx;
                xyEst.R(1, 1) = twist_msg.cov_yy;
                xyEst.z(0) = twist_msg.vx;
                xyEst.z(1) = twist_msg.vy;
                newRgbdMeasurement = true;
                numXYAccepted++;
            }
        }
    }

    //Mocap Z update
    void XYZEstimator::mocapUpdate(const MocapPoseSample& pose_msg)
    {

        if (useMocapZ)
        {
            if (chi2AcceptMocapZ(pose_msg.z))
            {
                zEst.z(0) = pose_msg.z;
                newSonarMeasurement = true;
            }
        }
    }

    // SensorManager::rcRawCallback
    void XYZEstimator::rcRawUpdate(const RcSample& msg) {
        //Check for toggled mocap RC switch
        if (!mocapSwitchOn && msg.values[mocapOverrideChannel] > 1500)
        {
            if (enableMocapXY)
            {
                useMocapXY = true;
                report(LogLevel::Warn, "Mocap XY feedback enabled");
            }

            if (enableMocapZ)
            {
                useMocapZ = true;
                report(LogLevel::Warn, "Mocap Z feedback enabled");
            }

            mocapSwitchOn = true;
        }
        else if (mocapSwitchOn && msg.values[mocapOverrideChannel] <= 1500)
        {
            if (enableRGBD)
            {
                useMocapXY = false;
                report(LogLevel::Warn, "Mocap XY feedback disabled");
            }

            if (enableSonar)
            {
                useMocapZ =  false;
                report(LogLevel::Warn, "Mocap Z feedback disabled");
            }

            mocapSwitchOn = false;
        }
    }

    bool XYZEstimator::chi2Accept(float range_measurement)
    {
        numZGates++;
        //Compute Mahalanobis distance.
        range = Eigen::VectorXd(1);
        range(0) = -range_measurement;
        expected_measurement = Eigen::VectorXd(1);
        expected_measurement = zEst.H * zEst.xHat;

        Eigen::MatrixXd S(1, 1);
        S = zEst.H * zEst.P * zEst.H.transpose() + zEst.R;

        Mahalanobis_D_hat_square(0) = (range - expected_measurement).transpose() * S.inverse() * (range - expected_measurement);
        Mahalanobis_D_hat(0) = sqrt(Mahalanobis_D_hat_square(0));

        //Value for 99% we need 6.63.
        //Value for 95% we 3.84
        if (Mahalanobis_D_hat_square(0) > mahalanobis_distance_sonar)
        {
            report(LogLevel::Info, "Range measurement rejected");
            return false;
        }
        else
        {
            return true;
        }
    }

    bool XYZEstimator::chi2AcceptRgbd(const TwistSample& twist_msg) 
    {
        numXYGates++;

        //Compute Mahalanobis distance.
       measurement << twist_msg.vx, twist_msg.vy;
       expected_rgbd = xyEst.H * xyEst.xHat;

        Eigen::MatrixXd S(1, 1);
        S = xyEst.H * xyEst.P * xyEst.H.transpose() + xyEst.R;

        Mahalanobis_D_hat_square(0) = (measurement - expected_rgbd).transpose() * S.inverse() * (measurement - expected_rgbd);
        Mahalanobis_D_hat(0) = sqrt(Mahalanobis_D_hat_square(0));

        //Value for 99% we need 6.63.
        //Value for 95% we 3.84
        if (Mahalanobis_D_hat_square(0) > mahalanobis_distance_rgbd_xy_)
        {
            report(LogLevel::Info, "RGBD measurement rejected");
            return false;
        } 
        else 
        {
            return true;
        }
    }

    bool XYZEstimator::chi2AcceptMocapZ(float z_mocap_ned)
    {
        numZGates++;
        //Compute Mahalanobis distance.
        range = Eigen::VectorXd(1);
        range(0) = z_mocap_ned;
        expected_measurement = Eigen::VectorXd(1);
        expected_measurement = zEst.H * zEst.xHat;

        Eigen::MatrixXd S(1, 1);
        S = zEst.H * zEst.P * zEst.H.transpose() + zEst.R;

        Mahalanobis_D_hat_square(0) =
                (range - expected_measurement).transpose() * S.inverse() * (range - expected_measurement);
        Mahalanobis_D_hat(0) = sqrt(Mahalanobis_D_hat_square(0));

        //Value for 99% we need 6.63.
        //Value for 95% we 3.84
        if (Mahalanobis_D_hat_square(0) > mahalanobis_distance_mocap_z)
        {
            report(LogLevel::Info, "MOCAP Z measurement rejected");
            return false;
        }
        else
        {
            return true;
        }
    }

    bool XYZEstimator::chi2AcceptMocapXY(const TwistSample& twist_msg) 
    {
        numXYGates++;
        //Compute Mahalanobis distance.
        Eigen::Vector2d measurement;
        measurement << twist_msg.vx, twist_msg.vy;

        Eigen::Vector2d expected_rgbd;
        expected_rgbd = xyEst.H * xyEst.xHat;
        Eigen::MatrixXd S(1, 1);
        //        Eigen::Matrix2d mocap_R;

        S = xyEst.H * xyEst.P * xyEst.H.transpose() + xyEst.R;
        Mahalanobis_D_hat_square(0) = (measurement - expected_rgbd).transpose() * S.inverse() * (measurement - expected_rgbd);
        Mahalanobis_D_hat(0) = sqrt(Mahalanobis_D_hat_square(0));
        //Value for 99% we need 6.63.
        //Value for 95% we 3.84
        if (Mahalanobis_D_hat_square(0) > mahalanobis_distance_mocap_xy_)
        {
            report(LogLevel::Info, "MOCAP XY measurement rejected");
            return false;
        } 
        else 
        {
            return true;
        }
        
    }

    void XYZEstimator::checkTakeoffState(double accMagnitude)
    {
        sonarTakeoffState = zEst.z(0) <= -0.25;

        //Check variance of accelerometer vector magnitude
        if ((!accTakeoffState) || (!sonarTakeoffState && accTakeoffState))
        {
            if (numAccSamples < ACC_SAMPLE_SIZE)
            {
                accSamples[numAccSamples++] = accMagnitude;
            }
            else
            {
                //shift sample in and compute the sample mean simultaneously
                accMean = 0;
                for (int i = 0; i < ACC_SAMPLE_SIZE - 1; i++)
                {
                    accSamples[i] = accSamples[i + 1];
                    accMean += accSamples[i];
                }
                accSamples[ACC_SAMPLE_SIZE - 1] = accMagnitude;
                accMean += accMagnitude;
                accMean /= (double) ACC_SAMPLE_SIZE;

                //Finally, compute the sample variance
                accVariance = 0;
                for (int i = 0; i < ACC_SAMPLE_SIZE; i++)
                {
                    double distFromMean = accSamples[i] - accMean;
                    accVariance += (distFromMean * distFromMean);
                }
                accVariance /= (double) (ACC_SAMPLE_SIZE - 1);
                accTakeoffState = accVariance >= ACC_TAKEOFF_VARIANCE;
            }
        }
        else if (numAccSamples > 0)
        {
            numAccSamples = 0;
        }

        //Publish changes in takeoff state
        if (accTakeoffState && sonarTakeoffState && !takeoffState)
        {
            report(LogLevel::Info, "Takeoff!");

            zEst.setTakeoffState(true);
            takeoffState = true;
            numTakeoffTransitions++;
        }
        else if (!accTakeoffState && !sonarTakeoffState && takeoffState)
        {
            report(LogLevel::Info, "Landing!");

            zEst.setTakeoffState(false);
            takeoffState = false;
            numTakeoffTransitions++;
        }
    }

    void XYZEstimator::saveMinusState()
    {
        //XY estimator publisher block------------------------------------------
        xyMinus.x = xyEst.xHat;
        xyMinus.P = xyEst.P;

        //Z estimator publisher block------------------------------------------
        zMinus.z = zEst.xHat(0, 0);
        zMinus.z_dot = zEst.xHat(1, 0);
        zMinus.bias = zEst.xHat(2, 0);
        zMinus.u = zEst.u(0);
        zMinus.P = zEst.P;
    }

    XYState XYZEstimator::xyPlusState() const
    {
        XYState s;
        s.x = xyEst.xHat;
        s.P = xyEst.P;
        return s;
    }

    ZState XYZEstimator::plusState() const
    {
        ZState s;
        s.z = zEst.xHat(0);
        s.z_dot = zEst.xHat(1);
        s.bias = zEst.xHat(2);
        s.u = zEst.u(0);
        s.P = zEst.P;
        return s;
    }

    //Utility function
    double getVectorMagnitude(double x, double y, double z)
    {
        return sqrt(x * x + y * y + z * z);
    }
}

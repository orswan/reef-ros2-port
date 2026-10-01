//
// Created by humberto on 5/20/19.
//
// ROS 2 port (P08) of rgbd_to_velocity b7637198: the converter without ROS.
// Messages are plain structs with the ROS 1 member paths and types;
// publishers are sinks set by the node; parameters come in
// ConverterParameters (validated before construction, VISION.md Q10).
// Statement order and Eigen expressions are the original's (VISION.md §2).

#ifndef RGBD_TO_VELOCITY_H
#define RGBD_TO_VELOCITY_H
#endif //TURTLE_NAV_TURTLE_H
#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <cmath>

namespace rgbd_to_velocity {

    // Plain stand-ins for the messages (ROS 1 member paths and types).
    struct Stamp { int64_t sec = 0; uint32_t nsec = 0;
        double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); } };
    struct Header { Stamp stamp; };
    struct Point { double x = 0, y = 0, z = 0; };
    struct Quaternion { double x = 0, y = 0, z = 0, w = 0; };   // ROS 1 default: all zero
    struct Odometry { Header header; struct { struct { Point position; Quaternion orientation; } pose; } pose; };
    struct PoseStamped { struct { Point position; Quaternion orientation; } pose; };
    struct Vector3 { double x = 0, y = 0, z = 0; };
    struct DeltaToVel {
        Header header;
        std::array<double, 6> S_upper_bound{}, S_lower_bound{};
        std::array<double, 3> scaled_std_xyz{};
        struct { Header header; struct { struct { Vector3 linear; } twist; std::array<double, 36> covariance{}; } twist; } vel;
    };

    struct ConverterParameters {
        double alpha = 1.0;
        double x_vel_covariance = 0.01;
        double y_vel_covariance = 0.01;
        std::vector<double> body_to_camera_quat;    // x, y, z, w (required, 4 values)
        std::vector<double> body_to_camera_trans;   // required, 3 values (read but unused, Q7)
    };
    // One message per invalid parameter; empty if valid (Q10).
    std::vector<std::string> parameterErrors(const ConverterParameters& p);

    class RgbdToVelocity {
    public:
        explicit RgbdToVelocity(const ConverterParameters& params);//Constructor
        ~RgbdToVelocity();//Destructor

        // Outputs: rgbd_to_velocity/init_frame and rgbd_to_velocity/body_level_frame.
        std::function<void(const DeltaToVel&)> velocity_init_frame_publisher_;
        std::function<void(const DeltaToVel&)> velocity_level_body_publisher_;
        long initFramePublished = 0, bodyLevelPublished = 0;

        double alpha;
        double current_time_stamp;
        double previous_time_stamp;
        double pitch;
        double roll;
        double yaw;
        double DT;
        double beta_0;
        double x_vel_covariance;
        double y_vel_covariance;
        int counterOfSamples;

        //Variable section
        Odometry odom_msg;
        PoseStamped pose;
        DeltaToVel vel_msg;
        void CtoYawPitchRoll213(Eigen::Matrix3d C);
        void CtoYawPitchRoll321(Eigen::Matrix3d C);
        Eigen::Matrix3d quaternionToRotation(Eigen::Vector3d quaternionVectorPart, double quaternionScalarPart);
        Eigen::Vector4d multiplyQuat(Eigen::MatrixXd q,Eigen::MatrixXd p);

        Eigen::MatrixXd inv_previous_quaternion_init_to_body;
        Eigen::MatrixXd current_quaternion_init_to_body;
        Eigen::Matrix3d C_from_init_to_camera_level_frame;
        Eigen::Matrix3d C_from_camera_level_frame_to_NED_level_frame;
        Eigen::Matrix3d covariance_matrix_in_init;
        Eigen::Matrix3d covariance_matrix_in_body_level;

        Eigen::Vector3d beta;
        Eigen::Vector3d previous_velocity_init;
        Eigen::Vector3d estimated_velocity_init;
        Eigen::Vector3d filtered_velocity_init;
        Eigen::Vector3d filtered_velocity_body_leveled_frame;
        Eigen::Vector3d previous_position_init;
        Eigen::Vector3d current_position_init;
        Eigen::VectorXd quaternion_body_to_camera;
        Eigen::Vector3d translation_body_to_camera;


        //Declare all callbacks here
        void poseCallback(const Odometry& msg);


    };
}

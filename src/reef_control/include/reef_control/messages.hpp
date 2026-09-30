// Plain stand-ins for the ROS 1 messages reef_control 12237b76 uses, with
// the same member paths and field types (command fields float32), so the
// ported controller code reads as the original. No ROS types; the node
// converts to and from ROS 2 messages (control_node.hpp).
#ifndef REEF_CONTROL_MESSAGES_HPP
#define REEF_CONTROL_MESSAGES_HPP

#include <cstdint>

namespace reef_control
{
    // Message stamp (ROS 1 Time: uint32 sec; ROS 2: int32 sec; both fit).
    struct Stamp { int64_t sec = 0; uint32_t nsec = 0; };
    struct Header { Stamp stamp; };

    // ROS 1 (Time - Time).toSec(): the difference normalized as roscpp_core
    // does (nsec in [0, 1e9)), then (double)sec + 1e-9 * (double)nsec.
    double durationSec(const Stamp& a, const Stamp& b);

    struct Vector3 { double x = 0, y = 0, z = 0; };
    struct Quaternion { double x = 0, y = 0, z = 0, w = 0; };   // ROS 1 default: all zero

    // nav_msgs/Odometry (fields used by the controller)
    struct Odometry
    {
        Header header;
        struct { struct { Vector3 position; Quaternion orientation; } pose; } pose;
        struct { struct { Vector3 linear; } twist; } twist;
    };

    // reef_msgs/XYZEstimate
    struct XYZEstimate
    {
        Header header;
        struct { double x_dot = 0, y_dot = 0; } xy_plus;
        struct { double z = 0, z_dot = 0; } z_plus;
    };

    // reef_msgs/DesiredVector, reef_msgs/DesiredState
    struct DesiredVector { double x = 0, y = 0, z = 0, yaw = 0; };
    struct DesiredState
    {
        Header header;
        uint32_t node_id = 0;
        DesiredVector pose, velocity, acceleration, attitude;
        bool attitude_valid = false, position_valid = false, velocity_valid = false,
             acceleration_valid = false, altitude_only = false;
    };

    // geometry_msgs/PoseStamped (position and orientation)
    struct PoseStamped { struct { Vector3 position; Quaternion orientation; } pose; };

    // rosflight_msgs/Status (armed), std_msgs/Bool
    struct Status { bool armed = false; };
    struct Bool { bool data = false; };

    // rosflight_msgs/Command of rosflight 44e5f37e (x, y, z, F float32)
    struct Command
    {
        static constexpr uint8_t MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2;
        static constexpr uint8_t IGNORE_NONE = 0, IGNORE_X = 1, IGNORE_Y = 2, IGNORE_Z = 4, IGNORE_F = 8;
        Header header;   // left zero, as the original; the node stamps the ROS message
        uint8_t mode = 0;
        uint8_t ignore = 0;
        float x = 0, y = 0, z = 0, F = 0;
    };
}

#endif

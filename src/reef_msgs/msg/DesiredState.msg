# Setpoint for reef_control (fields as in reef_msgs 7fb63ff; ported in P06).
# Frames and units as used by reef_control 12237b76 (docs/CONTROL_CHAIN.md):
#   pose:         x, y [m] in the mocap NED frame (position mode); z [m] NED,
#                 positive down (altitude setpoint, used in every mode);
#                 yaw [rad] (heading, position mode)
#   velocity:     x, y [m/s] body-level frame (x forward, y right); z [m/s]
#                 NED; yaw = yaw RATE [rad/s]
#   acceleration: controller outputs on controller_state; x, y are used as
#                 pitch/roll angles [rad] (pitch = -x, roll = y); z is the
#                 negated throttle
#   attitude:     x roll [rad], y pitch [rad], yaw sent as a yaw RATE
# Flags select the mode (CONTROL_CHAIN.md section 4). node_id: unused.
std_msgs/Header header
uint32 node_id
DesiredVector pose
DesiredVector velocity
DesiredVector acceleration
DesiredVector attitude
bool attitude_valid
bool position_valid
bool velocity_valid
bool acceleration_valid
bool altitude_only

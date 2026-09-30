# REEF estimator output, published after every IMU propagation.
# header.stamp: stamp of the IMU message that triggered the step.
# header.frame_id: empty; the fields use two frames (see XYEstimate, ZEstimate).
# node_id: not set by reef_estimator (0).
# This is NOT a full pose: no horizontal position, no attitude.
std_msgs/Header header
uint32 node_id

XYEstimate xy_plus
ZEstimate z_plus

# REEF estimator debug output (only when debug_mode is true).
# *_minus: after propagation, before the measurement update of the same step.
# *_plus: after the update. header as in XYZEstimate; node_id not set (0).
std_msgs/Header header
uint32 node_id

XYDebugEstimate xy_minus
XYDebugEstimate xy_plus

ZDebugEstimate z_minus
ZDebugEstimate z_plus

# Body-level velocity measurement produced by rgbd_to_velocity (not yet ported).
# Ported from reef_msgs 7fb63ff. ROS 2 field names must be lower case, so
# S_upper_bound and S_lower_bound became s_upper_bound and s_lower_bound.
#
# reef_estimator reads only vel.twist.twist.linear.x/.y [m/s] and
# vel.twist.covariance[0] and [7] (variances of x and y [m^2/s^2], row-major
# 6x6), in the body-level frame (yaw-aligned, gravity-levelled, x forward,
# y right, z down). The other fields are defined by the producer.

std_msgs/Header header

float64[6] s_upper_bound
float64[6] s_lower_bound
float64[3] scaled_std_xyz
geometry_msgs/TwistWithCovarianceStamped vel

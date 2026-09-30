# Full vertical filter state, NED (positive down).
# Ported from reef_msgs 7fb63ff. ROS 2 field names must be lower case, so the
# covariance field P became p.
#
# z [m], z_dot [m/s], bias [m/s^2] (acceleration offset b, a = u - b).
# u [m/s^2]: vertical specific force in NED plus 9.81 (the filter input).
# p: covariance of [z, z_dot, bias], row-major 3x3.
# truth, z_error, z_dot_error: not set by reef_estimator (0). Not measurements.
# sigma_plus/sigma_minus: [z, z_dot, bias] +/- 3 sqrt(diag(p)).
float64 z
float64 z_dot
float64 bias
float64 u
float64[9] p

float64[2] truth
float64 z_error
float64 z_dot_error
float64[3] sigma_plus
float64[3] sigma_minus

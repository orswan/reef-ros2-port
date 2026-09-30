# Full horizontal filter state (body-level frame).
# x_dot, y_dot [m/s]; pitch_bias, roll_bias [rad]; xa_bias, ya_bias [m/s^2].
# sigma_plus/sigma_minus: state +/- 3 sqrt(diag(P)), in the order above.
float64 x_dot
float64 y_dot
float64 pitch_bias
float64 roll_bias
float64 xa_bias
float64 ya_bias
float64[6] sigma_plus
float64[6] sigma_minus

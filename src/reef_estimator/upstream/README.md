# Not yet ported

`include/xy_estimator.h` and `src/xy_estimator.cpp` are the unmodified
horizontal filter of reef_estimator master `e4179f48`. They are not built.
They move into the package when the horizontal filter is ported; until then
`reef_check.sh estimator` reports horizontal coverage as NOT IMPLEMENTED.
The rest of the ROS 1 package (catkin files, launch, params, scripts, docs)
was removed from the tree and remains in the history.

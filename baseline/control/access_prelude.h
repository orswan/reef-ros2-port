// Adaptation C1 (see baseline/README.md): force-included into every
// translation unit of the controller reference harness, as A1 is for the
// estimator. Library headers first (include-guarded), then the access
// keywords are redefined so the harness can read the controller's private
// and protected state. Access control only; no layout or arithmetic change.
#pragma once
#define BOOST_BIND_GLOBAL_PLACEHOLDERS  // roscpp exposed boost::bind's _1, _2 globally
#include <algorithm>
#include <cmath>
#include <math.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <sstream>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#define private public
#define protected public

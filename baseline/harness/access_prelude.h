// Adaptation A1 (see baseline/README.md): force-included into every
// translation unit of the reference harness.
//
// It first includes every standard, Boost and Eigen header that the pinned
// REEF sources and the ROS stand-ins use. These headers are include-guarded,
// so when the REEF sources include them again later they are not reprocessed.
// Only then are the access keywords redefined, which lets the harness read
// the estimators' private and protected state. Because the redefinition
// happens after all library headers, it affects only the REEF class
// definitions, identically in every translation unit. It changes access
// control only, not layout or arithmetic.
#pragma once
#include <cmath>
#include <math.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#define private public
#define protected public

//
// Created by prashant on 2/28/19.
//
// ROS 2 port (P03). Depends on Eigen and the standard library only.
//
// Kept with legacy semantics (reef_msgs 7fb63ff): vectorToMatrix (row-major),
// vectorToDiagMatrix, matrixToArray (row-major; boost::array became the
// std::array used by ROS 2 messages).
//
// importMatrixFromParamServer (ROS 1 parameter server) is split into
// importMatrixFromVector() here and importMatrixFromParameter() in
// parameters.hpp. The legal forms keep their legacy meaning: n*m values fill
// the matrix row-major, checked first; n values on an n x n matrix fill the
// diagonal. Unlike the original, invalid input is an error and leaves the
// matrix unchanged. The original zero-filled missing parameters and left
// wrong-sized matrices uninitialized (BASELINE_DECISION.md D10).
//
// Not ported: matrixToVector (no return statement), verifyDimensions,
// loadTransform (not used by reef_estimator).

#ifndef PROJECT_MATRIX_OPERATION_H
#define PROJECT_MATRIX_OPERATION_H

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace reef_msgs
{
template <class Derived>
bool vectorToMatrix(Eigen::MatrixBase<Derived>& mat, const std::vector<double>& vec)
{
  if(vec.size() != static_cast<std::size_t>(mat.rows()*mat.cols()))
    return false;
  for(Eigen::Index i=0; i < mat.rows(); i++)
  {
    for(Eigen::Index j=0; j < mat.cols(); j++)
    {
      mat(i,j) = vec[mat.cols()*i+j];
    }
  }
  return true;
}

// The original asserted vec.size() == rows only; a non-square matrix could be
// written out of bounds. The port also requires a square matrix.
template <class Derived>
bool vectorToDiagMatrix(Eigen::MatrixBase<Derived>& mat, const std::vector<double>& vec)
{
  if(mat.rows() != mat.cols() || vec.size() != static_cast<std::size_t>(mat.rows()))
    return false;
  mat.setZero();
  for(Eigen::Index i=0; i < mat.rows(); i++)
  {
    mat(i,i) = vec[i];
  }
  return true;
}

template <class Derived, std::size_t N>
bool matrixToArray(const Eigen::MatrixBase<Derived> &mat, std::array<double,N> &vec)
{
  if(vec.size() != static_cast<std::size_t>(mat.rows()*mat.cols()))
    return false;
  for(Eigen::Index i=0; i < mat.rows(); i++)
  {
    for(Eigen::Index j=0; j < mat.cols(); j++)
    {
      vec[mat.cols()*i+j] = mat(i,j);
    }
  }
  return true;
}

enum class MatrixLayout { Full, Diagonal };

struct MatrixImport
{
  bool ok = false;
  MatrixLayout layout = MatrixLayout::Full;
  std::string error;  // empty when ok
};

// Human-readable list of the accepted value counts for a rows x cols matrix.
inline std::string acceptedSizes(Eigen::Index rows, Eigen::Index cols)
{
  std::string s = std::to_string(rows*cols) + " values (" + std::to_string(rows) + "x" +
                  std::to_string(cols) + ", row-major)";
  if(rows == cols && rows > 1)
    s += " or " + std::to_string(rows) + " values (diagonal)";
  return s;
}

// Fills mat from vec with the legacy rules. On error, mat is not modified.
template <class Derived>
MatrixImport importMatrixFromVector(Eigen::MatrixBase<Derived>& mat, const std::vector<double>& vec,
                                    const std::string& name)
{
  MatrixImport result;
  const std::string accepted = acceptedSizes(mat.rows(), mat.cols());
  if(vec.empty())
  {
    result.error = name + " is empty; expected " + accepted;
    return result;
  }
  for(std::size_t k=0; k < vec.size(); k++)
  {
    if(!std::isfinite(vec[k]))
    {
      result.error = name + "[" + std::to_string(k) + "] is not finite";
      return result;
    }
  }
  if(vectorToMatrix(mat, vec))
  {
    result.ok = true;
    result.layout = MatrixLayout::Full;
  }
  else if(vectorToDiagMatrix(mat, vec))
  {
    result.ok = true;
    result.layout = MatrixLayout::Diagonal;
  }
  else
  {
    result.error = name + " has " + std::to_string(vec.size()) + " values; expected " + accepted;
  }
  return result;
}

// Additional P03 validation (not in the original). Returns an empty string if
// mat is square, finite, exactly symmetric, and has a non-negative diagonal.
template <class Derived>
std::string covarianceError(const Eigen::MatrixBase<Derived>& mat, const std::string& name)
{
  if(mat.rows() != mat.cols())
    return name + " is not square";
  for(Eigen::Index i=0; i < mat.rows(); i++)
  {
    for(Eigen::Index j=0; j < mat.cols(); j++)
    {
      if(!std::isfinite(mat(i,j)))
        return name + "(" + std::to_string(i) + "," + std::to_string(j) + ") is not finite";
      if(mat(i,j) != mat(j,i))
        return name + " is not symmetric at (" + std::to_string(i) + "," + std::to_string(j) + ")";
    }
    if(mat(i,i) < 0)
      return name + "(" + std::to_string(i) + "," + std::to_string(i) + ") is negative";
  }
  return "";
}

// Additional P03 validation. Returns an empty string if every element is in [lo, hi].
template <class Derived>
std::string rangeError(const Eigen::MatrixBase<Derived>& mat, const std::string& name, double lo, double hi)
{
  for(Eigen::Index i=0; i < mat.size(); i++)
  {
    const double v = mat.coeff(i);
    if(!(v >= lo && v <= hi))
      return name + "[" + std::to_string(i) + "] = " + std::to_string(v) + " is outside [" +
             std::to_string(lo) + ", " + std::to_string(hi) + "]";
  }
  return "";
}

}


#endif //PROJECT_MATRIX_OPERATION_H

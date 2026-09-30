// Reader for test/data/legacy_helper_vectors.txt (see
// baseline/harness/helper_vectors.cpp for the format).
#ifndef REEF_MSGS__TEST__LEGACY_VECTORS_HPP_
#define REEF_MSGS__TEST__LEGACY_VECTORS_HPP_

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace legacy
{

struct Case
{
  std::string kind;           // Q2R, RPY, MAT, ARR
  std::string name;           // MAT only
  int rows = 0, cols = 0;     // MAT, ARR
  char status = 0;            // MAT only: F, D, Z, U
  std::vector<double> in, out;
};

inline double parse(const std::string & tok)
{
  // strtod reads C99 hex floats, "inf", "nan" and "-nan" exactly.
  char * end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') {
    throw std::runtime_error("bad number: " + tok);
  }
  return v;
}

inline std::vector<Case> load(const char * path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error(std::string("cannot open ") + path);
  }
  std::vector<Case> cases;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    Case c;
    ss >> c.kind;
    std::size_t n_in = 0;
    if (c.kind == "Q2R") {
      n_in = 4;
    } else if (c.kind == "RPY") {
      n_in = 9;
    } else if (c.kind == "MAT") {
      ss >> c.name >> c.rows >> c.cols >> n_in;
    } else if (c.kind == "ARR") {
      ss >> c.rows >> c.cols;
      n_in = static_cast<std::size_t>(c.rows * c.cols);
    } else {
      throw std::runtime_error("unknown case: " + line);
    }
    std::string tok;
    for (std::size_t k = 0; k < n_in; k++) {
      ss >> tok;
      c.in.push_back(parse(tok));
    }
    ss >> tok;
    if (tok != "|") {
      throw std::runtime_error("missing separator: " + line);
    }
    if (c.kind == "MAT") {
      ss >> c.status;
    }
    while (ss >> tok) {
      c.out.push_back(parse(tok));
    }
    cases.push_back(c);
  }
  return cases;
}

// Bit-identical, except that any NaN matches any NaN (legacy NaN signs and
// payloads are not part of the contract).
inline bool same(double a, double b)
{
  if (std::isnan(a) || std::isnan(b)) {
    return std::isnan(a) && std::isnan(b);
  }
  return std::memcmp(&a, &b, sizeof a) == 0;
}

}  // namespace legacy

#endif  // REEF_MSGS__TEST__LEGACY_VECTORS_HPP_

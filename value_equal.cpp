#include "value_equal.h"

template<>
bool value_equal(const double& t1, const double& t2) {
  return (t1 - t2) >= -1e-6 && (t1 - t2) <= 1e-6;
}

template<>
bool value_equal(const float& t1, const float& t2) {
  return (t1 - t2) >= -1e-6 && (t1 - t2) <= 1e-6;
}

template<>
bool value_b(const double& t1, const double& t2) {
  return (t1 - t2) > 1e-6;
}

template<>
bool value_s(const double& t1, const double& t2) {
  return (t1 - t2) < -1e-6;
}
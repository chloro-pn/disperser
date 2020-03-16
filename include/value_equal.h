#pragma once

template<class T>
bool value_equal(const T& t1, const T& t2) {
  return t1 == t2;
}

template<>
bool value_equal(const double& t1, const double& t2);

template<>
bool value_equal(const float& t1, const float& t2);

template<class T>
bool value_b(const T& t1, const T& t2) {
  return t1 > t2;
}

template<>
bool value_b(const double& t1, const double& t2);

template<class T>
bool value_s(const T& t1, const T& t2) {
  return t1 < t2;
}

template<>
bool value_s(const double& t1, const double& t2);
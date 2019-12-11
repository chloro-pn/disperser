#pragma once
#include <string>

namespace pnlog {
  template<class T>
  std::string convert_to_string(const T& t) {
    return std::to_string(t);
  }

  template<>
  std::string convert_to_string(const std::string& str);

  std::string convert_to_string(const char* ptr);

  template<size_t n>
  std::string convert_to_string(const char(&ref)[n]) {
    return std::string(ref);
  }
}//namespace pnlog
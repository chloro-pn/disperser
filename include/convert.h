#pragma once
#include <string>

template<class T>
std::string convert_to_string(const T& t) {
  return std::to_string(t);
}

template<>
std::string convert_to_string(const std::string& str);

/*
这里可以说明一个问题，在类型推断过程中可以看到，指针的*左边的const不是指针类型本身的const，我们把它作为
类型本身的一部分。
而*右边的const就像其他非指针类型的const属性一样参与推断。
*/
using char_type = const char*;
template<>
std::string convert_to_string(const char_type& ptr);

std::string convert_to_string(const char* ptr);
#include "convert.h"

template<>
std::string convert_to_string(const std::string& str) {
	return str;
}

template<>
std::string convert_to_string(const char_type& ptr) {
	//ptr的类型是const char* const;
	return std::string(ptr);
}

std::string convert_to_string(const char* ptr) {
	return std::string(ptr);
}
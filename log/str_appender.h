#pragma once
#include "convert.h"
#include <string>

namespace pnlog {
  template<class LastType>
  std::string piece(LastType&& last) {
    return convert_to_string(std::forward<LastType>(last));
  }

  template<class ThisType, class... Types>
  std::string piece(ThisType&& tt, Types&&... args) {
    std::string tmp(convert_to_string(std::forward<ThisType>(tt)));
    tmp.append(piece(std::forward<Types>(args)...));
    return tmp;
  }
}//namespace pnlog
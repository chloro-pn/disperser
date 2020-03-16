#pragma once
#include "../log/back_end.h"
#include "../log/capture.h"
#include "../log/str_appender.h"
#include "../log/file_out_stream.h"
#include <memory>

namespace pnlog {
  extern std::shared_ptr<BackEnd> backend;
  extern std::shared_ptr<CapTure> capture;
}

using pnlog::piece;

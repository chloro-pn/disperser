#pragma once
#include <cstdio>
#include <string>
#include "type.h"
#include "out_stream_base.h"

namespace pnlog {
  class FileOutStream : public out_stream_base {
  private:
    FILE * file_;
    bool closed_;

  public:
    explicit FileOutStream(std::string filename);

    void write(const char* ptr, size_type n) override;

    void close() override;

    ~FileOutStream();
  };
}//namespace pnlog
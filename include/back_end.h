#pragma once
#include "file_out_stream.h"
#include "type.h"
#include "buf_container.h"
#include "thread_pool.h"
#include "out_stream_base.h"
#include <memory>
#include <atomic>

class BackEnd {
public:
  static constexpr size_t FILES = 4;

  void write(size_type index, const char* ptr, size_type n);

  explicit BackEnd(size_type size);

  void open(size_type index, out_stream_base* out, size_type log_container_size = 12);

  std::shared_ptr<out_stream_base>& out_stream(size_type index);

  BufContainer& buf_container(size_type index);

  void stop();

  ~BackEnd();

private:
  ThreadPool pool_;
  std::shared_ptr<out_stream_base> out_streams_[FILES];
  BufContainer buf_containers_[FILES];
  std::atomic<bool> stop_;

  void rangecheck(size_type index) const;
};
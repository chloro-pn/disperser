#pragma once
#include "type.h"
#include "char_array.h"
#include "thread_pool.h"
#include "out_stream_base.h"
#include "spin_lock.h"
#include <memory>
#include <atomic>
#include <vector>
#include <condition_variable>
#include <mutex>

namespace pnlog {
  class BackEnd {
    friend class CapTure;

    using lock_type = std::mutex;
  public:

    static std::shared_ptr<BackEnd> get_instance();

    BackEnd(const BackEnd&) = delete;
    BackEnd(BackEnd&&) = delete;
    BackEnd& operator=(const BackEnd&) = delete;
    BackEnd& operator=(BackEnd&&) = delete;

    void open(size_type index, out_stream_base* out, size_type log_container_size = 6);

    void open_syn(size_type index, out_stream_base* out);

    void close(size_type index);

    void stop();

    void abort(const char* = nullptr);

    ~BackEnd();

    bool rangecheck(size_type index) const;

  private:
    ThreadPool pool_;
    std::vector<std::shared_ptr<out_stream_base>> out_streams_;
    std::vector<std::shared_ptr<CharArray>> bufs_;
    std::vector<std::shared_ptr<lock_type>> mutexs_;
    std::vector<std::shared_ptr<std::condition_variable>> cv_can_write_;
    std::vector<bool> stops_;

    size_type size_of_streams_and_bufs_;
    std::atomic<bool> stop_;

    explicit BackEnd(size_type size);

    void write(size_type index, const char* ptr, size_type n);

    std::shared_ptr<out_stream_base>& out_stream(size_type index);

    void all_flush();

    void write_in_thread_pool(size_type index);
  };
}//namespace pnlog
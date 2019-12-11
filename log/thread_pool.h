#pragma once
#include <thread>
#include <vector>
#include <functional>
#include <list>
#include <condition_variable>
#include "type.h"
#include "spin_lock.h"

namespace pnlog {
  class ThreadPool {
  private:
    std::vector<std::thread> threads_;
    using task_type = std::function<void()>;
    std::list<task_type> tasks_;
    size_type th_counts_;
    std::condition_variable_any cv_;
    spin mut_;
    bool stop_;

  public:
    ThreadPool(size_type count);

    void each_thread();

    void start();

    void stop();

    template<class F>
    void push_task(F&& func) {
      mut_.lock();
      tasks_.push_back(std::forward<F>(func));
      mut_.unlock();
      cv_.notify_one();
    }


    ~ThreadPool();
  };
}//namespace pnlog
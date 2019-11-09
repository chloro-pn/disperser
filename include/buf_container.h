#pragma once
#include <list>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include "char_array.h"
#include "type.h"
#include "out_stream_base.h"

class BufContainer {
private:
  using CharArrayType = CharArray;
  CharArrayType buf_;
  std::list<CharArrayType> other_buf_;
  size_type bufs_;
  size_type the_first_clean_;
  std::condition_variable cv_can_write_;
  std::mutex mut_;
  bool inited_;
  bool stop_;

public:
	static constexpr size_type buf_size_ = 4096;

	BufContainer();

	bool init(size_type bufs);

  inline
  bool inited() const {
    return inited_;
  }

	bool write(const char* ptr, size_type n);

	void swapInBack();

	bool backEnd(std::shared_ptr<out_stream_base> out);

	void stop();

	~BufContainer();
};
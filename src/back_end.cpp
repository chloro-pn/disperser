#include "back_end.h"
#include "capture.h"
#include "std_out_stream.h"
#include "platform.h"
#include <iostream>

static_assert(BufContainer::buf_size_ >= CapTure::buf_size_, "container buf_size should >= capture buf_size!");

//这个函数不是线程安全的，是前后端连接的纽带。
//前端通过线程安全的组件在一个线程中调用此函数
void BackEnd::write(size_type index, const char* ptr, size_type n) {
  if (stop_.load() == true) {
    return;
  }
	rangecheck(index);
	if (out_stream(index) == nullptr) {
    stop();
		pf::fprintf(stderr, "write non-opened file : %d\n", static_cast<int>(index));
    system("pause");
		exit(-1);
	}
	if (buf_container(index).inited() == false) {
		out_stream(index)->write(ptr, n);
	}
	else {
		bool succ = buf_container(index).write(ptr, n);
		if (succ == false) {
      stop();
			pf::fprintf(stderr, "buf_container write error:%d\n",static_cast<int>(index));
      system("pause");
			exit(-1);
		}
	}
}

BackEnd::BackEnd(size_type size) :pool_(size),stop_(false) {
	for (size_type i = 0; i < FILES; ++i) {
		out_streams_[i] = nullptr;
	}
	out_stream(0).reset(new StdOutStream(stdout));
	out_stream(1).reset(new StdOutStream(stderr));
	pool_.start();
}

void BackEnd::open(size_type index,out_stream_base* out, size_type log_container_size) {
	rangecheck(index);
	if (out_stream(index) != nullptr) {
		stop();
    pf::fprintf(stderr, "register opened file");
    system("pause");
		exit(-1);
	}

	out_stream(index) = std::shared_ptr<out_stream_base>(out);
	buf_container(index).init(log_container_size);

	pool_.push_task([this,index]()->bool {
		return buf_container(index).backEnd(out_stream(index));
		//上一个bug居然是这里的捕获有问题。emm
		});
}

inline
std::shared_ptr<out_stream_base>& BackEnd::out_stream(size_type index) {
	rangecheck(index);
	return out_streams_[index];
}

inline
BufContainer& BackEnd::buf_container(size_type index) {
	rangecheck(index);
	return buf_containers_[index];
}

//需要是线程安全的,可能在不同线程调用stop函数。
//考虑用原子变量实现线程安全。
void BackEnd::stop() {
  bool exp = false;
	if (stop_.compare_exchange_strong(exp,true)) {
		for (size_type i = 0; i < FILES; ++i) {
			buf_container(i).stop();
		}
		pool_.stop();
	}
}

BackEnd::~BackEnd() {

}

void BackEnd::rangecheck(size_type index) const {
	if (index < 0 || index >= FILES) {
		const_cast<BackEnd*>(this)->stop();
    pf::fprintf(stderr, "index out of range");
    system("pause");
		exit(-1);
	}
}
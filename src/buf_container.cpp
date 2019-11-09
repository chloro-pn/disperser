#include "buf_container.h"
#include "out_stream_base.h"
#include "platform.h"

BufContainer::BufContainer() :the_first_clean_(1), 
							  inited_(false), 
							  bufs_(-1), 
							  stop_(false), 
							  buf_(buf_size_) {

}

bool BufContainer::init(size_type bufs) {
	if (bufs < 1) {
		return false;
	}
	other_buf_.resize(static_cast<unsigned int>(bufs));
	bufs_ = bufs;
	inited_ = true;
	return true;
}

bool BufContainer::write(const char* ptr, size_type n) {
	buf_.append(ptr, n);
	if (buf_.error() == true) {
		swapInBack();
		buf_.append(ptr, n); //如果向后端输出的log太长怎么处理？考虑这个问题。要确保前端传来的log至少能被空的一个buf装入并write，否则陷入无限递归。
		if (buf_.error() == true) {
			return false;
		}
	}
	return true;
}

void BufContainer::swapInBack() { //写入方
	std::unique_lock<std::mutex> mut(mut_);
	if (buf_.getSize() == 0) {
		return;
	}
	while (the_first_clean_ > bufs_) {
		cv_can_write_.wait(mut);
	}
	auto it = other_buf_.begin();
	for (size_type i = 1; i < the_first_clean_; ++i) {
		++it;
	}

	using std::swap;
	swap(buf_, *it);
	++the_first_clean_;
	mut.unlock();
}

bool BufContainer::backEnd(std::shared_ptr<out_stream_base> out) {
	static CharArrayType backbuf(buf_size_); 
	std::unique_lock<std::mutex> mut(mut_);
	if (the_first_clean_ <= 1 && stop_ == true) { //actually the_first_clean_不会小于1.
		return false; //bool返回值为了实现在线程中不断地自我加入，如果返回false
		//则意味着不再自我加入了。
		//当调用stop时，不仅将stop_设置为true，还要将当前未满的buf转换到后台写入文件。
	}
	if (the_first_clean_ <= 1 && stop_ == false) {
		return true;
	}
	//std::cout << "buf back end : " << the_first_clean_ << "\n";
	CharArrayType tmp = std::move(other_buf_.front());
	other_buf_.erase(other_buf_.begin());
	other_buf_.push_back(std::move(tmp));
	std::swap(backbuf, other_buf_.back());
	--the_first_clean_;
	mut.unlock();
	cv_can_write_.notify_all();
	out->write(backbuf.getBuf(), backbuf.getSize());
	backbuf.setZero();
	return true;
}

void BufContainer::stop() {
	if (inited() == true) {
		swapInBack();
		mut_.lock();
		stop_ = true;
		mut_.unlock();
	}
}

BufContainer::~BufContainer() {

}
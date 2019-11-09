#pragma once
#include <thread>
#include <vector>
#include <functional>
#include <list>
#include <condition_variable>
#include <mutex>
#include "type.h"

class ThreadPool {
private:
	std::vector<std::thread> threads_;
	using task_type = std::function<bool()>;
	std::list<task_type> tasks_;
	size_type th_counts_;
	std::condition_variable cv_;
	std::mutex mut_;
	bool stop_;

public:
	ThreadPool(size_type count):th_counts_(count),stop_(false) {

	}

	void each_thread() {
		while (true) {
			std::unique_lock<std::mutex> lock(mut_);
			while (tasks_.empty() == true && stop_ == false) {
				cv_.wait(lock);
			}
			if (stop_ == true && tasks_.empty() == true) {
				return;
			}
			auto tmp = std::move(tasks_.front());
			tasks_.erase(tasks_.begin());
			lock.unlock();
			bool conti = tmp();
			if (conti == true) {
				push_task(tmp);
			}
		}
	}

	void start() {
		for (size_type i = 0; i < th_counts_; ++i) {
			threads_.emplace_back([this]()->void {
				this->each_thread();
				});
		}
	}

	void stop() {
		mut_.lock();
		stop_ = true;
		mut_.unlock();
		cv_.notify_all();
		for (auto it = threads_.begin(); it != threads_.end(); ++it) {
			it->join();
		}
	}

	template<class F>
	void push_task(const F& func) {
		mut_.lock();
		tasks_.push_back(func);
		mut_.unlock();
		cv_.notify_one();
	}

	~ThreadPool() {

	}
};
#include "capture.h"
#include "platform.h"

CapTure::CapTure(BackEnd& b) :back_(b), index_(0) {

}

void CapTure::log(size_type index, Level level,size_type line, const char* file, const std::string& str) {
	static thread_local CharArray tmp(buf_size_);
	tmp.append("file: ");
	tmp.append(file);
	tmp.append(" line: ");
	tmp.append(convert_to_string(line).c_str());
	tmp.append(" : ");
	tmp.append(str.c_str());
	tmp.append("\n");
	if (tmp.error() == true) {
    back_.stop();
		pf::fprintf(stderr, "log is too long!\n");
    system("pause");
		exit(-1);
	}
	std::unique_lock<std::mutex> mut(mut_);
	back_.write(index, tmp.getBuf(), tmp.getSize());
	mut.unlock();
	if (level == Level::FATAL) {
		back_.stop();//需要是线程安全的。
    system("pause");
		exit(-1);
	}
	tmp.setZero();
}

void CapTure::log_trace(size_type index, size_type line, const char* file, const std::string& str) {
	if (default_level_ <= Level::TRACE) {
		log(index, Level::TRACE, line, file, str);
	}
}

void CapTure::log_debug(size_type index, size_type line, const char* file, const std::string& str) {
	if (default_level_ <= Level::DEBUG) {
		log(index, Level::DEBUG, line, file, str);
	}
}

void CapTure::log_error(size_type index, size_type line, const char* file, const std::string& str) {
	if (default_level_ <= Level::ERROR) {
		log(index, Level::ERROR, line, file, str);
	}
}

void CapTure::log_fatal(size_type index, size_type line, const char* file, const std::string& str) {
	if (default_level_ <= Level::FATAL) {
		log(index, Level::FATAL, line, file, str);
	}
}

void CapTure::log_trace(size_type index, const std::string& str) {
  if (default_level_ <= Level::TRACE) {
    log(index, Level::TRACE, -1, "", str);
  }
}

void CapTure::log_debug(size_type index, const std::string& str) {
  if (default_level_ <= Level::DEBUG) {
    log(index, Level::DEBUG, -1, "", str);
  }
}

void CapTure::log_error(size_type index, const std::string& str) {
  if (default_level_ <= Level::ERROR) {
    log(index, Level::ERROR, -1, "", str);
  }
}

void CapTure::log_fatal(size_type index, const std::string& str) {
  if (default_level_ <= Level::FATAL) {
    log(index, Level::FATAL, -1, "", str);
  }
}

void CapTure::setLevel(Level l) {
	default_level_ = l;
}

CapTure::Level CapTure::getLevel(){
	return default_level_;
}

CapTure::~CapTure() {

}

CapTure::Level CapTure::default_level_ = CapTure::Level::DEBUG;
#include "std_out_stream.h"
#include "platform.h"
#include <cstdio>

StdOutStream::StdOutStream(FILE* f) :file_(f) {

}

void StdOutStream::write(const char* ptr, size_type n) {
	//fprintf_s(file_, ptr);
	pf::fprintf(file_, ptr);
}

void StdOutStream::close() {

}

StdOutStream::~StdOutStream() {
	close();
}
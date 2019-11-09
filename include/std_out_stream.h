#pragma once
#include "out_stream_base.h"
#include "type.h"
#include <cstdio>

class StdOutStream : public out_stream_base {
public:
	StdOutStream(FILE* f);

	void write(const char* ptr, size_type n) override;

	void close() override ;

	~StdOutStream();
private:
	FILE* file_;
};
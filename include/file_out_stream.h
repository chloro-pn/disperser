#pragma once
#include <cstdio>
#include <string>
#include "type.h"
#include "out_stream_base.h"

class FileOutStream : public out_stream_base {
private:
	FILE* file_;

public:
	FileOutStream(std::string filename);

	void write(const char* ptr, size_type n) override;

	void close() override ;

	~FileOutStream();
};
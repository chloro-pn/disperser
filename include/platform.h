#pragma once
#include <cstdio>
struct win {};
struct linux {};

template<class T>
class platform;

template<>
class platform<win> {
public:
	static void fopen(FILE** stream,const char* str,const char* mode) {
		if (fopen_s(stream, str, mode) != 0) {
			::fprintf_s(stderr, "file open error!");
		}
	}

	template<class... Types>
	static void fprintf(FILE* stream,const char* str,Types&&... args) {
		::fprintf_s(stream, str, std::forward<Types>(args)...);
	}

	static void fwrite(const char* buf,size_t size,size_t count,FILE* stream) {
		::fwrite(buf, size, count, stream);
	}

	static void fflush(FILE* stream) {
		::fflush(stream);
	}

	static void fclose(FILE* stream) {
		::fclose(stream);
	}
};

#ifdef _WIN32
using pf = platform<win>;
#endif
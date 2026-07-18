#pragma once

// Minimal CUDA driver API stubs for safe_call_utils.h

typedef int CUresult;
enum { CUDA_SUCCESS = 0 };

inline CUresult cuGetErrorName(CUresult, const char** name) {
	static const char* kName = "CUDA_ERROR_NOT_SUPPORTED";
	if (name) *name = kName;
	return CUDA_SUCCESS;
}

inline CUresult cuGetErrorString(CUresult, const char** str) {
	static const char* kStr = "CUDA driver is stubbed (CPU-only build)";
	if (str) *str = kStr;
	return CUDA_SUCCESS;
}

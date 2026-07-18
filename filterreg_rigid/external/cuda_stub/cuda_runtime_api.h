#pragma once

// Minimal CUDA runtime API stubs. GPU allocation paths abort at runtime;
// rigid_pt2pt / rigid_pt2pl only use MemoryContext::CpuMemory.

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

typedef int cudaError_t;
enum {
	cudaSuccess = 0,
	cudaErrorNotSupported = 9999
};

typedef enum {
	cudaMemcpyHostToHost = 0,
	cudaMemcpyHostToDevice = 1,
	cudaMemcpyDeviceToHost = 2,
	cudaMemcpyDeviceToDevice = 3,
	cudaMemcpyDefault = 4
} cudaMemcpyKind;

typedef void* cudaStream_t;

inline const char* cudaGetErrorName(cudaError_t) { return "cudaErrorNotSupported"; }
inline const char* cudaGetErrorString(cudaError_t) {
	return "CUDA runtime is stubbed (CPU-only build)";
}
inline cudaError_t cudaDeviceReset() { return cudaSuccess; }

inline cudaError_t cudaMalloc(void**, size_t) {
	fprintf(stderr, "cudaMalloc called in CPU-only stub build\n");
	return cudaErrorNotSupported;
}

inline cudaError_t cudaFree(void*) { return cudaSuccess; }

inline cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind) {
	if (dst && src && count) std::memcpy(dst, src, count);
	return cudaSuccess;
}

inline cudaError_t cudaMemcpyAsync(void* dst, const void* src, size_t count, cudaMemcpyKind, cudaStream_t = 0) {
	return cudaMemcpy(dst, src, count, cudaMemcpyDefault);
}

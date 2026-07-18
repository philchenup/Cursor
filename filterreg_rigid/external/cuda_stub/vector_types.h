#pragma once

// Minimal CUDA vector type stubs for CPU-only FilterReg builds.
// Enough for float2/float3/float4 used throughout the codebase.

#include <cstdint>

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#ifndef __forceinline__
#define __forceinline__ inline
#endif

struct __attribute__((aligned(8))) float2 {
	float x, y;
};

struct float3 {
	float x, y, z;
};

struct __attribute__((aligned(16))) float4 {
	float x, y, z, w;
};

struct __attribute__((aligned(4))) uchar4 {
	unsigned char x, y, z, w;
};

struct __attribute__((aligned(8))) int2 {
	int x, y;
};

struct int3 {
	int x, y, z;
};

struct __attribute__((aligned(16))) int4 {
	int x, y, z, w;
};

#pragma once

#include "vector_types.h"

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#ifndef __forceinline__
#define __forceinline__ inline
#endif

static inline __host__ __device__ float2 make_float2(float x, float y) {
	float2 t; t.x = x; t.y = y; return t;
}

static inline __host__ __device__ float3 make_float3(float x, float y, float z) {
	float3 t; t.x = x; t.y = y; t.z = z; return t;
}

static inline __host__ __device__ float4 make_float4(float x, float y, float z, float w) {
	float4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static inline __host__ __device__ uchar4 make_uchar4(unsigned char x, unsigned char y, unsigned char z, unsigned char w) {
	uchar4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

static inline __host__ __device__ int2 make_int2(int x, int y) {
	int2 t; t.x = x; t.y = y; return t;
}

static inline __host__ __device__ int3 make_int3(int x, int y, int z) {
	int3 t; t.x = x; t.y = y; t.z = z; return t;
}

static inline __host__ __device__ int4 make_int4(int x, int y, int z, int w) {
	int4 t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}

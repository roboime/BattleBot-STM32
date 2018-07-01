/*
 * vectors.h
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#ifndef VECTORS_H_
#define VECTORS_H_

#include <stdint.h>

typedef struct { int32_t x, y, z; } int32_vec3_t;
typedef struct { int64_t x, y, z; } int64_vec3_t;

inline static int64_vec3_t vec_to64(int32_vec3_t v)
{
	return (int64_vec3_t){ v.x, v.y, v.z };
}

inline static int32_vec3_t vec_to32(int64_vec3_t v)
{
	return (int32_vec3_t){ v.x, v.y, v.z };
}

inline static int32_vec3_t vec_upper_half(int64_vec3_t v)
{
	return (int32_vec3_t){ v.x>>32, v.y>>32, v.z>>32 };
}

int32_vec3_t vec_add32(int32_vec3_t v1, int32_vec3_t v2);
int64_vec3_t vec_add64(int64_vec3_t v1, int64_vec3_t v2);
int32_vec3_t vec_sub32(int32_vec3_t v1, int32_vec3_t v2);
int64_vec3_t vec_sub64(int64_vec3_t v1, int64_vec3_t v2);
int32_vec3_t vec_mul32(int32_t s, int32_vec3_t v);
int64_vec3_t vec_mul64(int64_t s, int64_vec3_t v);
int32_vec3_t vec_div32(int32_vec3_t v, int32_t s);
int64_vec3_t vec_div64(int64_vec3_t v, int64_t s);
int32_vec3_t vec_cross32(int32_vec3_t v1, int32_vec3_t v2);
int64_vec3_t vec_cross64(int64_vec3_t v1, int64_vec3_t v2);

#endif /* VECTORS_H_ */

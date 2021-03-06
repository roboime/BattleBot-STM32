/*
 * vectors.c
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#include "vectors.h"
#include "isqrt.h"

int32_vec3_t vec_add32(int32_vec3_t v1, int32_vec3_t v2)
{
	return (int32_vec3_t){ v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

int64_vec3_t vec_add64(int64_vec3_t v1, int64_vec3_t v2)
{
	return (int64_vec3_t){ v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

int32_vec3_t vec_sub32(int32_vec3_t v1, int32_vec3_t v2)
{
	return (int32_vec3_t){ v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

int64_vec3_t vec_sub64(int64_vec3_t v1, int64_vec3_t v2)
{
	return (int64_vec3_t){ v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

int32_vec3_t vec_mul32(int32_t s, int32_vec3_t v)
{
	return (int32_vec3_t){ s*v.x, s*v.y, s*v.z };
}

int64_vec3_t vec_mul64(int64_t s, int64_vec3_t v)
{
	return (int64_vec3_t){ s*v.x, s*v.y, s*v.z };
}

int32_vec3_t vec_div32(int32_vec3_t v, int32_t s)
{
	return (int32_vec3_t){ v.x/s, v.y/s, v.z/s };
}

int64_vec3_t vec_div64(int64_vec3_t v, int64_t s)
{
	return (int64_vec3_t){ v.x/s, v.y/s, v.z/s };
}

int32_vec3_t vec_cross32(int32_vec3_t v1, int32_vec3_t v2)
{
	return (int32_vec3_t)
	{
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

int64_vec3_t vec_cross64(int64_vec3_t v1, int64_vec3_t v2)
{
	return (int64_vec3_t)
	{
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

int32_vec3_t vec_shl32(int32_vec3_t v, uint8_t sh)
{
	return (int32_vec3_t){ v.x<<sh, v.y<<sh, v.z<<sh };
}

int64_vec3_t vec_shl64(int64_vec3_t v, uint8_t sh)
{
	return (int64_vec3_t){ v.x<<sh, v.y<<sh, v.z<<sh };
}

int32_vec3_t vec_shr32(int32_vec3_t v, uint8_t sh)
{
	return (int32_vec3_t){ v.x>>sh, v.y>>sh, v.z>>sh };
}

int64_vec3_t vec_shr64(int64_vec3_t v, uint8_t sh)
{
	return (int64_vec3_t){ v.x>>sh, v.y>>sh, v.z>>sh };
}

int64_t vec_sqnorm32(int32_vec3_t v)
{
	int64_t vi = vec_to64(v);
	return vi.x * vi.x + vi.y * vi.y + vi.z * vi.z;
}

int32_t vec_norm32(int32_vec3_t v)
{
	return isqrt64(vec_sqnorm32(v));
}

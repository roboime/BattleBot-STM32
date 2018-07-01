/*
 * quaternions.h
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

#include <stdint.h>

typedef struct { int32_t w, x, y, z; } int32_quat_t;
typedef struct { int64_t w, x, y, z; } int64_quat_t;

inline static int64_quat_t quat_to64(int32_quat_t q)
{
	return (int64_quat_t){ q.w, q.x, q.y, q.z };
}

inline static int32_quat_t quat_to32(int64_quat_t q)
{
	return (int32_quat_t){ q.w, q.x, q.y, q.z };
}

inline static int32_quat_t quat_upper_half(int64_quat_t q)
{
	return (int32_quat_t){ q.w>>32, q.x>>32, q.y>>32, q.z>>32 };
}

int32_quat_t quat_add32(int32_quat_t q1, int32_quat_t q2);
int64_quat_t quat_add64(int64_quat_t q1, int64_quat_t q2);
int32_quat_t quat_sub32(int32_quat_t q1, int32_quat_t q2);
int64_quat_t quat_sub64(int64_quat_t q1, int64_quat_t q2);
int32_quat_t quat_mulq32(int32_quat_t q1, int32_quat_t q2);
int64_quat_t quat_mulq64(int64_quat_t q1, int64_quat_t q2);
int32_quat_t quat_muls32(int32_t s, int32_quat_t q);
int64_quat_t quat_muls64(int64_t s, int64_quat_t q);

int32_quat_t quat_conj32(int32_quat_t q);
int64_quat_t quat_conj64(int64_quat_t q);

#endif /* QUATERNIONS_H_ */

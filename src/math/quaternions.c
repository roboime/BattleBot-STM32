/*
 * quaternions.c
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#include "quaternions.h"

int32_quat_t quat_add32(int32_quat_t q1, int32_quat_t q2)
{
	return (int32_quat_t){ q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z };
}

int64_quat_t quat_add64(int64_quat_t q1, int64_quat_t q2)
{
	return (int64_quat_t){ q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z };
}

int32_quat_t quat_sub32(int32_quat_t q1, int32_quat_t q2)
{
	return (int32_quat_t){ q1.w - q2.w, q1.x - q2.x, q1.y - q2.y, q1.z - q2.z };
}

int64_quat_t quat_sub64(int64_quat_t q1, int64_quat_t q2)
{
	return (int64_quat_t){ q1.w - q2.w, q1.x - q2.x, q1.y - q2.y, q1.z - q2.z };
}

int32_quat_t quat_mulq32(int32_quat_t q1, int32_quat_t q2)
{
	return (int32_quat_t)
	{
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
		q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x
	};
}

int64_quat_t quat_mulq64(int64_quat_t q1, int64_quat_t q2)
{
	return (int64_quat_t)
	{
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
		q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x
	};
}

int32_quat_t quat_muls32(int32_t s, int32_quat_t q)
{
	return (int32_quat_t){ s*q.w, s*q.x, s*q.y, s*q.z };
}

int64_quat_t quat_muls64(int64_t s, int64_quat_t q)
{
	return (int64_quat_t){ s*q.w, s*q.x, s*q.y, s*q.z };
}

int32_quat_t quat_conj32(int32_quat_t q)
{
	return (int32_quat_t){ q.w, -q.x, -q.y, -q.z };
}

int64_quat_t quat_conj64(int64_quat_t q)
{
	return (int64_quat_t){ q.w, -q.x, -q.y, -q.z };
}

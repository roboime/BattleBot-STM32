/*
 * isqrt.c
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"
#include "isqrt.h"

int32_t isqrt32(int32_t x)
{
	if (x < 0) return -1;
	if (!(x & ~1)) return x;

	int32_t y = 0;
	int32_t b = 1 << ((32 - __CLZ(x)) & ~1);

	while (b)
	{
		if (x >= y+b)
		{
			x -= y+b;
			y = (y >> 1) + b;
		}
		else y >>= 1;
		b >>= 2;
	}

	return y;
}

inline static uint8_t clz64(int64_t x)
{
	return x > 0x100000000LL ? __CLZ(x>>32) : 32 + __CLZ(x & 0xFFFFFFFF);
}

int64_t isqrt64(int64_t x)
{
	if (x < 0) return -1LL;
	if (!(x & ~1)) return x;

	int32_t y = 0;
	int32_t b = 1LL << ((64 - clz64(x)) & ~1);

	while (b)
	{
		if (x >= y+b)
		{
			x -= y+b;
			y = (y >> 1) + b;
		}
		else y >>= 1;
		b >>= 2;
	}

	return y;
}

#pragma once

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

typedef unsigned int uint;
typedef float scalar;
typedef unsigned char ubyte;

#define FP_MAX				FLT_MAX
#define FP_PRECISION		1e-4f

__forceinline bool isNaN(const float & val)
{
	return (val != val);
}

__forceinline bool isInf(const float & val)
{
	return (val > FLT_MAX || val < -FP_MAX);
}

__forceinline bool isSane(const float & val)
{
	return !isNaN(val) && !isInf(val);
}

__forceinline bool isDenorm(const float & val)
{
	return (val < FLT_EPSILON) && (val > -FLT_EPSILON);
}

inline bool isScalarLimited(const scalar & sc, const scalar & lim)
{
	bool result = true;
	if (sc > lim || sc < -lim)
		result = false;
	return result;
}

inline float randomN()
{
	return (rand() % 32768) / 32767.0f - 0.5f;
}

__forceinline uint rand30()
{
	return rand() | (rand() << 15);
}

template <class T>
inline T m_max(T val, T lim)
{
	return val > lim ? val : lim;
}
template <class T>
inline T m_min(T val, T lim)
{
	return val < lim ? val : lim;
}

template <class T>
inline T m_abs(T val)
{
	return (val < 0) ? (-val) : val;
}

inline int get2Pow(int num)
{
	int rpow = 0;
	while (num != 0)
	{
		if ((num & 1) != 0)
			return rpow;

		num >>= 1;
		rpow++;
	}
	return 0;
}


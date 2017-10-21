#pragma once

#include <math.h>

const float f_eps = 1e-5f;
const float PI = 3.141592653589793238462f;
const float _2PI = 6.283185307179586476925f;
const float _PI2 = 1.570796326794896619231f;

inline float fast_abs(float val)
{
	unsigned int temp = reinterpret_cast<unsigned int &>(val) & 0x7FFFFFFF;
	return reinterpret_cast<float &>(temp);
}

inline float fast_invSqrt(float val)
{
	union
	{
		int i;
		float f;
	} temp;

	temp.f = val;
	temp.i = 0x5f3759df - (temp.i >> 1);			// initial guess
	val *= 0.5f;									// half
	temp.f *= 1.5f - val * temp.f * temp.f;			// newton step, repeating this step will increase accuracy
	//temp.f *= 1.5f - val * temp.f * temp.f;

    return temp.f;
}

inline float Deg2Rad(float deg)
{
	return (deg / 180.0f) * PI;
}
inline float Rad2Deg(float rad)
{
	return (rad / PI) * 180.0f;
}

const float Deg2RadMul = _2PI / 360.0f;
const float Rad2DegMul = 360.0f / _2PI;

template <class T>
inline T sqr(T val)
{
	return val * val;
}

template <class T>
__forceinline T clamp(T val, T min, T max)
{
	if (val < min)
		val = min;
	else if (val > max)
		val = max;

	return val;
}

inline float m_log2(float n)
{
	// log(n) / log(2) is log2
	return logf(n) / logf(2.0f);
}

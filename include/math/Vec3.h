#pragma once

#include <math.h>

namespace math
{

class Vec3
{
public:

	union
	{
		struct
		{
			float x, y, z;
		};
		float v[3];
	};

	static Vec3 makeVec3(float x, float y, float z)
	{
		Vec3 temp;
		temp.x = x;
		temp.y = y;
		temp.z = z;
		return temp;
	}

	Vec3 operator + (const Vec3 & vec) const
	{
		return makeVec3(x + vec.x, y + vec.y, z + vec.z);
	}
	Vec3 operator - (const Vec3 & vec) const
	{
		return makeVec3(x - vec.x, y - vec.y, z - vec.z);
	}
	Vec3 operator - () const
	{
		return makeVec3(-x, -y, -z);
	}
	Vec3 & operator = (const Vec3 & vec)
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;
		return *this;
	}
	Vec3 & operator += (const Vec3 & vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return *this;
	}
	Vec3 & operator -= (const Vec3 & vec)
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return *this;
	}

	friend Vec3 operator * (const Vec3 & vec, const float & val)
	{
		return makeVec3(vec.x * val, vec.y * val, vec.z * val);
	}
	friend Vec3 operator * (const float & val, const Vec3 & vec)
	{
		return makeVec3(val * vec.x, val * vec.y, val * vec.z);
	}
	friend Vec3 operator / (const Vec3 & vec, const float & val)
	{
		return makeVec3(vec.x / val, vec.y / val, vec.z / val);
	}

	Vec3 & operator *= (const float & val)
	{
		x *= val;
		y *= val;
		z *= val;
		return *this;
	}
	Vec3 & operator /= (const float & val)
	{
		x /= val;
		y /= val;
		z /= val;
		return *this;
	}

	// functions
	float dot(const Vec3 & vec) const
	{
		return x*vec.x + y*vec.y + z*vec.z;
	}
	Vec3 cross(const Vec3 & vec) const
	{
		return makeVec3(
			y * vec.z - z * vec.y,
			z * vec.x - x * vec.z,
			x * vec.y - y * vec.x
			);
	}
	Vec3 mulHadamard(const Vec3 & vec) const
	{
		return makeVec3(x*vec.x, y*vec.y, z*vec.z);
	}
	float sqLen() const
	{
		return x*x + y*y + z*z;
	}
	float len() const
	{
		return sqrtf(x*x + y*y + z*z);
	}

	Vec3 & normalize()
	{
		float vecLen = len();

		const float eps = 1e-5f;
		if (vecLen < eps)
		{
			x = 0.0f;
			y = 1.0f;
			z = 0.0f;
		}
		else
		{
			x /= vecLen;
			y /= vecLen;
			z /= vecLen;
		}

		return *this;
	}
	Vec3 getNormalized() const
	{
		Vec3 vec(*this);
		return vec.normalize();
	}
	Vec3 & fastNormalize()
	{
		// TODO: implement fast normalization path via fast inv sqrt
		return normalize();
	}

	void tangentSpace(Vec3 & tan1, Vec3 & tan2) const
	{
		const float eps = 1e-5f;
		// Decide, what vector should we cross with
		if (x * x < z * z)
		{
			// Cross with vector (1 0 0)^T
			float vec_len = sqrtf(z * z + y * y);

			if (vec_len < eps)
			{
				return;
			}

			vec_len = 1.0f / vec_len;

			tan1.x = 0.0f;
			tan1.y = -vec_len * z;
			tan1.z =  vec_len * y;
		}
		else
		{
			// Cross with vector (0 0 1)^T
			float vec_len = sqrtf(y * y + x * x);

			if (vec_len < eps)
			{
				return;
			}

			vec_len = 1.0f / vec_len;

			tan1.x = -vec_len * y;
			tan1.y =  vec_len * x;
			tan1.z = 0.0f;
		}

		tan2 = cross(tan1);
		tan2.normalize();
	}
};

class Vec3C : public Vec3
{
public:

	Vec3C(float _x, float _y, float _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Vec3C(const Vec3 & vec)
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;
	}
};

}

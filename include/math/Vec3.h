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
	float triple(const Vec3 & v1, const Vec3 & v2) const
	{
		return dot(v1.cross(v2));
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

	Vec3C(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f)
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

// Calculate barycentric coordinates of a point within tiangle
static void computeBarycentric(
	const math::Vec3C & tv0,
	const math::Vec3C & tv1,
	const math::Vec3C & tv2,
	const math::Vec3C & p,
	float * bc0,
	float * bc1,
	float * bc2
	)
{
	using namespace math;

	Vec3 v0_p = p - tv0;
	Vec3 v0_v1 = tv1 - tv0;
	Vec3 v0_v2 = tv2 - tv0;

	float d01_01 = v0_v1.dot(v0_v1);
	float d01_02 = v0_v1.dot(v0_v2);
	float d02_02 = v0_v2.dot(v0_v2);

	float d0p_01 = v0_p.dot(v0_v1);
	float d0p_02 = v0_p.dot(v0_v2);

	float denom = d01_01 * d02_02 - d01_02 * d01_02;

	if (-1e-5f < denom && denom < 1e-5f)
	{
		*bc0 = 0.0f;
		*bc1 = 0.0f;
		*bc2 = 0.0f;
		return;
	}

	// Resulting point will have coordinates:
	//	p = bc0*tv0 + bc1*tv1 + bc2*tv2
	*bc1 = (d01_01 * d0p_02 - d01_02 * d0p_01) / denom;
	*bc2 = (d02_02 * d0p_01 - d01_02 * d0p_02) / denom;
	*bc0 = 1.0f - *bc1 - *bc2;
}

// Calculate barycentric coordinates of a point within tetrahedron
static void computeBarycentric(
	const math::Vec3C & tv0,
	const math::Vec3C & tv1,
	const math::Vec3C & tv2,
	const math::Vec3C & tv3,
	const math::Vec3C & p,
	float * bc0,
	float * bc1,
	float * bc2,
	float * bc3
	)
{
	using namespace math;

	Vec3 v0_p = p - tv0;
	Vec3 v1_p = p - tv1;

	Vec3 v0_v1 = tv1 - tv0;
	Vec3 v0_v2 = tv2 - tv0;
	Vec3 v0_v3 = tv3 - tv0;

	Vec3 v1_v2 = tv2 - tv1;
	Vec3 v1_v3 = tv3 - tv1;

	float vol = v0_v1.triple(v0_v2, v0_v3);
	if (-1e-5f < vol && vol < 1e-5f)
	{
		*bc0 = 0.0f;
		*bc1 = 0.0f;
		*bc2 = 0.0f;
		*bc3 = 0.0f;
		return;
	}
	float volInv = 1.0f / vol;
	*bc0 = volInv * v1_p.triple(v1_v3, v1_v2);
	*bc1 = volInv * v0_p.triple(v0_v2, v0_v3);
	*bc2 = volInv * v0_p.triple(v0_v3, v0_v1);
	*bc3 = volInv * v0_p.triple(v0_v1, v0_v2);
};

}

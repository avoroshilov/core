#pragma once

#include "math/AuxMath.h"
#include "math/Vec3.h"
#include "math/Mat33.h"
#include "math/Mat34.h"
#include "math/Mat44.h"

namespace math
{

class Quat
{
public:

	Vec3 v;		// Vector
	scalar s;	// Scalar

	// Constructors
	Quat(float _s, Vec3 _v): v(_v), s(_s) {}								// [scalar, vec]
	Quat(Vec3 _v, float _s = 0.0f): v(_v), s(_s) {}							// [vec, scalar]
	Quat(const Quat &_Quat): s(_Quat.s)										// copy
	{
		v = Vec3C(_Quat.v.x, _Quat.v.y, _Quat.v.z);
	}
	Quat() {}
	Quat(scalar _s, scalar _x, scalar _y, scalar _z):						// [s, x, y, z]
		s(_s)
	{
		v = Vec3C(_x, _y, _z);
	}

	float dot(const Quat & q) const
	{
		return v.x * q.v.x + v.y * q.v.y + v.z * q.v.z + s * q.s;
	}

	// Operators :: [x, y, z, s] concept
	Quat operator + (const Quat &addQ) const { return Quat(v + addQ.v, s + addQ.s); }
	Quat operator += (const Quat &addQ)
	{
		v += addQ.v; s += addQ.s;
		return *this;
	}

	Quat operator - (const Quat &subQ) const { return Quat(v - subQ.v, s - subQ.s); }
	Quat operator -= (const Quat &subQ)
	{
		v -= subQ.v; s -= subQ.s;
		return *this;
	}

	Quat operator * (const float &mulC) const { return Quat(v * mulC, s * mulC); }
	friend Quat operator * (const float &mulC, const Quat &q)
	{ return Quat(q.v * mulC, q.s * mulC); }

	Quat operator *= (const float &mulC)
	{
		v *= mulC; s *= mulC;
		return *this;
	}

	Quat operator / (const float &divC) const { return Quat(v / divC, s / divC); }

	Quat operator /= (const float &divC)
	{
		v /= divC; s /= divC;
		return *this;
	}

	// Operators :: [vector, s] concept
	Quat operator * (const Quat &mulQ) const
	{
#if 0
		return Quat(v.cross(mulQ.v) + v*mulQ.s + mulQ.v * s, (s*mulQ.s) - v.dot(mulQ.v));
#else
		// The above expression expanded for speed
		return Quat(
			mulQ.s*  s - mulQ.v.x*v.x - mulQ.v.y*v.y - mulQ.v.z*v.z,
			mulQ.s*v.x + mulQ.v.x*  s - mulQ.v.y*v.z + mulQ.v.z*v.y,
			mulQ.s*v.y + mulQ.v.x*v.z + mulQ.v.y*  s - mulQ.v.z*v.x,
			mulQ.s*v.z - mulQ.v.x*v.y + mulQ.v.y*v.x + mulQ.v.z*  s
			);
#endif
	}
	Quat operator *= (const Quat &mulQ)
	{
		scalar tmpS = s*mulQ.s - v.dot(mulQ.v);

		v = v.cross(mulQ.v) + v*mulQ.s + mulQ.v*s;
		s = tmpS;

		return *this;
	}

	// Norm && Magnitude
	scalar sqLen() const		{ return v.x * v.x + v.y * v.y + v.z * v.z + s * s; }
	scalar len() const			{ return sqrtf(sqLen()); }

	Quat normalize()
	{
		float mag = len();
		v.x /= mag;
		v.y /= mag;
		v.z /= mag;
		s /= mag;
		return *this;
	}

	// Conjugate :: [ -v, s ]
	Quat getConjugated() const		{ return Quat(-v, s); }

	// Inverse
	Quat getInversed() const		{ return getConjugated() / len(); }

	Vec3 rotate(const Vec3 & vec) const
	{
		// Given a vector vec=(x0, y0, z0) and a unit length quaternion q=(x, y, z, s), the vector
		//	vec'=(x1, y1, z1) which represents the rotation of vec by q is vec'= q*vec*q^{-1} where
		//	* indicates quaternion multiplication and where vec is treated as "pure" quaternion
		//	(x0, y0, z0, 0). Note that q^{-1} = <w, -x, -y, -z>, so no real work is required to
		//	invert q.
		//
		// q*vec*q^{-1} = q*(x0, y0, z0, 0)*q^{-1}
		//	= q*(x0*i+y0*j+z0*k)*q^{-1}
		//	= x0*(q*i*q^{-1})+y0*(q*j*q^{-1})+z0*(q*k*q^{-1})
		//
		// Converted back to the vector (by stripping zero scalar part), q*i*q^{-1}, q*j*q^{-1}, and
		//	q*k*q^{-1} are the columns of the rotation matrix computed in Quat::toMatrix33. The vector
		//	vec' is obtained as the product of that rotation matrix with vector vec. As such, the
		//	quaternion representation of a rotation matrix requires less space than the matrix and
		//	more time to compute the rotated vector (and easier to normalize).

		Vec3 rvec;
		// This is the rotation formula
		//rvec = (*this * Quat(vec, 0.0f) * getInversed()).v;
		// But we assume that quaternion is normalized, hence inverse could be replaced with conjugate
		rvec = (*this * Quat(vec, 0.0f) * getConjugated()).v;
		return rvec;
	}

	Quat & id() { v = Vec3C(0.0f, 0.0f, 0.0f); s = 1.0f; return *this; }

	// Converts UNIT Quaternion to Rotation Matrix
	static Mat33 toMatrix33(const Quat  &cQ)
	{ 
		Mat33 Res;

		scalar
			xx = cQ.v.x*cQ.v.x, yy = cQ.v.y*cQ.v.y, zz = cQ.v.z*cQ.v.z,
			xy = cQ.v.x*cQ.v.y, xz = cQ.v.x*cQ.v.z, yz = cQ.v.y*cQ.v.z,
			sx = cQ.s * cQ.v.x, sy = cQ.s * cQ.v.y, sz = cQ.s * cQ.v.z;

		Res._00 = 1.0f - 2 * (yy + zz);
		Res._01 =		 2 * (xy - sz);
		Res._02 =		 2 * (xz + sy);

		Res._10 =		 2 * (xy + sz);
		Res._11 = 1.0f - 2 * (xx + zz);
		Res._12 =		 2 * (yz - sx);

		Res._20 =		 2 * (xz - sy);
		Res._21 =		 2 * (yz + sx);
		Res._22 = 1.0f - 2 * (xx + yy);

		return Res;
	}
	// Converts Rotation Matrix to UNIT Quaternion
	static Quat fromMatrix33(const Mat33 & cM)
	{
		Quat result;

		scalar trace = cM._00 + cM._11 + cM._22 + 1.0f, s;

		if (trace > f_eps)
		{
			result.s = 0.5f * sqrtf(trace);
			s = 1.0f / (4.0f*result.s);
			result.v.x = (cM.t[2][1] - cM.t[1][2]) * s;
			result.v.y = (cM.t[0][2] - cM.t[2][0]) * s;
			result.v.z = (cM.t[1][0] - cM.t[0][1]) * s;
		}
		else
		{
			if ((cM.t[0][0] > cM.t[1][1]) && (cM.t[0][0] > cM.t[2][2]))
			{
				s = 2.0f * sqrtf(1.0f + cM.t[0][0] - cM.t[1][1] - cM.t[2][2]);

				result.v.x = 0.25f * s;
				result.v.y = (cM.t[0][1] + cM.t[1][0]) / s;
				result.v.z = (cM.t[0][2] + cM.t[2][0]) / s;
				result.s   = (cM.t[2][1] - cM.t[1][2]) / s;
			}
			else if (cM.t[1][1] > cM.t[2][2])
			{
				s = 2.0f * sqrtf(1.0f + cM.t[1][1] - cM.t[0][0] - cM.t[2][2]);

				result.v.x = (cM.t[0][1] + cM.t[1][0]) / s;
				result.v.y = 0.25f * s;
				result.v.z = (cM.t[1][2] + cM.t[2][1]) / s;
				result.s   = (cM.t[0][2] - cM.t[2][0]) / s;
			}
			else
			{
				s = 2.0f * sqrtf(1.0f + cM.t[2][2] - cM.t[0][0] - cM.t[1][1]);

				result.v.x = (cM.t[0][2] + cM.t[2][0]) / s;
				result.v.y = (cM.t[1][2] + cM.t[2][1]) / s;
				result.v.z = 0.25f * s;
				result.s   = (cM.t[1][0] - cM.t[0][1]) / s;
			}
		}
		
		return result;
	}
	void toAxisAngle(Vec3 * axis, scalar * angle)
	{
		// The quaternion representing the rotation is
		//	q = sin(A/2)*(x*i+y*j+z*k) + cos(A/2)

		scalar sqrLength = v.sqLen();
		if (sqrLength > 0.0)
		{
			*angle = 2.0f * acosf(s);
			scalar invLen = 1.0f / sqrtf(sqrLength);
			*axis = v * invLen;
		}
		else
		{
			// angle is 0 (mod 2*pi), so any axis will do
			*angle = 0.0f;
			*axis = Vec3C(1.0f, 0.0f, 0.0f);
		}
	}
	static Quat fromAxisAngle(const Vec3 & axis, scalar angleRad)
	{
		return Quat(axis * sinf(angleRad * 0.5f), cosf(angleRad * 0.5f));
	}

	static Quat sLerp(const Quat & p, const Quat & q, float t)
	{
		// Formula from "Animating Rotation with Quaternion Curves" by K. Shoemake, originally by G. Davis
		//	slerp(p, q, t) = sin((1-t)*theta)/sin(theta) * p + sin(t*theta)/sin(theta) * q
		//	p*q = cos(theta) => theta = arccos(p*q)

		scalar cosTheta = p.dot(q);
		scalar coeff0 = 1.0f, coeff1 = 1.0f;

		if (cosTheta < 0)
		{
			cosTheta = -cosTheta;
			coeff1 = -1.0f;
		}

		scalar theta = acos(cosTheta);

		const float quatSLerpEps = 1e-6f;
		if (fast_abs(theta) < quatSLerpEps)
			return p;

		scalar sinTheta = sinf(theta);
		scalar invSinTheta = 1.0f / sinTheta;

		coeff0 *= sinf((1.0f - t) * theta) * invSinTheta;
		coeff1 *= sinf(t * theta) * invSinTheta;

		return p * coeff0 + q * coeff1;
	}

	static Quat sQuad(scalar t, const Quat & p0, const Quat & p1, const Quat & q0, const Quat & q1)
	{
		// squad(p0, p1, q0, q1, t) = slerp( slerp(p0, p1, t), slerp(q0, q1, t), 2*t(1 - t) )
		Quat slerp_p = sLerp(p0, p1, t);
		Quat slerp_q = sLerp(q0, q1, t);

		return sLerp(slerp_p, slerp_q, 2.f*t * (1.f - t));
	}
};

}

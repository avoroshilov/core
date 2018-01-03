#pragma once

#include "math/Vec3.h"
#include "math/Vec4.h"
#include "math/Mat34.h"
#include "math/AuxMath.h"

typedef float scalar;

namespace math
{

class Mat44
{
public:

	union
	{
		struct
		{
			scalar _00, _01, _02, _03;
			scalar _10, _11, _12, _13;
			scalar _20, _21, _22, _23;
			scalar _30, _31, _32, _33;
		};

		scalar m[16];
		scalar t[4][4];
	};

	Mat44() { }
	Mat44(scalar _00_,		  scalar _01_ = 0.0f, scalar _02_ = 0.0f, scalar _03_ = 0.0f,
			scalar _10_ = 0.0f, scalar _11_ = 0.0f, scalar _12_ = 0.0f, scalar _13_ = 0.0f,
			scalar _20_ = 0.0f, scalar _21_ = 0.0f, scalar _22_ = 0.0f, scalar _23_ = 0.0f,
			scalar _30_ = 0.0f, scalar _31_ = 0.0f, scalar _32_ = 0.0f, scalar _33_ = 0.0f) :
			_00(_00_), _01(_01_), _02(_02_), _03(_03_),
			_10(_10_), _11(_11_), _12(_12_), _13(_13_),
			_20(_20_), _21(_21_), _22(_22_), _23(_23_),
			_30(_30_), _31(_31_), _32(_32_), _33(_33_)
	{
	}

	Mat44(const Mat44 & mat) :
			_00(mat._00), _01(mat._01), _02(mat._02), _03(mat._03),
			_10(mat._10), _11(mat._11), _12(mat._12), _13(mat._13),
			_20(mat._20), _21(mat._21), _22(mat._22), _23(mat._23),
			_30(mat._30), _31(mat._31), _32(mat._32), _33(mat._33)
	{
	}

	Mat44 & identity()
	{
		_00 = 1.0f; _01 = 0.0f; _02 = 0.0f; _03 = 0.0f;
		_10 = 0.0f; _11 = 1.0f; _12 = 0.0f; _13 = 0.0f;
		_20 = 0.0f; _21 = 0.0f; _22 = 1.0f; _23 = 0.0f;
		_30 = 0.0f; _31 = 0.0f; _32 = 0.0f; _33 = 1.0f;

		return *this;
	}
	Mat44 & zero()
	{
		_00 = 0.0f; _01 = 0.0f; _02 = 0.0f; _03 = 0.0f;
		_10 = 0.0f; _11 = 0.0f; _12 = 0.0f; _13 = 0.0f;
		_20 = 0.0f; _21 = 0.0f; _22 = 0.0f; _23 = 0.0f;
		_30 = 0.0f; _31 = 0.0f; _32 = 0.0f; _33 = 0.0f;

		return *this;
	}

	// Build matrix from basis vectors (vector == column)
	Mat44(const Vec3 & v0, const Vec3 & v1, const Vec3 & v2, const Vec3 & v3) :
			_00(v0.x), _01(v1.x), _02(v2.x), _03(v3.x),
			_10(v0.y), _11(v1.y), _12(v2.y), _13(v3.y),
			_20(v0.z), _21(v1.z), _22(v2.z), _23(v3.z),
			_30(0.0f), _31(0.0f), _32(0.0f), _33(1.0f)
	{
	}
	Mat44(const Vec3 & v0, const Vec3 & v1, const Vec3 & v2) :
			_00(v0.x), _01(v1.x), _02(v2.x), _03(0.0f),
			_10(v0.y), _11(v1.y), _12(v2.y), _13(0.0f),
			_20(v0.z), _21(v1.z), _22(v2.z), _23(0.0f),
			_30(0.0f), _31(0.0f), _32(0.0f), _33(1.0f)
	{
	}

	Vec4 getBasis0() const { return Vec4C(_00, _10, _20, _30); }
	Vec4 getBasis1() const { return Vec4C(_01, _11, _21, _31); }
	Vec4 getBasis2() const { return Vec4C(_02, _12, _22, _32); }
	Vec4 getBasis3() const { return Vec4C(_03, _13, _23, _33); }

	void setBasis0(const Vec3 & vec) { _00 = vec.x; _10 = vec.y; _20 = vec.z; }
	void setBasis1(const Vec3 & vec) { _01 = vec.x; _11 = vec.y; _21 = vec.z; }
	void setBasis2(const Vec3 & vec) { _02 = vec.x; _12 = vec.y; _22 = vec.z; }
	void setBasis3(const Vec3 & vec) { _03 = vec.x; _13 = vec.y; _23 = vec.z; }

	void setBasis0(const Vec4 & vec) { _00 = vec.x; _10 = vec.y; _20 = vec.z; _30 = vec.w; }
	void setBasis1(const Vec4 & vec) { _01 = vec.x; _11 = vec.y; _21 = vec.z; _31 = vec.w; }
	void setBasis2(const Vec4 & vec) { _02 = vec.x; _12 = vec.y; _22 = vec.z; _32 = vec.w; }
	void setBasis3(const Vec4 & vec) { _03 = vec.x; _13 = vec.y; _23 = vec.z; _33 = vec.w; }

	Mat44 operator + (const Mat44 & mat) const
	{
		return Mat44(
				_00 + mat._00, _01 + mat._01, _02 + mat._02, _03 + mat._03,
				_10 + mat._10, _11 + mat._11, _12 + mat._12, _13 + mat._13,
				_20 + mat._20, _21 + mat._21, _22 + mat._22, _23 + mat._23,
				_30 + mat._30, _31 + mat._31, _32 + mat._32, _33 + mat._33
				);
	}
	Mat44 operator - (const Mat44 & mat) const
	{
		return Mat44(
				_00 - mat._00, _01 - mat._01, _02 - mat._02, _03 - mat._03,
				_10 - mat._10, _11 - mat._11, _12 - mat._12, _13 - mat._13,
				_20 - mat._20, _21 - mat._21, _22 - mat._22, _23 - mat._23,
				_30 - mat._30, _31 - mat._31, _32 - mat._32, _33 - mat._33
				);
	}

	Mat44 & operator = (const Mat44 & mat)
	{
		_00 = mat._00; _01 = mat._01; _02 = mat._02; _03 = mat._03;
		_10 = mat._10; _11 = mat._11; _12 = mat._12; _13 = mat._13;
		_20 = mat._20; _21 = mat._21; _22 = mat._22; _23 = mat._23;
		_30 = mat._30; _31 = mat._31; _32 = mat._32; _33 = mat._33;

		return *this;
	}
	Mat44 & operator = (const Mat34 & mat34)
	{
		_00 = mat34._00; _01 = mat34._01; _02 = mat34._02; _03 = mat34._03;
		_10 = mat34._10; _11 = mat34._11; _12 = mat34._12; _13 = mat34._13;
		_20 = mat34._20; _21 = mat34._21; _22 = mat34._22; _23 = mat34._23;
		_30 = 0.0f; _31 = 0.0f; _32 = 0.0f; _33 = 1.0f;

		return *this;
	}
	Mat44 & operator = (const Mat33 & mat33)
	{
		_00 = mat33._00; _01 = mat33._01; _02 = mat33._02; _03 = 0.0f;
		_10 = mat33._10; _11 = mat33._11; _12 = mat33._12; _13 = 0.0f;
		_20 = mat33._20; _21 = mat33._21; _22 = mat33._22; _23 = 0.0f;
		_30 = 0.0f; _31 = 0.0f; _32 = 0.0f; _33 = 1.0f;

		return *this;
	}

	Mat44 operator * (const Mat44 & mat) const
	{
		Mat44 retMat;

#if 0
		*(retMat.m   ) = *(m   ) * *(mat.m   ) + *(m+ 1) * *(mat.m+ 4) +
							*(m+ 2) * *(mat.m+ 8) + *(m+ 3) * *(mat.m+12);
		*(retMat.m+ 1) = *(m   ) * *(mat.m+ 1) + *(m+ 1) * *(mat.m+ 5) +
							*(m+ 2) * *(mat.m+ 9) + *(m+ 3) * *(mat.m+13);
		*(retMat.m+ 2) = *(m   ) * *(mat.m+ 2) + *(m+ 1) * *(mat.m+ 6) +
							*(m+ 2) * *(mat.m+10) + *(m+ 3) * *(mat.m+14);
		*(retMat.m+ 3) = *(m   ) * *(mat.m+ 3) + *(m+ 1) * *(mat.m+ 7) +
							*(m+ 2) * *(mat.m+11) + *(m+ 3) * *(mat.m+15);

		*(retMat.m+ 4) = *(m+ 4) * *(mat.m   ) + *(m+ 5) * *(mat.m+ 4) +
							*(m+ 6) * *(mat.m+ 8) + *(m+ 7) * *(mat.m+12);
		*(retMat.m+ 5) = *(m+ 4) * *(mat.m+ 1) + *(m+ 5) * *(mat.m+ 5) +
							*(m+ 6) * *(mat.m+ 9) + *(m+ 7) * *(mat.m+13);
		*(retMat.m+ 6) = *(m+ 4) * *(mat.m+ 2) + *(m+ 5) * *(mat.m+ 6) +
							*(m+ 6) * *(mat.m+10) + *(m+ 7) * *(mat.m+14);
		*(retMat.m+ 7) = *(m+ 4) * *(mat.m+ 3) + *(m+ 5) * *(mat.m+ 7) +
							*(m+ 6) * *(mat.m+11) + *(m+ 7) * *(mat.m+15);

		*(retMat.m+ 8) = *(m+ 8) * *(mat.m   ) + *(m+ 9) * *(mat.m+ 4) +
							*(m+10) * *(mat.m+ 8) + *(m+11) * *(mat.m+12);
		*(retMat.m+ 9) = *(m+ 8) * *(mat.m+ 1) + *(m+ 9) * *(mat.m+ 5) +
							*(m+10) * *(mat.m+ 9) + *(m+11) * *(mat.m+13);
		*(retMat.m+10) = *(m+ 8) * *(mat.m+ 2) + *(m+ 9) * *(mat.m+ 6) +
							*(m+10) * *(mat.m+10) + *(m+11) * *(mat.m+14);
		*(retMat.m+11) = *(m+ 8) * *(mat.m+ 3) + *(m+ 9) * *(mat.m+ 7) +
							*(m+10) * *(mat.m+11) + *(m+11) * *(mat.m+15);

		*(retMat.m+12) = *(m+12) * *(mat.m   ) + *(m+13) * *(mat.m+ 4) +
							*(m+14) * *(mat.m+ 8) + *(m+15) * *(mat.m+12);
		*(retMat.m+13) = *(m+12) * *(mat.m+ 1) + *(m+13) * *(mat.m+ 5) +
							*(m+14) * *(mat.m+ 9) + *(m+15) * *(mat.m+13);
		*(retMat.m+14) = *(m+12) * *(mat.m+ 2) + *(m+13) * *(mat.m+ 6) +
							*(m+14) * *(mat.m+10) + *(m+15) * *(mat.m+14);
		*(retMat.m+15) = *(m+12) * *(mat.m+ 3) + *(m+13) * *(mat.m+ 7) +
							*(m+14) * *(mat.m+11) + *(m+15) * *(mat.m+15);
#else
		// More readable notation

		// Row #0
		retMat._00 = _00 * mat._00 + _01 * mat._10 + _02 * mat._20 + _03 * mat._30;
		retMat._01 = _00 * mat._01 + _01 * mat._11 + _02 * mat._21 + _03 * mat._31;
		retMat._02 = _00 * mat._02 + _01 * mat._12 + _02 * mat._22 + _03 * mat._32;
		retMat._03 = _00 * mat._03 + _01 * mat._13 + _02 * mat._23 + _03 * mat._33;

		// Row #1
		retMat._10 = _10 * mat._00 + _11 * mat._10 + _12 * mat._20 + _13 * mat._30;
		retMat._11 = _10 * mat._01 + _11 * mat._11 + _12 * mat._21 + _13 * mat._31;
		retMat._12 = _10 * mat._02 + _11 * mat._12 + _12 * mat._22 + _13 * mat._32;
		retMat._13 = _10 * mat._03 + _11 * mat._13 + _12 * mat._23 + _13 * mat._33;

		// Row #2
		retMat._20 = _20 * mat._00 + _21 * mat._10 + _22 * mat._20 + _23 * mat._30;
		retMat._21 = _20 * mat._01 + _21 * mat._11 + _22 * mat._21 + _23 * mat._31;
		retMat._22 = _20 * mat._02 + _21 * mat._12 + _22 * mat._22 + _23 * mat._32;
		retMat._23 = _20 * mat._03 + _21 * mat._13 + _22 * mat._23 + _23 * mat._33;

		// Row #3
		retMat._30 = _30 * mat._00 + _31 * mat._10 + _32 * mat._20 + _33 * mat._30;
		retMat._31 = _30 * mat._01 + _31 * mat._11 + _32 * mat._21 + _33 * mat._31;
		retMat._32 = _30 * mat._02 + _31 * mat._12 + _32 * mat._22 + _33 * mat._32;
		retMat._33 = _30 * mat._03 + _31 * mat._13 + _32 * mat._23 + _33 * mat._33;
#endif
		
		return retMat;
	}

	friend inline Mat44 operator * (const Mat44 & mat, const scalar & val)
	{
		return Mat44(
				mat._00 * val, mat._01 * val, mat._02 * val, mat._03 * val,
				mat._10 * val, mat._11 * val, mat._12 * val, mat._13 * val,
				mat._20 * val, mat._21 * val, mat._22 * val, mat._23 * val,
				mat._30 * val, mat._31 * val, mat._32 * val, mat._33 * val
				);
	}
	friend inline Mat44 operator * (const scalar & val, const Mat44 & mat)
	{
		return Mat44(
				mat._00 * val, mat._01 * val, mat._02 * val, mat._03 * val,
				mat._10 * val, mat._11 * val, mat._12 * val, mat._13 * val,
				mat._20 * val, mat._21 * val, mat._22 * val, mat._23 * val,
				mat._30 * val, mat._31 * val, mat._32 * val, mat._33 * val
				);
	}
	friend inline Mat44 operator / (const Mat44 & mat, const scalar & val)
	{
		return Mat44(
				mat._00 / val, mat._01 / val, mat._02 / val, mat._03 / val,
				mat._10 / val, mat._11 / val, mat._12 / val, mat._13 / val,
				mat._20 / val, mat._21 / val, mat._22 / val, mat._23 / val,
				mat._30 / val, mat._31 / val, mat._32 / val, mat._33 / val
				);
	}
	Mat44 & operator *= (const scalar & val)
	{
		_00 *= val; _01 *= val; _02 *= val; _03 *= val;
		_10 *= val; _11 *= val; _12 *= val; _13 *= val;
		_20 *= val; _21 *= val; _22 *= val; _23 *= val;
		_30 *= val; _31 *= val; _32 *= val; _33 *= val;

		return *this;
	}
	Mat44 & operator /= (const scalar & val)
	{
		_00 /= val; _01 /= val; _02 /= val; _03 /= val;
		_10 /= val; _11 /= val; _12 /= val; _13 /= val;
		_20 /= val; _21 /= val; _22 /= val; _23 /= val;
		_30 /= val; _31 /= val; _32 /= val; _33 /= val;

		return *this;
	}

	Vec3 operator * (const Vec3 & vec) const
	{
		return Vec3C(
				_00 * vec.x + _01 * vec.y + _02 * vec.z + _03,
				_10 * vec.x + _11 * vec.y + _12 * vec.z + _13,
				_20 * vec.x + _21 * vec.y + _22 * vec.z + _23
				);
	}

	Vec4 operator * (const Vec4 & vec) const
	{
		return Vec4C(
				_00 * vec.x + _01 * vec.y + _02 * vec.z + _03 * vec.w,
				_10 * vec.x + _11 * vec.y + _12 * vec.z + _13 * vec.w,
				_20 * vec.x + _21 * vec.y + _22 * vec.z + _23 * vec.w,
				_30 * vec.x + _31 * vec.y + _32 * vec.z + _33 * vec.w
				);
	}

	friend inline Mat44 operator * (const Mat44 & mat, const Mat34 & mat34)
	{
		Mat44 ret;

		ret._00 = mat._00 * mat34._00 + mat._01 * mat34._10 + mat._02 * mat34._20;
		ret._01 = mat._00 * mat34._01 + mat._01 * mat34._11 + mat._02 * mat34._21;
		ret._02 = mat._00 * mat34._02 + mat._01 * mat34._12 + mat._02 * mat34._22;
		ret._03 = mat._00 * mat34._03 + mat._01 * mat34._13 + mat._02 * mat34._23 + mat._03;

		ret._10 = mat._10 * mat34._00 + mat._11 * mat34._10 + mat._12 * mat34._20;
		ret._11 = mat._10 * mat34._01 + mat._11 * mat34._11 + mat._12 * mat34._21;
		ret._12 = mat._10 * mat34._02 + mat._11 * mat34._12 + mat._12 * mat34._22;
		ret._13 = mat._10 * mat34._03 + mat._11 * mat34._13 + mat._12 * mat34._23 + mat._13;

		ret._20 = mat._20 * mat34._00 + mat._21 * mat34._10 + mat._22 * mat34._20;
		ret._21 = mat._20 * mat34._01 + mat._21 * mat34._11 + mat._22 * mat34._21;
		ret._22 = mat._20 * mat34._02 + mat._21 * mat34._12 + mat._22 * mat34._22;
		ret._23 = mat._20 * mat34._03 + mat._21 * mat34._13 + mat._22 * mat34._23 + mat._23;

		ret._30 = mat._30 * mat34._00 + mat._31 * mat34._10 + mat._32 * mat34._20;
		ret._31 = mat._30 * mat34._01 + mat._31 * mat34._11 + mat._32 * mat34._21;
		ret._32 = mat._30 * mat34._02 + mat._31 * mat34._12 + mat._32 * mat34._22;
		ret._33 = mat._30 * mat34._03 + mat._31 * mat34._13 + mat._32 * mat34._23 + mat._33;

		return ret;
	}

	// functions
	Mat44 & transpose()
	{
		scalar temp;

#define MAT44_SWAP_ELTS(e0, e1) \
			temp = e0;\
			e0 = e1;\
			e1 = temp;

		MAT44_SWAP_ELTS(_01, _10)
		MAT44_SWAP_ELTS(_02, _20)
		MAT44_SWAP_ELTS(_03, _30)

		MAT44_SWAP_ELTS(_12, _21)
		MAT44_SWAP_ELTS(_13, _31)

		MAT44_SWAP_ELTS(_23, _32)

#undef MAT44_SWAP_ELTS

		return *this;
	}
	Mat44 getTransposed() const
	{
		Mat44 mat(*this);
		return mat.transpose();
	}

	Vec3 transformCopy(const Vec3 & vec) const
	{
		Vec3 vecCopy;
		vecCopy.x = _00 * vec.x + _01 * vec.y + _02 * vec.z + _03;
		vecCopy.y = _10 * vec.x + _11 * vec.y + _12 * vec.z + _13;
		vecCopy.z = _20 * vec.x + _21 * vec.y + _22 * vec.z + _23;
		return vecCopy;
	}
	Vec3 & transform(Vec3 & vec) const
	{
		Vec3 vecCopy(vec);
		vec.x = _00 * vecCopy.x + _01 * vecCopy.y + _02 * vecCopy.z + _03;
		vec.y = _10 * vecCopy.x + _11 * vecCopy.y + _12 * vecCopy.z + _13;
		vec.z = _20 * vecCopy.x + _21 * vecCopy.y + _22 * vecCopy.z + _23;
		return vec;
	}
	Vec3 & rotate(Vec3 & vec) const
	{
		Vec3 copy(vec);

		vec.x = _00 * copy.x + _01 * copy.y + _02 * copy.z;
		vec.y = _10 * copy.x + _11 * copy.y + _12 * copy.z;
		vec.z = _20 * copy.x + _21 * copy.y + _22 * copy.z;

		return vec;
	}
	Vec3 rotateCopy(const Vec3 & vec) const
	{
		Vec3 copy(vec);

		copy.x = _00 * vec.x + _01 * vec.y + _02 * vec.z;
		copy.y = _10 * vec.x + _11 * vec.y + _12 * vec.z;
		copy.z = _20 * vec.x + _21 * vec.y + _22 * vec.z;

		return copy;
	}
	Vec3 & invRotate(Vec3 & vec) const
	{
		Vec3 copy(vec);

		vec.x = _00 * copy.x + _10 * copy.y + _20 * copy.z;
		vec.y = _01 * copy.x + _11 * copy.y + _21 * copy.z;
		vec.z = _02 * copy.x + _12 * copy.y + _22 * copy.z;

		return vec;
	}
	Vec3 invRotateCopy(const Vec3 & vec) const
	{
		Vec3 copy;

		copy.x = _00 * vec.x + _10 * vec.y + _20 * vec.z;
		copy.y = _01 * vec.x + _11 * vec.y + _21 * vec.z;
		copy.z = _02 * vec.x + _12 * vec.y + _22 * vec.z;

		return copy;
	}

	Vec3 homogTransformCopy(const Vec3 & vec) const
	{
		Vec3 vecCopy;
		vecCopy.x = _00 * vec.x + _01 * vec.y + _02 * vec.z + _03;
		vecCopy.y = _10 * vec.x + _11 * vec.y + _12 * vec.z + _13;
		vecCopy.z = _20 * vec.x + _21 * vec.y + _22 * vec.z + _23;

		scalar w = _30 * vec.x + _31 * vec.y + _32 * vec.z + _33;
		vecCopy.x /= w;
		vecCopy.y /= w;
		vecCopy.z /= w;

		return vecCopy;
	}
	Vec3 & homogTransform(Vec3 & vec) const
	{
		Vec3 vecCopy(vec);
		vec.x = _00 * vecCopy.x + _01 * vecCopy.y + _02 * vecCopy.z + _03;
		vec.y = _10 * vecCopy.x + _11 * vecCopy.y + _12 * vecCopy.z + _13;
		vec.z = _20 * vecCopy.x + _21 * vecCopy.y + _22 * vecCopy.z + _23;
		
		scalar w = (_30 * vecCopy.x + _31 * vecCopy.y + _32 * vecCopy.z + _33);

		w = 1.0f / w;
		vec.x *= w;
		vec.y *= w;
		vec.z *= w;
		
		return vec;
	}
	Vec3 & homogTransform(Vec3 & vec, scalar & w) const
	{
		Vec3 vecCopy(vec);
		vec.x = _00 * vecCopy.x + _01 * vecCopy.y + _02 * vecCopy.z + _03;
		vec.y = _10 * vecCopy.x + _11 * vecCopy.y + _12 * vecCopy.z + _13;
		vec.z = _20 * vecCopy.x + _21 * vecCopy.y + _22 * vecCopy.z + _23;
		
		w = (_30 * vecCopy.x + _31 * vecCopy.y + _32 * vecCopy.z + _33);
		//w = 1.0f / w;
		//vec.x *= w;
		//vec.y *= w;
		//vec.z *= w;
		
		return vec;
	}

	Mat44 invertRTCopy() const
	{
		/*
			If matrix is represented as
			R	T
			0	1

			then its inverse is
			R^T		-(R^T * T)
			0		1
		*/

		Mat44 result;

		// Inv rotation
		result._00 = _00;
		result._01 = _10;
		result._02 = _20;
		result._10 = _01;
		result._11 = _11;
		result._12 = _21;
		result._20 = _02;
		result._21 = _12;
		result._22 = _22;

		result._30 = 0.0f;
		result._31 = 0.0f;
		result._32 = 0.0f;

		// Inv translation
		result._03 = -(result._00 * _03 + result._01 * _13 + result._02 * _23);
		result._13 = -(result._10 * _03 + result._11 * _13 + result._12 * _23);
		result._23 = -(result._20 * _03 + result._21 * _13 + result._22 * _23);
		result._33 = 1.0f;

		return result;
	}

	scalar determinant() const
	{
		scalar A, B, C, D, E, F;

		A = m[10] * m[15] - m[11] * m[14];
		B = m[ 9] * m[15] - m[11] * m[13];
		C = m[ 9] * m[14] - m[10] * m[13];
		D = m[ 8] * m[15] - m[11] * m[12];
		E = m[ 8] * m[14] - m[10] * m[12];
		F = m[ 8] * m[13] - m[ 9] * m[12];

		return m[0] * (A * m[5] - B * m[6] + C * m[7])
				- m[1] * (A * m[4] - D * m[6] + E * m[7])
				+ m[2] * (B * m[4] - D * m[5] + F * m[7])
				- m[3] * (C * m[4] - E * m[5] + F * m[6]);
	}

	Mat44 & invert()
	{
		scalar det = 1.0f / determinant();

		#define MDET3(a0,a1,a2,a3,a4,a5,a6,a7,a8) \
					( a0 * (a4 * a8 - a5 * a7) \
					- a1 * (a3 * a8 - a5 * a6) \
					+ a2 * (a3 * a7 - a4 * a6) )

		Mat44 temp(*this);

		// ~ 160 mul & 80 add
		m[ 0] =  MDET3(temp.m[5], temp.m[6], temp.m[7], temp.m[9], temp.m[10], temp.m[11], temp.m[13], temp.m[14], temp.m[15]) * det;
		m[ 1] = -MDET3(temp.m[1], temp.m[2], temp.m[3], temp.m[9], temp.m[10], temp.m[11], temp.m[13], temp.m[14], temp.m[15]) * det;
		m[ 2] =  MDET3(temp.m[1], temp.m[2], temp.m[3], temp.m[5], temp.m[ 6], temp.m[ 7], temp.m[13], temp.m[14], temp.m[15]) * det;
		m[ 3] = -MDET3(temp.m[1], temp.m[2], temp.m[3], temp.m[5], temp.m[ 6], temp.m[ 7], temp.m[ 9], temp.m[10], temp.m[11]) * det;
		m[ 4] = -MDET3(temp.m[4], temp.m[6], temp.m[7], temp.m[8], temp.m[10], temp.m[11], temp.m[12], temp.m[14], temp.m[15]) * det;
		m[ 5] =  MDET3(temp.m[0], temp.m[2], temp.m[3], temp.m[8], temp.m[10], temp.m[11], temp.m[12], temp.m[14], temp.m[15]) * det;
		m[ 6] = -MDET3(temp.m[0], temp.m[2], temp.m[3], temp.m[4], temp.m[ 6], temp.m[ 7], temp.m[12], temp.m[14], temp.m[15]) * det;
		m[ 7] =  MDET3(temp.m[0], temp.m[2], temp.m[3], temp.m[4], temp.m[ 6], temp.m[ 7], temp.m[ 8], temp.m[10], temp.m[11]) * det;
		m[ 8] =  MDET3(temp.m[4], temp.m[5], temp.m[7], temp.m[8], temp.m[ 9], temp.m[11], temp.m[12], temp.m[13], temp.m[15]) * det;
		m[ 9] = -MDET3(temp.m[0], temp.m[1], temp.m[3], temp.m[8], temp.m[ 9], temp.m[11], temp.m[12], temp.m[13], temp.m[15]) * det;
		m[10] =  MDET3(temp.m[0], temp.m[1], temp.m[3], temp.m[4], temp.m[ 5], temp.m[ 7], temp.m[12], temp.m[13], temp.m[15]) * det;
		m[11] = -MDET3(temp.m[0], temp.m[1], temp.m[3], temp.m[4], temp.m[ 5], temp.m[ 7], temp.m[ 8], temp.m[ 9], temp.m[11]) * det;
		m[12] = -MDET3(temp.m[4], temp.m[5], temp.m[6], temp.m[8], temp.m[ 9], temp.m[10], temp.m[12], temp.m[13], temp.m[14]) * det;
		m[13] =  MDET3(temp.m[0], temp.m[1], temp.m[2], temp.m[8], temp.m[ 9], temp.m[10], temp.m[12], temp.m[13], temp.m[14]) * det;
		m[14] = -MDET3(temp.m[0], temp.m[1], temp.m[2], temp.m[4], temp.m[ 5], temp.m[ 6], temp.m[12], temp.m[13], temp.m[14]) * det;
		m[15] =  MDET3(temp.m[0], temp.m[1], temp.m[2], temp.m[4], temp.m[ 5], temp.m[ 6], temp.m[ 8], temp.m[ 9], temp.m[10]) * det;

		#undef MDET3

		return *this;
	}

	Mat44 getInverse() const
	{
		scalar det = 1.0f / determinant();

		#define MDET3(a0,a1,a2,a3,a4,a5,a6,a7,a8) \
					( a0 * (a4 * a8 - a5 * a7) \
					- a1 * (a3 * a8 - a5 * a6) \
					+ a2 * (a3 * a7 - a4 * a6) )

		// ~ 160 mul & 80 add
		return Mat44(
				MDET3(m[5], m[6], m[7], m[9], m[10], m[11], m[13], m[14], m[15]) * det,
			-MDET3(m[1], m[2], m[3], m[9], m[10], m[11], m[13], m[14], m[15]) * det,
				MDET3(m[1], m[2], m[3], m[5], m[ 6], m[ 7], m[13], m[14], m[15]) * det,
			-MDET3(m[1], m[2], m[3], m[5], m[ 6], m[ 7], m[ 9], m[10], m[11]) * det,
			-MDET3(m[4], m[6], m[7], m[8], m[10], m[11], m[12], m[14], m[15]) * det,
				MDET3(m[0], m[2], m[3], m[8], m[10], m[11], m[12], m[14], m[15]) * det,
			-MDET3(m[0], m[2], m[3], m[4], m[ 6], m[ 7], m[12], m[14], m[15]) * det,
				MDET3(m[0], m[2], m[3], m[4], m[ 6], m[ 7], m[ 8], m[10], m[11]) * det,
				MDET3(m[4], m[5], m[7], m[8], m[ 9], m[11], m[12], m[13], m[15]) * det,
			-MDET3(m[0], m[1], m[3], m[8], m[ 9], m[11], m[12], m[13], m[15]) * det,
				MDET3(m[0], m[1], m[3], m[4], m[ 5], m[ 7], m[12], m[13], m[15]) * det,
			-MDET3(m[0], m[1], m[3], m[4], m[ 5], m[ 7], m[ 8], m[ 9], m[11]) * det,
			-MDET3(m[4], m[5], m[6], m[8], m[ 9], m[10], m[12], m[13], m[14]) * det,
				MDET3(m[0], m[1], m[2], m[8], m[ 9], m[10], m[12], m[13], m[14]) * det,
			-MDET3(m[0], m[1], m[2], m[4], m[ 5], m[ 6], m[12], m[13], m[14]) * det,
				MDET3(m[0], m[1], m[2], m[4], m[ 5], m[ 6], m[ 8], m[ 9], m[10]) * det);

		#undef MDET3
	}

	// Vector Axis must be normalized for correct Rotating [ RAD ]
	///////////////////////////////////////////////////////////////////////
	void fillRotation(const Vec3 & axis, float radAngle)
	{
		float cf = cosf(radAngle),
				sf = sinf(radAngle);

		float xx = sqr(axis.x),
				yy = sqr(axis.y),
				zz = sqr(axis.z);
		float xy = axis.x * axis.y,
				yz = axis.y * axis.z,
				zx = axis.z * axis.x;

		_00 = xx + cf * (1.0f - xx);
		_01 = xy - cf * xy - sf * axis.z;
		_02 = zx - cf * zx + sf * axis.y;

		_10 = xy - cf * xy + sf * axis.z;
		_11 = yy + cf * (1.0f - yy);
		_12 = yz - cf * yz - sf * axis.x;

		_20 = zx - cf * zx - sf * axis.y;
		_21 = yz - cf * yz + sf * axis.x;
		_22 = zz + cf * (1.0f - zz);
	}
	void fillTranslation(const Vec3 & vec)
	{
		_03 = vec.x;
		_13 = vec.y;
		_23 = vec.z;
	}
	void fillScale(const Vec3 & vec)
	{
		_00 = vec.x;
		_11 = vec.y;
		_22 = vec.z;
	}

	Mat44 & rotate(const Vec3 & axis, float radAngle)
	{
		Mat44 rotMat;
		rotMat.fillRotation(axis, radAngle);
		rotMat.fillTranslation(Vec3C(0.0f, 0.0f, 0.0f));
		rotMat._30 = 0.0f;
		rotMat._31 = 0.0f;
		rotMat._32 = 0.0f;
		rotMat._33 = 1.0f;

		*this = *this * rotMat;

		return *this;
	}
	Mat44 & translate(const Vec3 & vec)
	{
		Mat44 tranMat;
		tranMat.identity();
		tranMat.fillTranslation(vec);

		*this = *this * tranMat;

		return *this;
	}
	Mat44 & scale(const Vec3 & vec)
	{
		Mat44 scMat(0.0f);
		scMat.identity();
		scMat.fillScale(vec);

		*this = *this * scMat;

		return *this;
	}

	static const Mat44 Id()
	{
		static Mat44 idMatrix44(1.0f, 0.0f, 0.0f, 0.0f,
								0.0f, 1.0f, 0.0f, 0.0f,
								0.0f, 0.0f, 1.0f, 0.0f,
								0.0f, 0.0f, 0.0f, 1.0f);
		return idMatrix44;
	}
};

}

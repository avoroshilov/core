#pragma once

#include "helpers/common.h"
#include "math/AuxMath.h"
#include "math/Vec3.h"

namespace math
{

class Mat34
{
public:

	union
	{
		struct
		{
			float _00, _01, _02, _03;
			float _10, _11, _12, _13;
			float _20, _21, _22, _23;
		};

		float m[12];
		float t[3][4];
	};

	Mat34() { }
	Mat34(float _00_,		 float _01_ = 0.0f, float _02_ = 0.0f, float _03_ = 0.0f,
			float _10_ = 0.0f, float _11_ = 0.0f, float _12_ = 0.0f, float _13_ = 0.0f,
			float _20_ = 0.0f, float _21_ = 0.0f, float _22_ = 0.0f, float _23_ = 0.0f):
			_00(_00_), _01(_01_), _02(_02_), _03(_03_),
			_10(_10_), _11(_11_), _12(_12_), _13(_13_),
			_20(_20_), _21(_21_), _22(_22_), _23(_23_)
	{
	}

	Mat34(const Mat34 & mat) :
			_00(mat._00), _01(mat._01), _02(mat._02), _03(mat._03),
			_10(mat._10), _11(mat._11), _12(mat._12), _13(mat._13),
			_20(mat._20), _21(mat._21), _22(mat._22), _23(mat._23)
	{
	}

	Mat34 & identity()
	{
		_00 = 1.0f; _01 = 0.0f; _02 = 0.0f; _03 = 0.0f;
		_10 = 0.0f; _11 = 1.0f; _12 = 0.0f; _13 = 0.0f;
		_20 = 0.0f; _21 = 0.0f; _22 = 1.0f; _23 = 0.0f;

		return *this;
	}
	Mat34 & zero()
	{
		_00 = 0.0f; _01 = 0.0f; _02 = 0.0f; _03 = 0.0f;
		_10 = 0.0f; _11 = 0.0f; _12 = 0.0f; _13 = 0.0f;
		_20 = 0.0f; _21 = 0.0f; _22 = 0.0f; _23 = 0.0f;

		return *this;
	}

	// Build matrix from basis vectors (vector == column)
	Mat34(const Vec3 & v0, const Vec3 & v1, const Vec3 & v2, const Vec3 & v3) :
			_00(v0.x), _01(v1.x), _02(v2.x), _03(v3.x),
			_10(v0.y), _11(v1.y), _12(v2.y), _13(v3.y),
			_20(v0.z), _21(v1.z), _22(v2.z), _23(v3.z)
	{
	}
	Mat34(const Vec3 & v0, const Vec3 & v1, const Vec3 & v2) :
			_00(v0.x), _01(v1.x), _02(v2.x), _03(0.0f),
			_10(v0.y), _11(v1.y), _12(v2.y), _13(0.0f),
			_20(v0.z), _21(v1.z), _22(v2.z), _23(0.0f)
	{
	}

	Vec3 GetBasis0() { return Vec3C(_00, _10, _20); }
	Vec3 GetBasis1() { return Vec3C(_01, _11, _21); }
	Vec3 GetBasis2() { return Vec3C(_02, _12, _22); }
	Vec3 GetBasis3() { return Vec3C(_03, _13, _23); }

	Mat34 operator + (const Mat34 & mat) const
	{
		return Mat34(
				_00 + mat._00, _01 + mat._01, _02 + mat._02, _03 + mat._03,
				_10 + mat._10, _11 + mat._11, _12 + mat._12, _13 + mat._13,
				_20 + mat._20, _21 + mat._21, _22 + mat._22, _23 + mat._23
				);
	}
	Mat34 & operator += (const Mat34 & mat)
	{
		_00 = _00 + mat._00; _01 = _01 + mat._01; _02 = _02 + mat._02; _03 = _03 + mat._03;
		_10 = _10 + mat._10; _11 = _11 + mat._11; _12 = _12 + mat._12; _13 = _13 + mat._13;
		_20 = _20 + mat._20; _21 = _21 + mat._21; _22 = _22 + mat._22; _23 = _23 + mat._23;

		return *this;
	}

	Mat34 operator - (const Mat34 & mat) const
	{
		return Mat34(
				_00 - mat._00, _01 - mat._01, _02 - mat._02, _03 - mat._03,
				_10 - mat._10, _11 - mat._11, _12 - mat._12, _13 - mat._13,
				_20 - mat._20, _21 - mat._21, _22 - mat._22, _23 - mat._23
				);
	}
	Mat34 & operator -= (const Mat34 & mat)
	{
		_00 = _00 - mat._00; _01 = _01 - mat._01; _02 = _02 - mat._02; _03 = _03 - mat._03;
		_10 = _10 - mat._10; _11 = _11 - mat._11; _12 = _12 - mat._12; _13 = _13 - mat._13;
		_20 = _20 - mat._20; _21 = _21 - mat._21; _22 = _22 - mat._22; _23 = _23 - mat._23;

		return *this;
	}

	Mat34 & operator = (const Mat34 & mat)
	{
		_00 = mat._00; _01 = mat._01; _02 = mat._02; _03 = mat._03;
		_10 = mat._10; _11 = mat._11; _12 = mat._12; _13 = mat._13;
		_20 = mat._20; _21 = mat._21; _22 = mat._22; _23 = mat._23;

		return *this;
	}

	Mat34 & operator *= (const Mat34 & mat)
	{
		Mat34 tempMat(*this);

		*(m   ) = *(tempMat.m   ) * *(mat.m   ) + *(tempMat.m+ 1) * *(mat.m+ 4) +
					*(tempMat.m+ 2) * *(mat.m+ 8);
		*(m+ 1) = *(tempMat.m   ) * *(mat.m+ 1) + *(tempMat.m+ 1) * *(mat.m+ 5) +
					*(tempMat.m+ 2) * *(mat.m+ 9);
		*(m+ 2) = *(tempMat.m   ) * *(mat.m+ 2) + *(tempMat.m+ 1) * *(mat.m+ 6) +
					*(tempMat.m+ 2) * *(mat.m+10);
		*(m+ 3) = *(tempMat.m   ) * *(mat.m+ 3) + *(tempMat.m+ 1) * *(mat.m+ 7) +
					*(tempMat.m+ 2) * *(mat.m+11) + *(tempMat.m+ 3);

		*(m+ 4) = *(tempMat.m+ 4) * *(mat.m   ) + *(tempMat.m+ 5) * *(mat.m+ 4) +
					*(tempMat.m+ 6) * *(mat.m+ 8);
		*(m+ 5) = *(tempMat.m+ 4) * *(mat.m+ 1) + *(tempMat.m+ 5) * *(mat.m+ 5) +
					*(tempMat.m+ 6) * *(mat.m+ 9);
		*(m+ 6) = *(tempMat.m+ 4) * *(mat.m+ 2) + *(tempMat.m+ 5) * *(mat.m+ 6) +
					*(tempMat.m+ 6) * *(mat.m+10);
		*(m+ 7) = *(tempMat.m+ 4) * *(mat.m+ 3) + *(tempMat.m+ 5) * *(mat.m+ 7) +
					*(tempMat.m+ 6) * *(mat.m+11) + *(tempMat.m+ 7);

		*(m+ 8) = *(tempMat.m+ 8) * *(mat.m   ) + *(tempMat.m+ 9) * *(mat.m+ 4) +
					*(tempMat.m+10) * *(mat.m+ 8);
		*(m+ 9) = *(tempMat.m+ 8) * *(mat.m+ 1) + *(tempMat.m+ 9) * *(mat.m+ 5) +
					*(tempMat.m+10) * *(mat.m+ 9);
		*(m+10) = *(tempMat.m+ 8) * *(mat.m+ 2) + *(tempMat.m+ 9) * *(mat.m+ 6) +
					*(tempMat.m+10) * *(mat.m+10);
		*(m+11) = *(tempMat.m+ 8) * *(mat.m+ 3) + *(tempMat.m+ 9) * *(mat.m+ 7) +
					*(tempMat.m+10) * *(mat.m+11) + *(tempMat.m+11);
	
		return *this;
	}
	Mat34 operator * (const Mat34 & mat) const
	{
		Mat34 retMat;

		*(retMat.m   ) = *(m   ) * *(mat.m   ) + *(m+ 1) * *(mat.m+ 4) +
							*(m+ 2) * *(mat.m+ 8);
		*(retMat.m+ 1) = *(m   ) * *(mat.m+ 1) + *(m+ 1) * *(mat.m+ 5) +
							*(m+ 2) * *(mat.m+ 9);
		*(retMat.m+ 2) = *(m   ) * *(mat.m+ 2) + *(m+ 1) * *(mat.m+ 6) +
							*(m+ 2) * *(mat.m+10);
		*(retMat.m+ 3) = *(m   ) * *(mat.m+ 3) + *(m+ 1) * *(mat.m+ 7) +
							*(m+ 2) * *(mat.m+11) + *(m+ 3);

		*(retMat.m+ 4) = *(m+ 4) * *(mat.m   ) + *(m+ 5) * *(mat.m+ 4) +
							*(m+ 6) * *(mat.m+ 8);
		*(retMat.m+ 5) = *(m+ 4) * *(mat.m+ 1) + *(m+ 5) * *(mat.m+ 5) +
							*(m+ 6) * *(mat.m+ 9);
		*(retMat.m+ 6) = *(m+ 4) * *(mat.m+ 2) + *(m+ 5) * *(mat.m+ 6) +
							*(m+ 6) * *(mat.m+10);
		*(retMat.m+ 7) = *(m+ 4) * *(mat.m+ 3) + *(m+ 5) * *(mat.m+ 7) +
							*(m+ 6) * *(mat.m+11) + *(m+ 7);

		*(retMat.m+ 8) = *(m+ 8) * *(mat.m   ) + *(m+ 9) * *(mat.m+ 4) +
							*(m+10) * *(mat.m+ 8);
		*(retMat.m+ 9) = *(m+ 8) * *(mat.m+ 1) + *(m+ 9) * *(mat.m+ 5) +
							*(m+10) * *(mat.m+ 9);
		*(retMat.m+10) = *(m+ 8) * *(mat.m+ 2) + *(m+ 9) * *(mat.m+ 6) +
							*(m+10) * *(mat.m+10);
		*(retMat.m+11) = *(m+ 8) * *(mat.m+ 3) + *(m+ 9) * *(mat.m+ 7) +
							*(m+10) * *(mat.m+11) + *(m+11);
	
		return retMat;
	}


	friend inline Mat34 operator * (const Mat34 & mat, const float & val)
	{
		return Mat34(
				mat._00 * val, mat._01 * val, mat._02 * val, mat._03 * val,
				mat._10 * val, mat._11 * val, mat._12 * val, mat._13 * val,
				mat._20 * val, mat._21 * val, mat._22 * val, mat._23 * val
				);
	}
	friend inline Mat34 operator * (const float & val, const Mat34 & mat)
	{
		return Mat34(
				mat._00 * val, mat._01 * val, mat._02 * val, mat._03 * val,
				mat._10 * val, mat._11 * val, mat._12 * val, mat._13 * val,
				mat._20 * val, mat._21 * val, mat._22 * val, mat._23 * val
				);
	}
	friend inline Mat34 operator / (const Mat34 & mat, const float & val)
	{
		return Mat34(
				mat._00 / val, mat._01 / val, mat._02 / val, mat._03 / val,
				mat._10 / val, mat._11 / val, mat._12 / val, mat._13 / val,
				mat._20 / val, mat._21 / val, mat._22 / val, mat._23 / val
				);
	}

	Mat34 & operator *= (const float & val)
	{
		_00 *= val; _01 *= val; _02 *= val; _03 *= val;
		_10 *= val; _11 *= val; _12 *= val; _13 *= val;
		_20 *= val; _21 *= val; _22 *= val; _23 *= val;

		return *this;
	}
	Mat34 & operator /= (const float & val)
	{
		_00 /= val; _01 /= val; _02 /= val; _03 /= val;
		_10 /= val; _11 /= val; _12 /= val; _13 /= val;
		_20 /= val; _21 /= val; _22 /= val; _23 /= val;

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

	// functions
	scalar det33() const
	{
		return (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);
	}

	Mat34 & buildFromDirection(const Vec3 & direction)
	{
		identity();
		Vec3 tan1, tan2;
		direction.tangentSpace(tan2, tan1);
		_00 = tan1.x;
		_10 = tan1.y;
		_20 = tan1.z;
		_01 = direction.x;
		_11 = direction.y;
		_21 = direction.z;
		_02 = tan2.x;
		_12 = tan2.y;
		_22 = tan2.z;

		return *this;
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
	Vec3 & invTransformRT(Vec3 & vec) const
	{
		/*
		General formula:
			A - matrix itself
			R - 3x3 rotational block ( inv(R) == transpose(R) )
			t - 3x1 translational vector
		inv(A) * x = [inv(R) * (x - t)]
		*/

		Vec3 diff(vec);
		diff.x -= _03;
		diff.y -= _13;
		diff.z -= _23;
	
		vec.x = _00 * diff.x + _10 * diff.y + _20 * diff.z;
		vec.y = _01 * diff.x + _11 * diff.y + _21 * diff.z;
		vec.z = _02 * diff.x + _12 * diff.y + _22 * diff.z;

		return vec;
	}
	Vec3 invTransformRTCopy(const Vec3 & vec) const
	{
		Vec3 vecCopy(vec);
		return invTransformRT(vecCopy);
	}
	Vec3 & invTransform(Vec3 & vec) const
	{
		/*
		General formula:
			A - matrix itself
			R - 3x3 block
			t - 3x1 translational vector
		inv(A) * x = [inv(R) * (x - t)]
		*/

		Vec3 diff(vec);
		diff.x -= _03;
		diff.y -= _13;
		diff.z -= _23;
	
		float invDet33 = 1.0f / (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);

		vec.x = invDet33 * (_11*_22 - _12*_21) * diff.x + invDet33 * (_02*_21 - _01*_22) * diff.y + invDet33 * (_01*_12 - _02*_11) * diff.z;
		vec.y = invDet33 * (_12*_20 - _10*_22) * diff.x + invDet33 * (_00*_22 - _02*_20) * diff.y + invDet33 * (_02*_10 - _00*_12) * diff.z;
		vec.z = invDet33 * (_10*_21 - _11*_20) * diff.x + invDet33 * (_01*_20 - _00*_21) * diff.y + invDet33 * (_00*_11 - _01*_10) * diff.z;

		return vec;
	}
	Vec3 invTransformCopy(const Vec3 & vec) const
	{
		Vec3 vecCopy(vec);
		return invTransform(vecCopy);
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

	Vec3 & transformNormal(Vec3 & normal)
	{
		// transformNormal(M, n) = (M^-1T) * n
		float invDet33 = 1.0f / (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);
		Vec3 cnormal = invDet33 * normal;

		normal.x = (_11*_22 - _12*_21) * cnormal.x + (_12*_20 - _10*_22) * cnormal.y + (_10*_21 - _11*_20) * cnormal.z;
		normal.y = (_02*_21 - _01*_22) * cnormal.x + (_00*_22 - _02*_20) * cnormal.y + (_01*_20 - _00*_21) * cnormal.z;
		normal.z = (_01*_12 - _02*_11) * cnormal.x + (_02*_10 - _00*_12) * cnormal.y + (_00*_11 - _01*_10) * cnormal.z;

		return normal;
	}

	void invert33()
	{
		Mat34 tempCopy(*this);

		float invDet33 = 1.0f / (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);

		tempCopy._00 = invDet33 * (_11*_22 - _12*_21);
		tempCopy._01 = invDet33 * (_02*_21 - _01*_22);
		tempCopy._02 = invDet33 * (_01*_12 - _02*_11);

		tempCopy._10 = invDet33 * (_12*_20 - _10*_22);
		tempCopy._11 = invDet33 * (_00*_22 - _02*_20);
		tempCopy._12 = invDet33 * (_02*_10 - _00*_12);

		tempCopy._20 = invDet33 * (_10*_21 - _11*_20);
		tempCopy._21 = invDet33 * (_01*_20 - _00*_21);
		tempCopy._22 = invDet33 * (_00*_11 - _01*_10);

		*this = tempCopy;
	}

	Mat34 getTransposed33() const
	{
		Mat34 tempCopy;

		tempCopy._00 = _00;
		tempCopy._01 = _10;
		tempCopy._02 = _20;

		tempCopy._10 = _01;
		tempCopy._11 = _11;
		tempCopy._12 = _21;

		tempCopy._20 = _02;
		tempCopy._21 = _12;
		tempCopy._22 = _22;

		return tempCopy;
	}

	// Vector Axis must be normalized for correct Rotation [ RAD ]
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

	Mat34 & rotate(const Vec3 & axis, float radAngle)
	{
		Mat34 rotMat;
		rotMat.fillRotation(axis, radAngle);
		rotMat.fillTranslation(Vec3C(0.0f, 0.0f, 0.0f));

		*this = *this * rotMat;

		return *this;
	}
	Mat34 & translate(const Vec3 & vec)
	{
		Mat34 tranMat;
		tranMat.identity();
		tranMat.fillTranslation(vec);

		*this = *this * tranMat;

		return *this;
	}
	Mat34 & scale(const Vec3 & vec)
	{
		Mat34 scMat(0.0f);
		scMat.identity();
		scMat.fillScale(vec);

		*this = *this * scMat;

		return *this;
	}

	Mat34 getRotation() const
	{
		Mat34 rotMatrix(0.0f);

		rotMatrix._00 = _00;
		rotMatrix._01 = _01;
		rotMatrix._02 = _02;
		rotMatrix._10 = _10;
		rotMatrix._11 = _11;
		rotMatrix._12 = _12;
		rotMatrix._20 = _20;
		rotMatrix._21 = _21;
		rotMatrix._22 = _22;

		return rotMatrix;
	}

	Vec3 getTranslation() const
	{
		return Vec3C(_03, _13, _23);
	}
};

}

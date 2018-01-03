#pragma once

#include "helpers/common.h"
#include "math/AuxMath.h"
#include "math/Vec3.h"

namespace math
{

class Mat33
{
public:

	union
	{
		struct
		{
			float _00, _01, _02;
			float _10, _11, _12;
			float _20, _21, _22;
		};

		float m[9];
		float t[3][3];
	};

	Mat33() { }
	Mat33(float _00_,		 float _01_ = 0.0f, float _02_ = 0.0f,
			float _10_ = 0.0f, float _11_ = 0.0f, float _12_ = 0.0f,
			float _20_ = 0.0f, float _21_ = 0.0f, float _22_ = 0.0f):
			_00(_00_), _01(_01_), _02(_02_),
			_10(_10_), _11(_11_), _12(_12_),
			_20(_20_), _21(_21_), _22(_22_)
	{
	}

	Mat33(const Mat33 & mat) :
			_00(mat._00), _01(mat._01), _02(mat._02),
			_10(mat._10), _11(mat._11), _12(mat._12),
			_20(mat._20), _21(mat._21), _22(mat._22)
	{
	}

	Mat33 & identity()
	{
		_00 = 1.0f; _01 = 0.0f; _02 = 0.0f;
		_10 = 0.0f; _11 = 1.0f; _12 = 0.0f;
		_20 = 0.0f; _21 = 0.0f; _22 = 1.0f;

		return *this;
	}
	Mat33 & zero()
	{
		_00 = 0.0f; _01 = 0.0f; _02 = 0.0f;
		_10 = 0.0f; _11 = 0.0f; _12 = 0.0f;
		_20 = 0.0f; _21 = 0.0f; _22 = 0.0f;

		return *this;
	}

	// Build matrix from basis vectors (vector == column)
	Mat33(const Vec3 & v0, const Vec3 & v1, const Vec3 & v2) :
			_00(v0.x), _01(v1.x), _02(v2.x),
			_10(v0.y), _11(v1.y), _12(v2.y),
			_20(v0.z), _21(v1.z), _22(v2.z)
	{
	}

	Vec3 getBasis0() const { return Vec3C(_00, _10, _20); }
	Vec3 getBasis1() const { return Vec3C(_01, _11, _21); }
	Vec3 getBasis2() const { return Vec3C(_02, _12, _22); }

	void setBasis0(const Vec3 & vec) { _00 = vec.x; _10 = vec.y; _20 = vec.z; }
	void setBasis1(const Vec3 & vec) { _01 = vec.x; _11 = vec.y; _21 = vec.z; }
	void setBasis2(const Vec3 & vec) { _02 = vec.x; _12 = vec.y; _22 = vec.z; }

	Mat33 operator + (const Mat33 & mat) const
	{
		return Mat33(
				_00 + mat._00, _01 + mat._01, _02 + mat._02,
				_10 + mat._10, _11 + mat._11, _12 + mat._12,
				_20 + mat._20, _21 + mat._21, _22 + mat._22
				);
	}
	Mat33 & operator += (const Mat33 & mat)
	{
		_00 = _00 + mat._00; _01 = _01 + mat._01; _02 = _02 + mat._02;
		_10 = _10 + mat._10; _11 = _11 + mat._11; _12 = _12 + mat._12;
		_20 = _20 + mat._20; _21 = _21 + mat._21; _22 = _22 + mat._22;

		return *this;
	}

	Mat33 operator - (const Mat33 & mat) const
	{
		return Mat33(
				_00 - mat._00, _01 - mat._01, _02 - mat._02,
				_10 - mat._10, _11 - mat._11, _12 - mat._12,
				_20 - mat._20, _21 - mat._21, _22 - mat._22
				);
	}
	Mat33 & operator -= (const Mat33 & mat)
	{
		_00 = _00 - mat._00; _01 = _01 - mat._01; _02 = _02 - mat._02;
		_10 = _10 - mat._10; _11 = _11 - mat._11; _12 = _12 - mat._12;
		_20 = _20 - mat._20; _21 = _21 - mat._21; _22 = _22 - mat._22;

		return *this;
	}

	Mat33 & operator = (const Mat33 & mat)
	{
		_00 = mat._00; _01 = mat._01; _02 = mat._02;
		_10 = mat._10; _11 = mat._11; _12 = mat._12;
		_20 = mat._20; _21 = mat._21; _22 = mat._22;

		return *this;
	}

	Mat33 & operator *= (const Mat33 & mat)
	{
		Mat33 tempMat(*this);

		_00 = tempMat._00 * mat._00 + tempMat._01 * mat._10 + tempMat._02 * mat._20;
		_01 = tempMat._00 * mat._01 + tempMat._01 * mat._11 + tempMat._02 * mat._21;
		_02 = tempMat._00 * mat._02 + tempMat._01 * mat._12 + tempMat._02 * mat._22;

		_10 = tempMat._10 * mat._00 + tempMat._11 * mat._10 + tempMat._12 * mat._20;
		_11 = tempMat._10 * mat._01 + tempMat._11 * mat._11 + tempMat._12 * mat._21;
		_12 = tempMat._10 * mat._02 + tempMat._11 * mat._12 + tempMat._12 * mat._22;

		_20 = tempMat._20 * mat._00 + tempMat._21 * mat._10 + tempMat._22 * mat._20;
		_21 = tempMat._20 * mat._01 + tempMat._21 * mat._11 + tempMat._22 * mat._21;
		_22 = tempMat._20 * mat._02 + tempMat._21 * mat._12 + tempMat._22 * mat._22;
	
		return *this;
	}
	Mat33 operator * (const Mat33 & mat) const
	{
		Mat33 retMat;

		retMat._00 = _00 * mat._00 + _01 * mat._10 + _02 * mat._20;
		retMat._01 = _00 * mat._01 + _01 * mat._11 + _02 * mat._21;
		retMat._02 = _00 * mat._02 + _01 * mat._12 + _02 * mat._22;

		retMat._10 = _10 * mat._00 + _11 * mat._10 + _12 * mat._20;
		retMat._11 = _10 * mat._01 + _11 * mat._11 + _12 * mat._21;
		retMat._12 = _10 * mat._02 + _11 * mat._12 + _12 * mat._22;

		retMat._20 = _20 * mat._00 + _21 * mat._10 + _22 * mat._20;
		retMat._21 = _20 * mat._01 + _21 * mat._11 + _22 * mat._21;
		retMat._22 = _20 * mat._02 + _21 * mat._12 + _22 * mat._22;

		return retMat;
	}


	friend inline Mat33 operator * (const Mat33 & mat, const float & val)
	{
		return Mat33(
				mat._00 * val, mat._01 * val, mat._02 * val,
				mat._10 * val, mat._11 * val, mat._12 * val,
				mat._20 * val, mat._21 * val, mat._22 * val
				);
	}
	friend inline Mat33 operator * (const float & val, const Mat33 & mat)
	{
		return Mat33(
				mat._00 * val, mat._01 * val, mat._02 * val,
				mat._10 * val, mat._11 * val, mat._12 * val,
				mat._20 * val, mat._21 * val, mat._22 * val
				);
	}
	friend inline Mat33 operator / (const Mat33 & mat, const float & val)
	{
		return Mat33(
				mat._00 / val, mat._01 / val, mat._02 / val,
				mat._10 / val, mat._11 / val, mat._12 / val,
				mat._20 / val, mat._21 / val, mat._22 / val
				);
	}

	Mat33 & operator *= (const float & val)
	{
		_00 *= val; _01 *= val; _02 *= val;
		_10 *= val; _11 *= val; _12 *= val;
		_20 *= val; _21 *= val; _22 *= val;

		return *this;
	}
	Mat33 & operator /= (const float & val)
	{
		_00 /= val; _01 /= val; _02 /= val;
		_10 /= val; _11 /= val; _12 /= val;
		_20 /= val; _21 /= val; _22 /= val;

		return *this;
	}

	Vec3 operator * (const Vec3 & vec) const
	{
		return Vec3C(
				_00 * vec.x + _01 * vec.y + _02 * vec.z,
				_10 * vec.x + _11 * vec.y + _12 * vec.z,
				_20 * vec.x + _21 * vec.y + _22 * vec.z
				);
	}

	// functions
	scalar det() const
	{
		return (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);
	}

	Mat33 & buildFromDirection(const Vec3 & direction)
	{
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

	// Skew-symmetric cross product matrix
	Mat33 & crossProdMat(const Vec3 & vec)
	{
		_00 = 0.0f;
		_01 = -vec.z;
		_02 = vec.y;
		_10 = vec.z;
		_11 = 0.0f;
		_12 = -vec.x;
		_20 = -vec.y;
		_21 = vec.x;
		_22 = 0.0f;

		return *this;
	}

	Vec3 transformCopy(const Vec3 & vec) const
	{
		Vec3 vecCopy;
		vecCopy.x = _00 * vec.x + _01 * vec.y + _02 * vec.z;
		vecCopy.y = _10 * vec.x + _11 * vec.y + _12 * vec.z;
		vecCopy.z = _20 * vec.x + _21 * vec.y + _22 * vec.z;
		return vecCopy;
	}
	Vec3 & transform(Vec3 & vec) const
	{
		Vec3 vecCopy(vec);
		vec.x = _00 * vecCopy.x + _01 * vecCopy.y + _02 * vecCopy.z;
		vec.y = _10 * vecCopy.x + _11 * vecCopy.y + _12 * vecCopy.z;
		vec.z = _20 * vecCopy.x + _21 * vecCopy.y + _22 * vecCopy.z;
		return vec;
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

		float invDet = 1.0f / (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);

		vec.x = invDet * (_11*_22 - _12*_21) * vec.x + invDet * (_02*_21 - _01*_22) * vec.y + invDet * (_01*_12 - _02*_11) * vec.z;
		vec.y = invDet * (_12*_20 - _10*_22) * vec.x + invDet * (_00*_22 - _02*_20) * vec.y + invDet * (_02*_10 - _00*_12) * vec.z;
		vec.z = invDet * (_10*_21 - _11*_20) * vec.x + invDet * (_01*_20 - _00*_21) * vec.y + invDet * (_00*_11 - _01*_10) * vec.z;

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
		float invDet = 1.0f / (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);
		Vec3 cnormal = invDet * normal;

		normal.x = (_11*_22 - _12*_21) * cnormal.x + (_12*_20 - _10*_22) * cnormal.y + (_10*_21 - _11*_20) * cnormal.z;
		normal.y = (_02*_21 - _01*_22) * cnormal.x + (_00*_22 - _02*_20) * cnormal.y + (_01*_20 - _00*_21) * cnormal.z;
		normal.z = (_01*_12 - _02*_11) * cnormal.x + (_02*_10 - _00*_12) * cnormal.y + (_00*_11 - _01*_10) * cnormal.z;

		return normal;
	}

	Mat33 getInverted() const
	{
		Mat33 tempCopy;

		float det = (_00*_11*_22 + _01*_12*_20 + _02*_10*_21 - _00*_12*_21 - _01*_10*_22 - _02*_11*_20);

		if (fast_abs(det) > 1e-30f)
		{
			float invDet = 1.0f / det;

			tempCopy._00 = invDet * (_11*_22 - _12*_21);
			tempCopy._01 = invDet * (_02*_21 - _01*_22);
			tempCopy._02 = invDet * (_01*_12 - _02*_11);

			tempCopy._10 = invDet * (_12*_20 - _10*_22);
			tempCopy._11 = invDet * (_00*_22 - _02*_20);
			tempCopy._12 = invDet * (_02*_10 - _00*_12);

			tempCopy._20 = invDet * (_10*_21 - _11*_20);
			tempCopy._21 = invDet * (_01*_20 - _00*_21);
			tempCopy._22 = invDet * (_00*_11 - _01*_10);
		}
		else
		{
			tempCopy.zero();
		}

		return tempCopy;
	}

	Mat33 & invert()
	{
		*this = getInverted();
		return *this;
	}

	Mat33 sqrt(unsigned int maxIter = 10) const
	{
		Mat33 root = *this, invroot, temp;
		invroot.identity();

		for (unsigned int i = 0; i < maxIter; ++i)
		{
			temp = root;
			root = 0.5f  * (root + invroot.getInverted());
			invroot = 0.5f * (invroot + temp.getInverted());

			// Newton method (unstable)
			//		root = 0.5f  * (root + root.getInverted() * *this);
		}

		return root;
	}

	Mat33 getTransposed() const
	{
		Mat33 tempCopy;

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

	Mat33 & transpose()
	{
		*this = getTransposed();
		return *this;
	}

	void polarDecompose(Mat33 & p, Mat33 & u, unsigned int maxIter = 10) const
	{
		p = (getTransposed() * *this).sqrt(maxIter);
		u = *this * p.getInverted();
	}

	// Vector Axis must be normalized for correct Rotation [ RAD ]
	///////////////////////////////////////////////////////////////////////
	Mat33 & fillRotation(const Vec3 & axis, float radAngle)
	{
		float cf = cosf(radAngle),
			  sf = sinf(radAngle);

		float xx = axis.x * axis.x,
			  yy = axis.y * axis.y,
			  zz = axis.z * axis.z;
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

		return *this;
	}
	Mat33 & fillScale(const Vec3 & vec)
	{
		_00 = vec.x;
		_11 = vec.y;
		_22 = vec.z;

		return *this;
	}

	Mat33 & rotate(const Vec3 & axis, float radAngle)
	{
		Mat33 rotMat;
		rotMat.fillRotation(axis, radAngle);

		*this = *this * rotMat;

		return *this;
	}
	Mat33 & scale(const Vec3 & vec)
	{
		Mat33 scMat(0.0f);
		scMat.identity();
		scMat.fillScale(vec);

		*this = *this * scMat;

		return *this;
	}
};

}

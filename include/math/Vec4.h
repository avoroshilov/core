#pragma once

namespace math
{

	class Vec4
	{
	public:

		union
		{
			struct
			{
				float x, y, z, w;
			};
			float v[4];
		};

		static Vec4 makeVec4(float x, float y, float z, float w)
		{
			Vec4 temp;
			temp.x = x;
			temp.y = y;
			temp.z = z;
			temp.w = w;
			return temp;
		}

		Vec4 operator + (const Vec4 & vec) const
		{
			return makeVec4(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
		}
		Vec4 operator - (const Vec4 & vec) const
		{
			return makeVec4(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
		}
		Vec4 operator - () const
		{
			return makeVec4(-x, -y, -z, -w);
		}
		Vec4 & operator = (const Vec4 & vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;
			w = vec.w;
			return *this;
		}
		Vec4 & operator += (const Vec4 & vec)
		{
			x += vec.x;
			y += vec.y;
			z += vec.z;
			return *this;
		}
		Vec4 & operator -= (const Vec4 & vec)
		{
			x -= vec.x;
			y -= vec.y;
			z -= vec.z;
			w -= vec.w;
			return *this;
		}

		friend Vec4 operator * (const Vec4 & vec, const float & val)
		{
			return makeVec4(vec.x * val, vec.y * val, vec.z * val, vec.w * val);
		}
		friend Vec4 operator * (const float & val, const Vec4 & vec)
		{
			return makeVec4(val * vec.x, val * vec.y, val * vec.z, val * vec.w);
		}
		friend Vec4 operator / (const Vec4 & vec, const float & val)
		{
			return makeVec4(vec.x / val, vec.y / val, vec.z / val, vec.w / val);
		}

		Vec4 & operator *= (const float & val)
		{
			x *= val;
			y *= val;
			z *= val;
			w *= val;
			return *this;
		}
		Vec4 & operator /= (const float & val)
		{
			x /= val;
			y /= val;
			z /= val;
			w /= val;
			return *this;
		}

		// functions
		float dot(const Vec4 & vec) const
		{
			return x*vec.x + y*vec.y + z*vec.z + w*vec.w;
		}
		Vec4 mulHadamard(const Vec4 & vec) const
		{
			return makeVec4(x*vec.x, y*vec.y, z*vec.z, w*vec.w);
		}
		float sqLen() const
		{
			return x*x + y*y + z*z + w*w;
		}
		float len() const
		{
			return sqrtf(x*x + y*y + z*z + w*w);
		}

		Vec4 & normalize()
		{
			float vecLen = len();

			const float eps = 1e-5f;
			if (vecLen < eps)
			{
				x = 0.0f;
				y = 0.0f;
				z = 0.0f;
				w = 1.0f;
			}
			else
			{
				x /= vecLen;
				y /= vecLen;
				z /= vecLen;
				w /= vecLen;
			}

			return *this;
		}
		Vec4 getNormalized() const
		{
			Vec4 vec(*this);
			return vec.normalize();
		}
	};

	class Vec4C : public Vec4
	{
	public:

		Vec4C(float _x, float _y, float _z, float _w)
		{
			x = _x;
			y = _y;
			z = _z;
			w = _w;
		}

		Vec4C(const Vec4 & vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;
			w = vec.w;
		}
	};
}

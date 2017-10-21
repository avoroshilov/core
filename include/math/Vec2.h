#pragma once

namespace math
{

	class Vec2
	{
	public:

		union
		{
			struct
			{
				float x, y;
			};
			float v[2];
		};

		static Vec2 makeVec2(float x, float y)
		{
			Vec2 temp;
			temp.x = x;
			temp.y = y;
			return temp;
		}

		Vec2 operator + (const Vec2 & vec) const
		{
			return makeVec2(x + vec.x, y + vec.y);
		}
		Vec2 operator - (const Vec2 & vec) const
		{
			return makeVec2(x - vec.x, y - vec.y);
		}
		Vec2 operator - () const
		{
			return makeVec2(-x, -y);
		}
		Vec2 & operator = (const Vec2 & vec)
		{
			x = vec.x;
			y = vec.y;
			return *this;
		}
		Vec2 & operator += (const Vec2 & vec)
		{
			x += vec.x;
			y += vec.y;
			return *this;
		}
		Vec2 & operator -= (const Vec2 & vec)
		{
			x -= vec.x;
			y -= vec.y;
			return *this;
		}

		friend Vec2 operator * (const Vec2 & vec, const float & val)
		{
			return makeVec2(vec.x * val, vec.y * val);
		}
		friend Vec2 operator * (const float & val, const Vec2 & vec)
		{
			return makeVec2(val * vec.x, val * vec.y);
		}
		friend Vec2 operator / (const Vec2 & vec, const float & val)
		{
			return makeVec2(vec.x / val, vec.y / val);
		}

		Vec2 & operator *= (const float & val)
		{
			x *= val;
			y *= val;
			return *this;
		}
		Vec2 & operator /= (const float & val)
		{
			x /= val;
			y /= val;
			return *this;
		}

		// functions
		float dot(const Vec2 & vec) const
		{
			return x*vec.x + y*vec.y;
		}
		Vec2 perp() const
		{
			return makeVec2(y, -x);
		}
		Vec2 mulHadamard(const Vec2 & vec) const
		{
			return makeVec2(x*vec.x, y*vec.y);
		}
		float sqLen() const
		{
			return x*x + y*y;
		}
		float len() const
		{
			return sqrtf(x*x + y*y);
		}

		Vec2 & normalize()
		{
			float vecLen = len();

			const float eps = 1e-5f;
			if (vecLen < eps)
			{
				x = 0.0f;
				y = 1.0f;
			}
			else
			{
				x /= vecLen;
				y /= vecLen;
			}

			return *this;
		}
		Vec2 getNormalized() const
		{
			Vec2 vec(*this);
			return vec.normalize();
		}
	};

	class Vec2C : public Vec2
	{
	public:

		Vec2C(float _x, float _y)
		{
			x = _x;
			y = _y;
		}

		Vec2C(const Vec2 & vec)
		{
			x = vec.x;
			y = vec.y;
		}
	};
}

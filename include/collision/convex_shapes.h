#pragma once

class ConvexSphere : public ConvexHullShape
{
public:

	float m_radius;
	math::Vec3 m_origin;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eSphere; }

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		return m_radius * direction.getNormalized();
	}
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		return pos + getSupportVertexLocal(direction);
	}

	virtual math::Vec3 getOrigin() const
	{
		return m_origin;
	}
	virtual void setOrigin(const math::Vec3 & origin)
	{
		m_origin = origin;
	}

	virtual math::Mat33 getRotation() const
	{
		return math::Mat33().identity();
	}
	virtual void setRotation(const math::Mat33 & origin)
	{
	}

	virtual void calculateMassPropertiesUnitDensity()
	{
		m_com = math::Vec3C();
		float radSq = m_radius*m_radius;
		m_volume = 4.0f/3.0f * PI * radSq*m_radius;
		float sphInertiaComponent = 2.0f / 5.0f * m_volume * radSq;
		m_localInertia.zero();
		m_localInertia._00 = sphInertiaComponent;
		m_localInertia._11 = sphInertiaComponent;
		m_localInertia._22 = sphInertiaComponent;
	}
};

class ConvexBox : public ConvexHullShape
{
public:

	math::Vec3 m_halfExtents;
	math::Vec3 m_origin;
	math::Mat33 m_rotation;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eBox; }

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		math::Vec3 dirN = direction.getNormalized();
		return math::Vec3C(m_halfExtents.x * sign(dirN.x), m_halfExtents.y * sign(dirN.y), m_halfExtents.z * sign(dirN.z));
		//return math::Vec3C(m_halfExtents.x * sign(dirN.x), m_halfExtents.y * sign(dirN.y), m_halfExtents.z * sign(dirN.z)) + 0.1f * dirN;
	}
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		math::Vec3 dirLocal = rot.getConjugated().rotate(direction);
		return pos + rot.rotate(getSupportVertexLocal(dirLocal));
	}

	virtual math::Vec3 getOrigin() const
	{
		return m_origin;
	}
	virtual void setOrigin(const math::Vec3 & origin)
	{
		m_origin = origin;
	}

	virtual math::Mat33 getRotation() const
	{
		return m_rotation;
	}
	virtual void setRotation(const math::Mat33 & rotation)
	{
		m_rotation = rotation;
	}

	virtual void calculateMassPropertiesUnitDensity()
	{
		m_com = math::Vec3C();
		math::Vec3 extents = 2 * m_halfExtents;
		m_volume = extents.x*extents.y*extents.z;
		float inertiaComponentMul = 1.0f / 12.0f * m_volume;
		m_localInertia.zero();
		m_localInertia._00 = inertiaComponentMul * (extents.y*extents.y + extents.z*extents.z);
		m_localInertia._11 = inertiaComponentMul * (extents.x*extents.x + extents.z*extents.z);
		m_localInertia._22 = inertiaComponentMul * (extents.x*extents.x + extents.y*extents.y);
	}
};

class ConvexCapsule: public ConvexHullShape
{
public:

	math::Vec3 m_radHeight;
	math::Vec3 m_origin;
	math::Mat33 m_rotation;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eCapsule; }

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		math::Vec3 dirN = direction.getNormalized();
		return math::Vec3C(0.0f, m_radHeight.y * sign(dirN.y), 0.0f) + m_radHeight.x * dirN;
	}
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		math::Vec3 dirLocal = rot.getConjugated().rotate(direction);
		return pos + rot.rotate(getSupportVertexLocal(dirLocal));
	}

	virtual math::Vec3 getOrigin() const
	{
		return m_origin;
	}
	virtual void setOrigin(const math::Vec3 & origin)
	{
		m_origin = origin;
	}

	virtual math::Mat33 getRotation() const
	{
		return m_rotation;
	}
	virtual void setRotation(const math::Mat33 & rotation)
	{
		m_rotation = rotation;
	}

	virtual void calculateMassPropertiesUnitDensity()
	{
		m_com = math::Vec3C();
		
		// Vertical capsule
		float rad = m_radHeight.x;
		float radSq = rad*rad;
		float height = m_radHeight.y;
		float heightSq = height*height;

		float volCap = 2.0f / 3.0f * radSq*rad * PI;
		float volCylinder = height * PI * radSq;

		m_volume = volCylinder + 2.0f*volCap;

		// Calculated analytically as a sum of inertia tensors via parallel axis theorem
		m_localInertia.zero();
		m_localInertia._00 = volCylinder*(heightSq/12.0f + radSq/4.0f) + 2*volCap*(2.0f/5.0f * radSq + heightSq/2.0f + 3.0f/8.0f*height*rad);
		m_localInertia._11 = volCylinder*(radSq/2.0f) + 2*volCap*(2.0f/5.0f * radSq);
		m_localInertia._22 = volCylinder*(heightSq/12.0f + radSq/4.0f) + 2*volCap*(2.0f/5.0f * radSq + heightSq/2.0f + 3.0f/8.0f*height*rad);
	}
};

class ConvexCylinder: public ConvexHullShape
{
public:

	math::Vec3 m_radHeight;
	math::Vec3 m_origin;
	math::Mat33 m_rotation;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eCylinder; }

	math::Vec3 getXZDiskSupportPoint(float rad, const math::Vec3 & n)
	{
		math::Vec3 diskDir = math::Vec3C(n.x, 0.0f, n.z);
		float dirLen = diskDir.len();
		const float lenEps = 1e-6f;
		if (dirLen < lenEps)
			return math::Vec3C();
		else
			return (rad / dirLen) * diskDir;
	}
	math::Vec3 getYSegmentSupportPoint(float size, const math::Vec3 & n)
	{
		return n.y >= 0.0f ? math::Vec3C(0.0f, size, 0.0f) : math::Vec3C(0.0f, -size, 0.0f);
	}

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		math::Vec3 dirN = direction.getNormalized();
		return getXZDiskSupportPoint(m_radHeight.x, dirN) + getYSegmentSupportPoint(m_radHeight.y, dirN);
	}
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		math::Vec3 dirLocal = rot.getConjugated().rotate(direction);
		return pos + rot.rotate(getSupportVertexLocal(dirLocal));
	}

	virtual math::Vec3 getOrigin() const
	{
		return m_origin;
	}
	virtual void setOrigin(const math::Vec3 & origin)
	{
		m_origin = origin;
	}

	virtual math::Mat33 getRotation() const
	{
		return m_rotation;
	}
	virtual void setRotation(const math::Mat33 & rotation)
	{
		m_rotation = rotation;
	}

	virtual void calculateMassPropertiesUnitDensity()
	{
		m_com = math::Vec3C();
		
		// Vertical capsule
		float rad = m_radHeight.x;
		float radSq = rad*rad;
		float height = m_radHeight.y;
		float heightSq = height*height;

		m_volume = height * PI * radSq;

		// Calculated analytically
		m_localInertia.zero();
		m_localInertia._00 = m_volume*(heightSq/12.0f + radSq/4.0f);
		m_localInertia._11 = m_volume*(radSq/2.0f);
		m_localInertia._22 = m_volume*(heightSq/12.0f + radSq/4.0f);
	}
};

class ConvexCone: public ConvexHullShape
{
public:

	math::Vec3 m_radHeight;
	math::Vec3 m_origin;
	math::Mat33 m_rotation;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eCone; }

	math::Vec3 getXZDiskSupportPoint(float rad, const math::Vec3 & n)
	{
		math::Vec3 diskDir = math::Vec3C(n.x, 0.0f, n.z);
		float dirLen = diskDir.len();
		const float lenEps = 1e-6f;
		if (dirLen < lenEps)
			return math::Vec3C();
		else
			return (rad / dirLen) * diskDir;
	}

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		math::Vec3 dirN = direction.getNormalized();
		math::Vec3 diskSupport = getXZDiskSupportPoint(m_radHeight.x, dirN);
		diskSupport.y = -m_radHeight.y;
		math::Vec3 tipSupportPoint = math::Vec3C(0.0f, m_radHeight.y, 0.0f);

		math::Vec3 smoothness = 0.05f * m_radHeight.x * dirN;

		if (dirN.dot(diskSupport) > dirN.dot(tipSupportPoint))
			return diskSupport + smoothness;
		else
			return tipSupportPoint + smoothness;
	}
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		math::Vec3 dirLocal = rot.getConjugated().rotate(direction);
		return pos + rot.rotate(getSupportVertexLocal(dirLocal));
	}

	virtual math::Vec3 getOrigin() const
	{
		return m_origin;
	}
	virtual void setOrigin(const math::Vec3 & origin)
	{
		m_origin = origin;
	}

	virtual math::Mat33 getRotation() const
	{
		return m_rotation;
	}
	virtual void setRotation(const math::Mat33 & rotation)
	{
		m_rotation = rotation;
	}

	virtual void calculateMassPropertiesUnitDensity()
	{
		m_com = math::Vec3C();
		
		// Vertical capsule
		float rad = m_radHeight.x;
		float radSq = rad*rad;
		float height = m_radHeight.y;
		float heightSq = height*height;

		m_volume = (height / 3.0f) * PI * radSq;

		// Calculated analytically
		m_localInertia.zero();
		m_localInertia._00 = m_volume*(0.1f*heightSq + (3.0f/20.0f)*radSq);
		m_localInertia._11 = m_volume*(0.3f*radSq);
		m_localInertia._22 = m_volume*(0.1f*heightSq + (3.0f/20.0f)*radSq);
	}
};

class ConvexCustom : public ConvexHullShape
{
public:

	math::Vec3 m_origin;
	math::Mat33 m_rotation;

	std::vector<math::Vec3> m_hullTriangles;
	std::vector<math::Vec3> m_points;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eCustom; }

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		using namespace math;

		const size_t numPoints = m_points.size();
		if (numPoints < 1)
			return Vec3C();

		Vec3 * pts = m_points.data();

		//math::Vec3 dirN = direction.getNormalized();
		Vec3 curSupportPoint = pts[0];
		float maxProj = direction.dot(curSupportPoint);
		for (size_t pi = 1; pi < numPoints; ++pi)
		{
			float proj = direction.dot(pts[pi]);
			if (maxProj < proj)
			{
				maxProj = proj;
				curSupportPoint = pts[pi];
			}
		}
		return curSupportPoint;
		//return math::Vec3C(m_halfExtents.x * sign(dirN.x), m_halfExtents.y * sign(dirN.y), m_halfExtents.z * sign(dirN.z)) + 0.1f * dirN;
	}
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		math::Vec3 dirLocal = rot.getConjugated().rotate(direction);
		return pos + rot.rotate(getSupportVertexLocal(dirLocal));
	}

	virtual math::Vec3 getOrigin() const
	{
		return m_origin;
	}
	virtual void setOrigin(const math::Vec3 & origin)
	{
		m_origin = origin;
	}

	virtual math::Mat33 getRotation() const
	{
		return m_rotation;
	}
	virtual void setRotation(const math::Mat33 & rotation)
	{
		m_rotation = rotation;
	}

	virtual void calculateMassPropertiesUnitDensity()
	{
		m_hullTriangles.reserve(m_points.size() * 3);
		buildHullTriangles(this, &m_hullTriangles);
		calcHullMassProperties(m_hullTriangles, &m_volume, &m_com, &m_localInertia);

		// Possible with FP precision issues?
		if (m_volume < 0.0f)
		{
			m_volume = -m_volume;
			m_com = -m_com;
			m_localInertia = -1.0f * m_localInertia;

			// TODO: inverse triangle vertices order as well
		}
	}
};
#pragma once

#include <math/Vec3.h>
#include <math/Mat33.h>
#include <collision/convex_hull.h>

namespace physics
{

class GeometryShape
{
protected:
public:

	math::Vec3 m_origin;
	math::Mat33 m_rotation;
	float m_density;

	// Mass properties of geometry
	math::Vec3 m_geometryCoM;
	math::Mat33 m_geometryLocalInertia;
	float m_geometryVolume;

	enum class Type
	{
		eSphere,
		eGenericConvex,

		eNUM_ENTITIES
	};
	virtual Type getType() = 0;
	virtual void calculateMassProperties(float desiredMass = -1.0f) = 0;
	virtual math::Vec3 getBoundingPoint(const math::Vec3 & dir) = 0;
};

class GeometrySphere : public GeometryShape
{
public:

	float m_radius;
	virtual GeometryShape::Type getType() { return GeometryShape::Type::eSphere; }
	virtual void calculateMassProperties(float desiredMass = -1.0f)
	{
		float radSq = m_radius*m_radius;
		m_geometryVolume = 4.0f/3.0f * PI * radSq*m_radius;
		m_geometryCoM = math::Vec3C();

		float mass;
		if (desiredMass == -1.0f)
			mass = m_density * m_geometryVolume;
		else
		{
			mass = desiredMass;
			m_density = mass / m_geometryVolume;
		}

		float sphInertiaComponent = 2.0f / 5.0f * mass * radSq;
		m_geometryLocalInertia.zero();
		m_geometryLocalInertia._00 = sphInertiaComponent;
		m_geometryLocalInertia._11 = sphInertiaComponent;
		m_geometryLocalInertia._22 = sphInertiaComponent;
	}
	virtual math::Vec3 getBoundingPoint(const math::Vec3 & dir)
	{
		return m_radius * dir.getNormalized();
	}
};

class GeometryConvex : public GeometryShape
{
public:

	ConvexHullShape * m_convexHullShape;
	virtual GeometryShape::Type getType() { return GeometryShape::Type::eGenericConvex; }

	virtual void calculateMassProperties(float desiredMass = -1.0f)
	{
		m_convexHullShape->calculateMassPropertiesUnitDensity();

		if (desiredMass != -1.0f)
		{
			m_density = desiredMass / m_geometryVolume;
		}

		m_geometryVolume = m_convexHullShape->m_volume;
		m_geometryCoM = m_convexHullShape->m_com;
		m_geometryLocalInertia = m_density * m_convexHullShape->m_localInertia;
	}
	virtual math::Vec3 getBoundingPoint(const math::Vec3 & dir)
	{
		return m_convexHullShape->getSupportVertexLocal(dir);
	}
};

}
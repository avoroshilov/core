#pragma once

#include <physics/geometry.h>
#include <physics/solver_base.h>

namespace physics
{

class RigidBody
{
protected:
public:

	NodeTranslational * m_bodyL;
	NodeRotational * m_bodyR;

	// TODO: store positions and orientations of geometries in the rigid body, rather than
	//	in the geometry itself, for easy shape sharing
	std::vector<GeometryShape *> m_geometryShapes;

	math::Vec3 getOrigin()
	{
		if (!m_bodyR)
			return m_bodyL->m_pos;
		return m_bodyL->m_pos + m_bodyR->m_rot.rotate(-m_bodyR->m_com);
	}
	math::Vec3 transformPointWorldToOrigin(const math::Vec3 & pointWorld)
	{
		if (!m_bodyR)
			return pointWorld - m_bodyL->m_pos;

		math::Vec3 originPos = m_bodyL->m_pos + m_bodyR->m_rot.rotate(-m_bodyR->m_com);
		return m_bodyR->m_rot.getInversed().rotate(pointWorld - originPos);
	}
	math::Vec3 transformPointOriginToWorld(const math::Vec3 & pointVsOrigin)
	{
		if (!m_bodyR)
			return m_bodyL->m_pos + pointVsOrigin;

		return m_bodyL->m_pos + m_bodyR->m_rot.rotate(pointVsOrigin - m_bodyR->m_com);
	}

	void adjustCoM(const math::Vec3 & comShift)
	{
		using namespace math;

		if (m_bodyL->m_invMass == 0.0f)
			return;

		m_bodyL->m_pos += comShift;
		m_bodyR->m_com += comShift;

		Mat33 inertia = m_bodyR->m_invInertia;
		inertia.invert();

		float totalMass = 1.0f / m_bodyL->m_invMass;

		Mat33 steinerMatrix;
		float as = comShift.sqLen();

		// Steiner's theorem (parallel axis theorem)
		//	Inertia'_ij = Inertia_ij + m*(|dx|^2 * dkr_ij - dx_i * dx_j)
		//	where dkr_ij = Kronecker's delta, effectively identity matrix at i,j
		steinerMatrix._00 = as - comShift.x*comShift.x;
		steinerMatrix._01 = -comShift.x*comShift.y;
		steinerMatrix._02 = -comShift.x*comShift.z;
		steinerMatrix._10 = -comShift.y*comShift.x;
		steinerMatrix._11 = as - comShift.y*comShift.y;
		steinerMatrix._12 = -comShift.y*comShift.z;
		steinerMatrix._20 = -comShift.z*comShift.x;
		steinerMatrix._21 = -comShift.z*comShift.y;
		steinerMatrix._22 = as - comShift.z*comShift.z;

		inertia = inertia + totalMass * steinerMatrix;

		Mat33 invInertia = inertia;
		invInertia.invert();

		m_bodyR->m_invInertia = invInertia;
	}

	// Calculate mass properties, shift the body according to the center of mass
	void prepareGeometry(float desiredMass = -1.0f)
	{
		using namespace math;

		float totalMass = 0.0f;
		Vec3 com = Vec3C();
		for (size_t gi = 0, giEnd = m_geometryShapes.size(); gi < giEnd; ++gi)
		{
			GeometryShape * curShape = m_geometryShapes[gi];
			curShape->calculateMassProperties();
			float mass = curShape->m_density * curShape->m_geometryVolume;
			com += (curShape->m_origin + curShape->m_geometryCoM) * mass;
			totalMass += mass;
		}

		if (totalMass == 0.0f)
		{
			m_bodyL->m_invMass = 0.0f;
			m_bodyL->m_pos += com;
			m_bodyR->m_invInertia.zero();
			m_bodyR->m_com = com;
			return;
		}

		float invTotalMass = 1.0f / totalMass;
		com *= invTotalMass;

		// If desired mass is provided, we need to rescale all the shapes for the body to have
		//	the required mass
		if (desiredMass != -1.0f)
		{
			float massScale = desiredMass * invTotalMass;
			for (size_t gi = 0, giEnd = m_geometryShapes.size(); gi < giEnd; ++gi)
			{
				GeometryShape * curShape = m_geometryShapes[gi];
				curShape->m_density *= massScale;
				curShape->m_geometryLocalInertia *= massScale;
			}
			totalMass = desiredMass;
			invTotalMass = 1.0f / totalMass;
		}

		m_bodyL->m_invMass = invTotalMass;
		m_bodyL->m_pos += com;
		m_bodyR->m_com = com;

		Mat33 inertia;
		inertia.zero();
		for (size_t gi = 0, giEnd = m_geometryShapes.size(); gi < giEnd; ++gi)
		{
			GeometryShape * curShape = m_geometryShapes[gi];
			
			Vec3 toCoM = com - curShape->m_origin;
			float as = toCoM.sqLen();

			Mat33 steinerMatrix;
			// Steiner's theorem (parallel axis theorem)
			//	Inertia'_ij = Inertia_ij + m*(|dx|^2 * dkr_ij - dx_i * dx_j)
			//	where dkr_ij = Kronecker's delta, effectively identity matrix at i,j
			steinerMatrix._00 = as - toCoM.x*toCoM.x;
			steinerMatrix._01 = -toCoM.x*toCoM.y;
			steinerMatrix._02 = -toCoM.x*toCoM.z;
			steinerMatrix._10 = -toCoM.y*toCoM.x;
			steinerMatrix._11 = as - toCoM.y*toCoM.y;
			steinerMatrix._12 = -toCoM.y*toCoM.z;
			steinerMatrix._20 = -toCoM.z*toCoM.x;
			steinerMatrix._21 = -toCoM.z*toCoM.y;
			steinerMatrix._22 = as - toCoM.z*toCoM.z;
			
			// Inertia in this case is R * Inertia * R^T to account for the shape rotation
			inertia += curShape->m_rotation * curShape->m_geometryLocalInertia * curShape->m_rotation.getTransposed() + curShape->m_density * curShape->m_geometryVolume * steinerMatrix;
		}
		Mat33 invInertia = inertia;
		invInertia.invert();

		m_bodyR->m_invInertia = invInertia;
	}
};

}

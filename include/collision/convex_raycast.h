#pragma once

#include <math/Vec3.h>
#include <math/Mat33.h>
#include <math/Quat.h>

#include "contact_helpers.h"
#include "convex_hull.h"
#include "mpr.h"

class LineSegmentConvex : public ConvexHullShape
{
public:

	math::Vec3 m_point;
	math::Vec3 m_diff;

	virtual ConvexHullShape::Type getType() const { return ConvexHullShape::Type::eCustom; }

	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction)
	{
		if (m_diff.dot(direction) > 0)
			return m_point + m_diff;
		else
			return m_point;
	}

	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot)
	{
		math::Vec3 dirLocal = rot.getConjugated().rotate(direction);
		return pos + rot.rotate(getSupportVertexLocal(dirLocal));
	}

	virtual math::Vec3 getOrigin() const { return math::Vec3C(); }
	virtual void setOrigin(const math::Vec3 & origin) {}
	virtual math::Mat33 getRotation() const { return math::Mat33().identity(); }
	virtual void setRotation(const math::Mat33 & rotation) {}

	virtual void calculateMassPropertiesUnitDensity() {}
};

bool raycastVsConvex(ConvexHullShape * hull, const math::Vec3 & hullPos, const math::Quat & hullRot, const math::Vec3 & rayO, const math::Vec3 & rayDNormalized, float * rayScalar, float rayDMul = 20.0f)
{
	using namespace math;

	assert(fast_abs(rayDNormalized.sqLen() - 1.0f) < 1e-5f);

	LineSegmentConvex rayConvex;
	rayConvex.m_point = rayO;
	rayConvex.m_diff = rayDNormalized*rayDMul;

	Vec3 contactNormal, cp0, cp1;
	// Portals
	Vec3 pp00, pp01, pp02;
	Vec3 pp10, pp11, pp12;

	bool rayHit = mprContact(
		hull, &rayConvex,
		hullPos, hullRot,
		Vec3C(), Quat().id(),
		hullPos, rayO,
		&contactNormal, &cp0, &cp1,
		&pp00, &pp01, &pp02,
		&pp10, &pp11, &pp12
		);

	if (rayHit)
	{
		float proj = (cp0 - rayO).dot(rayDNormalized);
		rayConvex.m_diff = proj * rayDNormalized;

		Vec3 interiorPoint0 = hullPos + rayDNormalized;
		Vec3 interiorPoint1 = rayO - rayDNormalized;
		mprContact(
			hull, &rayConvex,
			hullPos, hullRot,
			Vec3C(), Quat().id(),
			interiorPoint0, interiorPoint1,
			&contactNormal,
			&cp0, &cp1,
			&pp00, &pp01, &pp02,
			&pp10, &pp11, &pp12
			);

		Vec3 cnTan0, cnTan1;
		const float dirDelta = 0.1f;
		contactNormal.tangentSpace(cnTan0, cnTan1);

		// sin(pi/4)==cos(pi/4)==sqrt(2)/2
		const float sinPi_4 = 0.70710678118654752440084436210485f;
		const float dirDeltaDiag = dirDelta*sinPi_4;

		Vec3 auxSupportPoints[8];

		// +-t0
		auxSupportPoints[0] = hull->getSupportVertex(-(contactNormal+cnTan0*dirDelta), hullPos, hullRot);
		auxSupportPoints[1] = hull->getSupportVertex(-(contactNormal-cnTan0*dirDelta), hullPos, hullRot);
		// +-t1
		auxSupportPoints[2] = hull->getSupportVertex(-(contactNormal+cnTan1*dirDelta), hullPos, hullRot);
		auxSupportPoints[3] = hull->getSupportVertex(-(contactNormal-cnTan1*dirDelta), hullPos, hullRot);
		// +-t0 +t1
		auxSupportPoints[4] = hull->getSupportVertex( -(contactNormal+dirDeltaDiag*( cnTan0+cnTan1)), hullPos, hullRot );
		auxSupportPoints[5] = hull->getSupportVertex( -(contactNormal+dirDeltaDiag*(-cnTan0+cnTan1)), hullPos, hullRot );
		// +-t0 -t1
		auxSupportPoints[6] = hull->getSupportVertex( -(contactNormal+dirDeltaDiag*( cnTan0-cnTan1)), hullPos, hullRot );
		auxSupportPoints[7] = hull->getSupportVertex( -(contactNormal+dirDeltaDiag*(-cnTan0-cnTan1)), hullPos, hullRot );

		uint32_t numUniqueVertices;
		removeDuplicatePointsInplace(auxSupportPoints, (uint32_t)8, &numUniqueVertices);

		Vec3 refPolyNormal = computeFaceNormal(auxSupportPoints, numUniqueVertices, -contactNormal);
			
		*rayScalar = (refPolyNormal.dot(auxSupportPoints[0]) - refPolyNormal.dot(rayO)) / (refPolyNormal.dot(rayDNormalized));

		// TODO: add sanity checks w/ support mapping projections and fall back to some not necessarily accurate solution
		if (*rayScalar < 0.0f)
			*rayScalar = 0.0f;

		return true;
	}
	else
	{
		return false;
	}
}


#pragma once

#include <assert.h>

#include "math/Vec3.h"
#include "math/Mat34.h"

#include "convex_hull.h"

template<typename T>
T sign(T val)
{
	return (val < (T)0) ? (T)-1 : (T)1;
}

bool mprHit(
	ConvexHullShape * shape0, ConvexHullShape * shape1,
	const math::Vec3 & s0pos, const math::Quat & s0rot,
	const math::Vec3 & s1pos, const math::Quat & s1rot,
	const math::Vec3 & interiorPoint0, const math::Vec3 & interiorPoint1
	)
{
	using namespace math;

	// Find origin ray
	const Vec3 interior0 = interiorPoint0;
	const Vec3 interior1 = interiorPoint1;

	Vec3 orP = interior1 - interior0;
	Vec3 orD = Vec3C(0.0f, 0.0f, 0.0f) - orP;

	auto supCSO = [&shape0, &shape1, &s0pos, &s0rot, &s1pos, &s1rot](const Vec3 & dir) -> Vec3
	{
		return shape1->getSupportVertex(dir, s1pos, s1rot) - shape0->getSupportVertex(-dir, s0pos, s0rot);
	};

	// Find candidate portal
	Vec3 v1 = supCSO(orP.getNormalized());
	Vec3 v2 = supCSO(v1.cross(orP).getNormalized());
	Vec3 v3 = supCSO((v2 - orP).cross(v1 - orP).getNormalized());

	auto pointPlaneDist = [](const Vec3 & po, const Vec3 & pn, const Vec3 & p) -> float
	{
		return (p - po).dot(pn);
	};
	auto pointAbovePlane = [](const Vec3 & po, const Vec3 & pn, const Vec3 & p) -> bool
	{
		return (p - po).dot(pn) > 0.0f;
	};

	Vec3 temp;
	auto swapVertices = [&temp](Vec3 & vertex0, Vec3 & vertex1)
	{
		temp = vertex0;
		vertex0 = vertex1;
		vertex1 = temp;
	};

	int DBGiterations = 0;
	while (true)
	{
		assert(DBGiterations < 1000);
		if (DBGiterations >= 1000)
			return false;

		// TODO: this can be optimized due to constant origin point
		Vec3 p0n = (v1 - orP).cross(v2 - orP);
		Vec3 p1n = (v2 - orP).cross(v3 - orP);
		Vec3 p2n = (v3 - orP).cross(v1 - orP);

		bool p0Out = pointAbovePlane(orP, p0n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p1Out = pointAbovePlane(orP, p1n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p2Out = pointAbovePlane(orP, p2n, Vec3C(0.0f, 0.0f, 0.0f));

		if (!p0Out && !p1Out && !p2Out)
			break;

		// Find new candidate
		if (p0Out)
		{
			swapVertices(v1, v2);
			v3 = supCSO(p0n.getNormalized());
		}
		else if (p1Out)
		{
			swapVertices(v2, v3);
			v1 = supCSO(p1n.getNormalized());
		}
		else if (p2Out)
		{
			swapVertices(v3, v1);
			v2 = supCSO(p2n.getNormalized());
		}

		++DBGiterations;
	}

	Vec3 v0 = orP;

	// Portal refinement
	DBGiterations = 0;
	while (true)
	{
		assert(DBGiterations < 1000);
		if (DBGiterations >= 1000)
			return false;

		// Check if origin lies within the tetrahedron (interior_point, portal): (v0, v1, v2, v3)

		Vec3 p0n = (v1 - v0).cross(v2 - v0);
		Vec3 p1n = (v2 - v0).cross(v3 - v0);
		Vec3 p2n = (v3 - v0).cross(v1 - v0);
		Vec3 p3n = (v3 - v1).cross(v2 - v1);

		bool p0Out = pointAbovePlane(v0, p0n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p1Out = pointAbovePlane(v0, p1n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p2Out = pointAbovePlane(v0, p2n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p3Out = pointAbovePlane(v1, p3n, Vec3C(0.0f, 0.0f, 0.0f));

		if (!p0Out && !p1Out && !p2Out && !p3Out)
			return true;

		// If origin is still outside the tetrahedron - the ray should still go inside it
		//	hence, all the planes except portal should still have the origin contained
		assert(p3Out);
		assert(!p0Out && !p1Out && !p2Out);

		// Find support in direction of portal
		Vec3 farSupportPoint = supCSO(p3n.getNormalized());

		// Check if origin lies above the support plane (support normal is p3n, and support point)
		// If it lies above the support plane, this means it is outside the CSO, since support point
		//	is frathest CSO point in that direction, all CSO is below the support plane

		// TODO: could be replaced with the simple "above" check really
		float originSupportPlaneDist = pointPlaneDist(farSupportPoint, p3n, Vec3C(0.0f, 0.0f, 0.0f));
		if (originSupportPlaneDist > 0.0f)
		{
			return false;
		}

		// Should always be positive
		// Normal should be unit here, as we're using absolute tolerance
		float portalSupportPlaneDist = pointPlaneDist(v1, p3n.getNormalized(), farSupportPoint);
		if (portalSupportPlaneDist < 1e-4f)
		{
			// If portal plane is very close to the new supporting plane, consider that a hit
			return true;
		}

		Vec3 v4 = farSupportPoint;

		// p0n NOT USED!! for convenience
		// TODO: remove minus here, rewrite for CW logic (from CCW now)
		p1n = (v0 - v4).cross(v1 - v4);
		p2n = (v0 - v4).cross(v2 - v4);
		p3n = (v0 - v4).cross(v3 - v4);
		p1Out = pointAbovePlane(v0, p1n, Vec3C(0.0f, 0.0f, 0.0f));
		p2Out = pointAbovePlane(v0, p2n, Vec3C(0.0f, 0.0f, 0.0f));
		p3Out = pointAbovePlane(v0, p3n, Vec3C(0.0f, 0.0f, 0.0f));

		// The logic behind this decision could be explained on the simple voronoi diagram of three regions,
		//	edges of those regions have CW normals.
		// Then, for origin to lie within a region, it needs adgacent edges to look in different directions:
		//	if it lies in the first region (b/w e1 and e2), origin will be above e1 plane, and below e2 plane => removing distant point v3
		//	if it lies in the second region (b/w e2 and e3), origin will be above e2 plane and below e3 plane => removing distant point v1
		//	and so on

		if (p1Out && !p2Out)
		{
			// Region 1
			v3 = v4;
		}
		else if (p2Out && !p3Out)
		{
			// Region 2
			v1 = v4;
		}
		else
		{
			// Region 3
			v2 = v4;
		}

		++DBGiterations;
	}

	return false;
}

bool mprContact(
		ConvexHullShape * shape0, ConvexHullShape * shape1,
		const math::Vec3 & s0pos, const math::Quat & s0rot,
		const math::Vec3 & s1pos, const math::Quat & s1rot,
		const math::Vec3 & interiorPoint0, const math::Vec3 & interiorPoint1,
		math::Vec3 * normal,
		math::Vec3 * cp0, math::Vec3 * cp1,
		math::Vec3 * pp00 = nullptr, math::Vec3 * pp01 = nullptr, math::Vec3 * pp02 = nullptr,
		math::Vec3 * pp10 = nullptr, math::Vec3 * pp11 = nullptr, math::Vec3 * pp12 = nullptr
		)
{
	using namespace math;

	assert(normal);

	// Find origin ray
	const Vec3 interior0 = interiorPoint0;
	const Vec3 interior1 = interiorPoint1;

	assert(isSane(interior0.x) && isSane(interior0.y) && isSane(interior0.z));
	assert(isSane(interior1.x) && isSane(interior1.y) && isSane(interior1.z));

	Vec3 v00 = interior0;
	Vec3 v01 = interior1;

	Vec3 orP = interior1 - interior0;
	Vec3 orD = Vec3C(0.0f, 0.0f, 0.0f) - orP;

	auto supCSOPoints = [&shape0, &shape1, &s0pos, &s0rot, &s1pos, &s1rot](const Vec3 & dir, Vec3 * sp0, Vec3 * sp1) -> Vec3
	{
		Vec3 supportPoint0 = shape0->getSupportVertex(-dir, s0pos, s0rot);
		Vec3 supportPoint1 = shape1->getSupportVertex( dir, s1pos, s1rot);

		*sp0 = supportPoint0;
		*sp1 = supportPoint1;

		return supportPoint1 - supportPoint0;
	};

	// Find candidate portal
	Vec3 v10, v11;
	Vec3 v1 = supCSOPoints(orP.getNormalized(), &v10, &v11);

	Vec3 v2, v20, v21;
	Vec3 v3, v30, v31;

	Vec3 dir2, dir3;

#if 0
	dir2 = v1.cross(orP).getNormalized();
	dir3 = (v2 - orP).cross(v1 - orP).getNormalized();
#else
	orP.tangentSpace(dir2, dir3);
#endif

	v2 = supCSOPoints(dir2, &v20, &v21);
	v3 = supCSOPoints(dir3, &v30, &v31);

	auto pointPlaneDist = [](const Vec3 & po, const Vec3 & pn, const Vec3 & p) -> float
	{
		return (p - po).dot(pn);
	};
	auto pointAbovePlane = [](const Vec3 & po, const Vec3 & pn, const Vec3 & p) -> bool
	{
		return (p - po).dot(pn) > 0.0f;
	};

	Vec3 temp;
	auto swapVertices = [&temp](Vec3 & vertex0, Vec3 & vertex1)
	{
		temp = vertex0;
		vertex0 = vertex1;
		vertex1 = temp;
	};

	int DBGiterations = 0;
	while (true)
	{
		assert(DBGiterations < 1000);
		if (DBGiterations >= 1000)
			return false;

		// TODO: this can be optimized due to constant origin point
		Vec3 p0n = (v1 - orP).cross(v2 - orP);
		Vec3 p1n = (v2 - orP).cross(v3 - orP);
		Vec3 p2n = (v3 - orP).cross(v1 - orP);

		float dist0 = pointPlaneDist(orP, p0n, Vec3C(0.0f, 0.0f, 0.0f));
		float dist1 = pointPlaneDist(orP, p1n, Vec3C(0.0f, 0.0f, 0.0f));
		float dist2 = pointPlaneDist(orP, p2n, Vec3C(0.0f, 0.0f, 0.0f));

		bool p0Out = (dist0 > 0.0f);
		bool p1Out = (dist1 > 0.0f);
		bool p2Out = (dist2 > 0.0f);

		if (!p0Out && !p1Out && !p2Out)
			break;

		// Find new candidate
		if (p0Out)
		{
			swapVertices(v1, v2);
			swapVertices(v10, v20);
			swapVertices(v11, v21);
			v3 = supCSOPoints(p0n.getNormalized(), &v30, &v31);
		}
		else if (p1Out)
		{
			swapVertices(v2, v3);
			swapVertices(v20, v30);
			swapVertices(v21, v31);
			v1 = supCSOPoints(p1n.getNormalized(), &v10, &v11);
		}
		else if (p2Out)
		{
			swapVertices(v3, v1);
			swapVertices(v30, v10);
			swapVertices(v31, v11);
			v2 = supCSOPoints(p2n.getNormalized(), &v20, &v21);
		}

		++DBGiterations;
	}

	Vec3 v0 = orP;

	bool hitRegistered = false;

	// Portal refinement
	DBGiterations = 0;
	while (true)
	{
		assert(DBGiterations < 1000);
		if (DBGiterations >= 1000)
			return false;

		// Check if origin lies within the portal
		//	portal is defined by tetrahedron (v0, v1, v2, v3)

		Vec3 p0n = (v1 - v0).cross(v2 - v0);
		Vec3 p1n = (v2 - v0).cross(v3 - v0);
		Vec3 p2n = (v3 - v0).cross(v1 - v0);
		Vec3 p3n = (v3 - v1).cross(v2 - v1);

		if (p3n.x == 0.0f && p3n.y == 0.0f && p3n.z == 0.0f)
		{
			// If normal is zero, this means the tetrahedron is degenerate (line), and we mimic the
			//	expected behavior with just setting normal of the portal to the origin ray direction
			p3n = orD.getNormalized();
		}

		bool p0Out = pointAbovePlane(v0, p0n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p1Out = pointAbovePlane(v0, p1n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p2Out = pointAbovePlane(v0, p2n, Vec3C(0.0f, 0.0f, 0.0f));
		bool p3Out = pointAbovePlane(v1, p3n, Vec3C(0.0f, 0.0f, 0.0f));

		// We're in contact generation mode, and we shouldn't exit when origin is known to
		//	be inside the CSO. Instead, we need to keep updating the portal until it
		//	reaches the surface and no portal closer to surface could be generated, this
		//	would mean that we have portal on the surface, that would be indicating face
		//	on the CSO, closer to the origin; and we could use that face as an approximation
		//	for the penetration characteristics
		if (!p0Out && !p1Out && !p2Out && !p3Out)
		{
			if (!hitRegistered)
			{
				*normal = p3n.getNormalized();

				float bc0, bc1, bc2, bc3;
				computeBarycentric(v0, v1, v2, v3, Vec3C(), &bc0, &bc1, &bc2, &bc3);

				*cp0 = v00 * bc0 + v10 * bc1 + v20 * bc2 + v30 * bc3;
				*cp1 = v01 * bc0 + v11 * bc1 + v21 * bc2 + v31 * bc3;

				if (pp00)
					*pp00 = v10;
				if (pp01)
					*pp01 = v20;
				if (pp02)
					*pp02 = v30;

				if (pp10)
					*pp10 = v11;
				if (pp11)
					*pp11 = v21;
				if (pp12)
					*pp12 = v31;

				hitRegistered = true;
			}
		}

		// The ray should still go inside the tetra; hence, all the side planes should still
		//	have the origin contained
		// Check if precision issues caused the algorm to derail
		bool precisionIssues = p0Out || p1Out || p2Out;
		if (precisionIssues)
		{
			return hitRegistered;
		}

		// Find support in direction of portal
		Vec3 v40, v41; 
		Vec3 farSupportPoint = supCSOPoints(p3n.getNormalized(), &v40, &v41);

		// Check if origin lies above the support plane (support normal is p3n, and support point)
		// If it lies above the support plane, this means it is outside the CSO, since support point
		//	is frathest CSO point in that direction, all CSO is below the support plane

		// TODO: could be replaced with the simple "above" check really
		float originSupportPlaneDist = pointPlaneDist(farSupportPoint, p3n, Vec3C(0.0f, 0.0f, 0.0f));
		if (originSupportPlaneDist > 0.0f)
		{
			return false;
		}

		// Should always be positive
		// Normal should be unit here, as we're using absolute tolerance
		Vec3 p3n_nrm = p3n.getNormalized();
		float portalSupportPlaneDist = pointPlaneDist(v1, p3n_nrm, farSupportPoint);
		
		const float distEps = 1e-4f;
		if (portalSupportPlaneDist < distEps)
		{
			*normal = p3n_nrm;

			float bc0, bc1, bc2, bc3;
			computeBarycentric(v0, v1, v2, v3, Vec3C(), &bc0, &bc1, &bc2, &bc3);

			*cp0 = v00 * bc0 + v10 * bc1 + v20 * bc2 + v30 * bc3;
			*cp1 = v01 * bc0 + v11 * bc1 + v21 * bc2 + v31 * bc3;

			assert(isSane(cp0->x) && isSane(cp0->y) && isSane(cp0->z));
			assert(isSane(cp1->x) && isSane(cp1->y) && isSane(cp1->z));

			if (pp00)
				*pp00 = v10;
			if (pp01)
				*pp01 = v20;
			if (pp02)
				*pp02 = v30;

			if (pp10)
				*pp10 = v11;
			if (pp11)
				*pp11 = v21;
			if (pp12)
				*pp12 = v31;

			// If portal plane is very close to the new supporting plane, consider that a hit
			return true;
		}

		Vec3 v4 = farSupportPoint;

		// p0n NOT USED!! for convenience
		// TODO: remove minus here, rewrite for CW logic (from CCW now)
		p1n = (v0 - v4).cross(v1 - v4);
		p2n = (v0 - v4).cross(v2 - v4);
		p3n = (v0 - v4).cross(v3 - v4);
		p1Out = pointAbovePlane(v0, p1n, Vec3C(0.0f, 0.0f, 0.0f));
		p2Out = pointAbovePlane(v0, p2n, Vec3C(0.0f, 0.0f, 0.0f));
		p3Out = pointAbovePlane(v0, p3n, Vec3C(0.0f, 0.0f, 0.0f));

		// The logic behind this decision could be explained on the simple voronoi diagram of three regions,
		//	edges of those regions have CW normals.
		// Then, for origin to lie within a region, it needs adgacent edges to look in different directions:
		//	if it lies in the first region (b/w e1 and e2), origin will be above e1 plane, and below e2 plane => removing distant point v3
		//	if it lies in the second region (b/w e2 and e3), origin will be above e2 plane and below e3 plane => removing distant point v1
		//	and so on

		if (p1Out && !p2Out)
		{
			// Region 1
			v3 = v4;
			v30 = v40;
			v31 = v41;
		}
		else if (p2Out && !p3Out)
		{
			// Region 2
			v1 = v4;
			v10 = v40;
			v11 = v41;
		}
		else
		{
			// Region 3
			v2 = v4;
			v20 = v40;
			v21 = v41;
		}

		++DBGiterations;
	}

	return false;
}


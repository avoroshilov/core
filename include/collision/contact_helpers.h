#pragma once

#include <cstdint>
#include <assert.h>

#include <math/AuxMath.h>
#include <math/Vec3.h>
#include <math/Mat34.h>

// Simple function that goes through all the points [O(N*N)] and detects duplicate points,
//	once detected, the duplicate gets replaced with the last point, thus the algorithm is
//	inplace.
void removeDuplicatePointsInplace(math::Vec3 * points, uint32_t numInputPoints, uint32_t * numOutputPoints)
{
	using namespace math;

	const float sqThreshold = 1e-10f;
	uint32_t numFinalPoints = numInputPoints;
	for (uint32_t pi = 1; pi < numFinalPoints; ++pi)
	{
		for (uint32_t pj = 0; pj < pi; ++pj)
		{
			if ((points[pj] - points[pi]).sqLen() < sqThreshold)
			{
				// Points match, replace current point with last
				if (pi != numFinalPoints-1)
				{
					points[pi] = points[numFinalPoints-1];
				}
				--numFinalPoints;
				--pi;	// point i replaced, need to reprocess it
				break;
			}
		}
	}
	*numOutputPoints = numFinalPoints;
}

// Simple Gift Wrap algorithm, generates a contour around a point cloud (which is supposedly 2D,
//	but oriented in 3D space). The contour forms a convex polygon, which is also contiguous, i.e.
//	vertices (i, i+1) form a contour edge, and closed, i.e. (numVertices-1, 0) forms a final edge
//	in sequence.
// Function builds indices array, and the final convex polygon could be formed by going through the
//	indices list and fetching vertices from the original input points array, using those indices.
// Approximate normal is used to check the winding of polygon
void convexWrap2D(const math::Vec3 * points, uint32_t numInputPoints, const math::Vec3 & approximateNormal, uint32_t * convexWrapPointIndices, uint32_t * numConvexWrapPoints)
{
	using namespace math;

	if (numInputPoints < 3)
	{
		*numConvexWrapPoints = numInputPoints;
		for (uint32_t pi = 0; pi < numInputPoints; ++pi)
		{
			convexWrapPointIndices[pi] = pi;
		}
		return;
	}

	// TODO: special case of triangle, with easy winding check

	// Since the point cloud is flattened in 3D (and not "real" 2D), we need to compute
	//	spanning vectors of the point cloud plane.
	// Find two vectors representing the plane of the set of points
	// More robust way would be to use PCA
	Vec3 bX = points[1] - points[0];
	bX.normalize();
	Vec3 supportVec = points[2] - points[0];
	supportVec.normalize();
	Vec3 bZ = bX.cross(supportVec);
	//Vec3 bY = bZ.cross(bX);

	if (bZ.dot(approximateNormal) < 0.0f)
	{
		bX = -bX;
		bZ = -bZ;
	}

	assert(bX.sqLen() > 1e-10f);
	//assert(bY.sqLen() > 1e-10f);
	assert(bZ.sqLen() > 1e-10f);

	bX.normalize();
	bZ.normalize();

	// Select point with minimal projection along bX ("leftmost")
	uint32_t pointOnHullIdx = 0;
	float minCoord = points[pointOnHullIdx].dot(bX);
	for (uint32_t pi = 1; pi < numInputPoints; ++pi)
	{
		float coord = points[pi].dot(bX);
		if (minCoord > coord)
		{
			minCoord = coord;
			pointOnHullIdx = pi;
		}
	}

	// In this part of the algorithm:
	//	- First, take the leftmost point as the initial vertex of the 2D convex hull
	//	- Next, go through all the vertices in the point cloud and find vertex, so that
	//		all other points will be on one side of the edge, formed with the previous
	//		point. This will give edge of the convex hull.
	//	- Repeat the process of finding the "wrapping" edges, untill the hull will close,
	//		i.e. the next vertex will match the very first (leftmost) vertex of the hull.
	uint32_t hullActivePoint = 0;
	uint32_t endPointIdx;

	do
	{
		convexWrapPointIndices[hullActivePoint] = pointOnHullIdx;
		endPointIdx = 0;
		Vec3 hullEdge = points[endPointIdx] - points[pointOnHullIdx];
		for (uint32_t pi = 1; pi < numInputPoints; ++pi)
		{
			Vec3 candidateEdge = points[pi] - points[pointOnHullIdx];
			if (endPointIdx == pointOnHullIdx || hullEdge.cross(candidateEdge).dot(bZ) > 0.0f)
			{
				endPointIdx = pi;
				hullEdge = candidateEdge;
				continue;
			}
		}
		++hullActivePoint;
		pointOnHullIdx = endPointIdx;

		// Additional condition that should prevent inf looping and subsequent memory violations
		//	sometimes, when points are really close to the origin point, the algorithm gets
		//	confused, and selects one of the points extremely close to the original point, due to
		//	precision issues; so we terminate if the amount of vertices in convex poly is similar
		//	to the input vertices. There are better ways to handle this, but this one simple, fast,
		//	and works reasonably well
		if (hullActivePoint == numInputPoints)
			break;
	} while (pointOnHullIdx != convexWrapPointIndices[0]);

	*numConvexWrapPoints = hullActivePoint;
}

// NOTE!! Clipping could produce more vertices than initially fed into
void clipPolygons(
	const math::Vec3 * inPoly0, uint32_t numVerts0,
	const math::Vec3 * inPoly1, uint32_t numVerts1,
	const math::Vec3 & poly1Normal,
	math::Vec3 * clippedBuf0, math::Vec3 * clippedBuf1, size_t clipBufsSize,
	float * planesBuf1,
	math::Vec3 * clippedPoly, uint32_t * numVertsClipped
	)
{
	using namespace math;

	uint32_t curBuf = 1;

	// Algorithm works by sequentially clipping polygon0 by planes,
	//	one clipping plane is formed by polygon1 edge, and an inward
	//	facing normal for that edge.
	// We need two intermediate buffers to do that, first we clip buf0
	//	against edgePlane0, and put results into buf1. Then, we clip
	//	the result from previos clipping (buf1) against edgePlane1, and
	//	put store results in the free buffer (buf0). And then we clip
	//	buf0 against edgePlane2, etc.
	Vec3 * clipBufV[2] = { clippedBuf0, clippedBuf1 };
	
	// Additional level of indirection allows to avoid couple of copies
	const Vec3 * srcPolygonBuf = inPoly0;
	Vec3 * dstPolygonBuf = clipBufV[curBuf];

	// Preparing poligon that we're about to clip (cvx0)
	int srcPointsNum = numVerts0, clippedPointsNum;

	float * planes = planesBuf1;

	// Preparing data for clipping polygon (cvx1)
	// Fill in the clipping planes
	// TODO: use Vec4 for plane, and implement helper function in math, that
	//	works with Vec4 as with plane storage
	const int numPlanesToTest = numVerts1;
	for (int pi = 0; pi < numPlanesToTest; ++pi)
	{
		int pi1 = (pi + 1)%numPlanesToTest;
		Vec3 planeNormal = (inPoly1[pi1] - inPoly1[pi]).cross(poly1Normal);
		planeNormal = planeNormal.getNormalized();
		float planeD = planeNormal.dot(inPoly1[pi]);
		planes[pi*4+0] = planeNormal.x;
		planes[pi*4+1] = planeNormal.y;
		planes[pi*4+2] = planeNormal.z;
		planes[pi*4+3] = -planeD;
	}

	// Sutherland-Hodgman clipping
	//////////////////////////////////////////////////////////////////////////
	for (int j = 0; j < numPlanesToTest; ++j)
	{
		clippedPointsNum = 0;

		scalar prevDist =
			planes[j*4+0] * srcPolygonBuf[0].x +
			planes[j*4+1] * srcPolygonBuf[0].y +
			planes[j*4+2] * srcPolygonBuf[0].z +
			planes[j*4+3];
		for (int k = 0; (k < srcPointsNum) && (clippedPointsNum < (int)clipBufsSize); ++k)
		{
			int k_1 = (k + 1) % srcPointsNum;

			// Clipping here
			scalar curDist =
				planes[j*4+0] * srcPolygonBuf[k_1].x +
				planes[j*4+1] * srcPolygonBuf[k_1].y +
				planes[j*4+2] * srcPolygonBuf[k_1].z +
				planes[j*4+3];

			if ((prevDist >= 0.0f) && (curDist >= 0.0f))
			{
				// v0 in, v1 in
				// * Store v1
				dstPolygonBuf[clippedPointsNum] = srcPolygonBuf[k_1];
				++clippedPointsNum;
				assert(clippedPointsNum < (int)clipBufsSize);
			}
			else if ((prevDist >= 0.0f) && (curDist < 0.0f))
			{
				// v0 in, v1 out
				// * Store intersection of the (k, k+1) edge with the clipping plane
				float t = prevDist / (prevDist - curDist);
				dstPolygonBuf[clippedPointsNum] = srcPolygonBuf[k] + (srcPolygonBuf[k_1] - srcPolygonBuf[k]) * t;
				++clippedPointsNum;
				assert(clippedPointsNum < (int)clipBufsSize);
			}
			else if ((prevDist < 0.0f) && (curDist >= 0.0f))
			{
				// v0 out, v1 in
				// * Store intersection of the (k, k+1) edge with the clipping plane
				// * Store v1 as well
				float t = prevDist / (prevDist - curDist);
				dstPolygonBuf[clippedPointsNum] = srcPolygonBuf[k] + (srcPolygonBuf[k_1] - srcPolygonBuf[k]) * t;
				++clippedPointsNum;
				if (clippedPointsNum < (int)clipBufsSize)
				{
					dstPolygonBuf[clippedPointsNum] = srcPolygonBuf[k_1];
					++clippedPointsNum;
				}
				assert(clippedPointsNum < (int)clipBufsSize);
			}

			prevDist = curDist;
		}

		srcPointsNum = clippedPointsNum;
		curBuf = 1 - curBuf;

		srcPolygonBuf = clipBufV[1 - curBuf];
		dstPolygonBuf = clipBufV[curBuf];

		if (srcPointsNum == 0)
			break;
	}

	// Copying resulting clipped poly to the output buffers
	// TODO: avoid copying by just returning the pointer to final clipBuf
	*numVertsClipped = srcPointsNum;
	for (int pi = 0; pi < srcPointsNum; ++pi)
	{
		clippedPoly[pi] = clipBufV[1-curBuf][pi];
	}
}

// Contact reduction - keep only vertices that expand the contact rectangle
// TODO: speed up by removing already processed vertices from the queue
void reduceContacts(const math::Vec3 * inputContacts, uint32_t numInputContacts, math::Vec3 * outputContacts, uint32_t * numOutputContacts)
{
	using namespace math;

	uint32_t numFinalContactPoints = 0;

	// Stage0: select starting point
	// TODO: selecting point at idx 0 is not good due to time coherency problems
	//	(point0 could be spatially in different places), so it's better to choose
	//	a consistent point with max X or something
	// TODO: once point is taken, it shouldn't be considered in the next check
	outputContacts[numFinalContactPoints++] = inputContacts[0];

	// Stage1: Find farthest point from pt0
	int nextPointIndex = 0;
	float maxLen = 0.0f;
	for (uint32_t cpi = 0; cpi < numInputContacts; ++cpi)
	{
		float len = (inputContacts[cpi] - outputContacts[0]).sqLen();
		if (maxLen < len)
		{
			maxLen = len;
			nextPointIndex = cpi;
		}
	}
	outputContacts[numFinalContactPoints++] = inputContacts[nextPointIndex];

	// Stage2: Find point that will build maximum area triangle with the edge we already have.
	//	Storing maxCossProd for the Stage3 where we need to find triangles on the other side
	Vec3 finalContactEdge = outputContacts[1] - outputContacts[0];
	Vec3 maxCossProd = Vec3C(0.0f, 1.0f, 0.0f);
	nextPointIndex = 0;
	float maxArea = 0.0f;
	for (uint32_t cpi = 0; cpi < numInputContacts; ++cpi)
	{
		// Cross product is an area of parallelogram based on two vectors, we can use that 
		Vec3 crossProd = finalContactEdge.cross(inputContacts[cpi] - outputContacts[0]);
		float area = crossProd.sqLen();
		if (maxArea < area)
		{
			maxArea = area;
			maxCossProd = crossProd;
			nextPointIndex = cpi;
		}
	}
	outputContacts[numFinalContactPoints++] = inputContacts[nextPointIndex];

	// Stage3: Find the second triangle with maximum area, with a constraint that it should
	//	be on another side of the contact egde
	nextPointIndex = 0;
	maxArea = 0.0f;
	for (uint32_t cpi = 0; cpi < numInputContacts; ++cpi)
	{
		Vec3 crossProd = finalContactEdge.cross(inputContacts[cpi] - outputContacts[0]);
		float area = crossProd.sqLen();
		if (maxArea < area)
		{
			// Triangle should be to the other side of the finalContactEdge
			if (crossProd.dot(maxCossProd) < 0.0f)
			{
				maxArea = area;
				nextPointIndex = cpi;
			}
		}
	}
	outputContacts[numFinalContactPoints++] = inputContacts[nextPointIndex];

	// TODO: remove duplicates sq 1e-10

	*numOutputContacts = numFinalContactPoints;
};

// Calculates face normal, given the approximate normal direction
math::Vec3 computeFaceNormal(math::Vec3 * faceVertices, uint32_t numFaceVertices, const math::Vec3 & faceNormalDir)
{
	using namespace math;

	Vec3 faceNormal = Vec3C();
	Vec3 edge01 = faceVertices[1]-faceVertices[0];
	for (uint32_t pi = 2; pi < numFaceVertices; ++pi)
	{
		Vec3 intermNormal = edge01.cross(faceVertices[pi]-faceVertices[0]);
		if (intermNormal.dot(faceNormalDir) > 0.0f)
			faceNormal += intermNormal;
		else
			faceNormal -= intermNormal;
	}
	faceNormal.normalize();
	return faceNormal;
};

void clipEdgeAgainstFace(math::Vec3 * edgeVertices, const math::Vec3 * faceVertices, uint32_t numFaceVertices, const math::Vec3 & faceNormal)
{
	using namespace math;

	// TODO: add signalling of an edge actually being outside of the face poly prism

	// Clip the edge with face edge-planes, testing really only face edges that intersect the
	//	candidate line (the face edge vertices will line on a different sides of candidate
	//	edge plane)
	Vec3 edge = edgeVertices[1] - edgeVertices[0];
	Vec3 candidateEdgeNormal = faceNormal.cross(edge);
	float candidateEdgeNormalD = candidateEdgeNormal.dot(edgeVertices[0]);
	for (uint32_t pi = 0; pi < numFaceVertices; ++pi)
	{
		uint32_t pi1 = (pi+1) % numFaceVertices;
		float p0D = candidateEdgeNormal.dot(faceVertices[pi]) - candidateEdgeNormalD;
		float p1D = candidateEdgeNormal.dot(faceVertices[pi1]) - candidateEdgeNormalD;
		if (p0D * p1D < 0.0f)
		{
			// Face edge plane intersects the candidate line
			Vec3 faceEdgeNormal = faceNormal.cross(faceVertices[pi1] - faceVertices[pi]);
			float faceEdgeNormalD = faceEdgeNormal.dot(faceVertices[pi]);

			float e0Dist = faceEdgeNormal.dot(edgeVertices[0]) - faceEdgeNormalD;

			// Two ways to interpret intersection point:
			// Algebraic:
			//	fn * (e0 + e01*t) - fD = 0
			//	fn * e0 + t * fn * e01 = D
			//	t * fn * e01 = D - fn * e0
			//	t = (D - fn * e0) / (fn * e01)
			// Geometric:
			//	how many projections of edge are required to get to the origin (intersection point)
			//	from the projection of edge point0

			float intersection = (-e0Dist) / (faceEdgeNormal.dot(edge));
			// Check if intersection point is on the candidate edge, and replace
			//	the point that lies outside
			if (intersection > 0.0f && intersection < 1.0f)
			{
				if (e0Dist > 0.0f)
				{
					edgeVertices[0] += intersection*edge;
				}
				else
				{
					// TODO: calculate via shift
					edgeVertices[1] -= (1.0f - intersection)*edge;
				}
				edge = edgeVertices[1] - edgeVertices[0];
			}
		}
	}
};
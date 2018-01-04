#pragma once

#include <assert.h>
#include <vector>

#include <math/AuxMath.h>
#include <math/Vec3.h>
#include <math/Quat.h>

class ConvexHullShape
{
protected:
public:

	enum class Type
	{
		eSphere,
		eCapsule,
		eCylinder,
		eCone,
		eBox,
		eCustom,

		eNUM_ENTRIES
	};
	
	// Mass properties (all for unit density - this should be taken into account on higher levels)
	math::Vec3 m_com;
	math::Mat33 m_localInertia;
	float m_volume;
	virtual void calculateMassPropertiesUnitDensity() = 0;

	virtual Type getType() const = 0;
	virtual math::Vec3 getSupportVertexLocal(const math::Vec3 & direction) = 0;
	virtual math::Vec3 getSupportVertex(const math::Vec3 & direction, const math::Vec3 & pos, const math::Quat & rot) = 0;
	virtual math::Vec3 getOrigin() const = 0;
	virtual void setOrigin(const math::Vec3 & origin) = 0;
	virtual math::Mat33 getRotation() const = 0;
	virtual void setRotation(const math::Mat33 & rotation) = 0;
};

// Function that tries to find convex hull triangle mesh by ~uniformly sampling a sphere
//	and fetching support vertices in those directions. Sampling is based on constructing
//	geodesic sphere from subdivision of the original polytope (in this case, octahedron).
void populateHullTriangles(ConvexHullShape * hullShape, std::vector<math::Vec3> * hullTriangles)
{
	using namespace math;

	assert(hullTriangles);

	struct SupportDirectionTriangle
	{
		Vec3 n0;
		Vec3 n1;
		Vec3 n2;
		int subDivDepth;
	};

	std::vector<SupportDirectionTriangle> supportDirTris;

	// Building initial octahedron
	supportDirTris.push_back({ Vec3C( 0.0f,  0.0f,  1.0f), Vec3C( 1.0f,  0.0f,  0.0f), Vec3C( 0.0f,  1.0f,  0.0f), 0 }); // AABBmax
	supportDirTris.push_back({ Vec3C( 0.0f,  0.0f, -1.0f), Vec3C( 0.0f,  1.0f,  0.0f), Vec3C( 1.0f,  0.0f,  0.0f), 0 });
	supportDirTris.push_back({ Vec3C( 0.0f,  1.0f,  0.0f), Vec3C( 0.0f,  0.0f, -1.0f), Vec3C(-1.0f,  0.0f,  0.0f), 0 });
	supportDirTris.push_back({ Vec3C(-1.0f,  0.0f,  0.0f), Vec3C( 0.0f,  0.0f,  1.0f), Vec3C( 0.0f,  1.0f,  0.0f), 0 });
	supportDirTris.push_back({ Vec3C( 0.0f,  0.0f,  1.0f), Vec3C( 0.0f, -1.0f,  0.0f), Vec3C( 1.0f,  0.0f,  0.0f), 0 });
	supportDirTris.push_back({ Vec3C( 0.0f,  0.0f, -1.0f), Vec3C( 1.0f,  0.0f,  0.0f), Vec3C( 0.0f, -1.0f,  0.0f), 0 });
	supportDirTris.push_back({ Vec3C( 0.0f, -1.0f,  0.0f), Vec3C(-1.0f,  0.0f,  0.0f), Vec3C( 0.0f,  0.0f, -1.0f), 0 }); // AABBmin
	supportDirTris.push_back({ Vec3C(-1.0f,  0.0f,  0.0f), Vec3C( 0.0f, -1.0f,  0.0f), Vec3C( 0.0f,  0.0f,  1.0f), 0 });

	// Go through all triangles, subdividing each (1 tri -> 4 tri), use the vertices of subdivided triangles
	//	as directions to fetch support mapping and form approximate convex hull triangle
	while (supportDirTris.size() > 0)
	{
		SupportDirectionTriangle & curSupportTri = supportDirTris.back();

		// Depth of the subdivision
		const int subDivDepthMax = 3;
		if (curSupportTri.subDivDepth < subDivDepthMax)
		{
			Vec3 n0 = curSupportTri.n0, n1 = curSupportTri.n1, n2 = curSupportTri.n2;

			// Subdividing triangle
			Vec3 mid_n0_n1 = 0.5f * (n0 + n1);
			mid_n0_n1.normalize();

			Vec3 mid_n1_n2 = 0.5f * (n1 + n2);
			mid_n1_n2.normalize();

			Vec3 mid_n2_n0 = 0.5f * (n2 + n0);
			mid_n2_n0.normalize();

			// Reuse source triangle
			curSupportTri.n0 = mid_n0_n1;
			curSupportTri.n1 = mid_n1_n2;
			curSupportTri.n2 = mid_n2_n0;
			int subDivDepthNext = ++curSupportTri.subDivDepth;

			supportDirTris.push_back({ n0, mid_n0_n1, mid_n2_n0, subDivDepthNext });
			supportDirTris.push_back({ mid_n0_n1, n1, mid_n1_n2, subDivDepthNext });
			supportDirTris.push_back({ mid_n2_n0, mid_n1_n2, n2, subDivDepthNext });
		}
		else
		{
			Vec3 sv0 = hullShape->getSupportVertexLocal(curSupportTri.n0);
			Vec3 sv1 = hullShape->getSupportVertexLocal(curSupportTri.n1);
			Vec3 sv2 = hullShape->getSupportVertexLocal(curSupportTri.n2);

			supportDirTris.pop_back();
			hullTriangles->push_back(sv0);
			hullTriangles->push_back(sv1);
			hullTriangles->push_back(sv2);
		}
	}
}

// Simplistic duplicate/degenerate triangles filtering, based on primitive quantization
void filterHullTriangles(std::vector<math::Vec3> * hullTriangles, size_t firstTriVertexIdx = 0)
{
	using namespace math;

	struct vtxQ
	{
		int x, y, z;
	};
	auto quantize = [](const Vec3 & v) -> vtxQ
	{
		const float stepSize = 1e-4f;
		vtxQ vq;
		vq.x = (int)(v.x / stepSize);
		vq.y = (int)(v.y / stepSize);
		vq.z = (int)(v.z / stepSize);
		return vq;
	};

	size_t numTriangleVertices = hullTriangles->size() - firstTriVertexIdx;

	// Filter triangles
	std::vector<vtxQ> hullTrianglesQ;
	hullTrianglesQ.reserve(numTriangleVertices);

	std::vector<Vec3> preFilterHull(hullTriangles->begin() + firstTriVertexIdx, hullTriangles->end());
	hullTriangles->resize(firstTriVertexIdx);

	const size_t totalNumTriangles = numTriangleVertices / 3;
	for (size_t triSrc = 0; triSrc < totalNumTriangles; ++triSrc)
	{
		bool skipTriangle = false;

		vtxQ vtxSrc[3] =
		{
			quantize(preFilterHull[triSrc*3+0]),
			quantize(preFilterHull[triSrc*3+1]),
			quantize(preFilterHull[triSrc*3+2])
		};

		// Check if triangle is degenerate (line or point)
		if (vtxSrc[0].x == vtxSrc[1].x && vtxSrc[0].y == vtxSrc[1].y && vtxSrc[0].z == vtxSrc[1].z ||
			vtxSrc[2].x == vtxSrc[1].x && vtxSrc[2].y == vtxSrc[1].y && vtxSrc[2].z == vtxSrc[1].z ||
			vtxSrc[0].x == vtxSrc[2].x && vtxSrc[0].y == vtxSrc[2].y && vtxSrc[0].z == vtxSrc[2].z)
		{
			skipTriangle = true;
		}

		size_t numTrisToProcess = (hullTriangles->size()-firstTriVertexIdx) / 3;
		for (size_t triDst = 0; !skipTriangle && triDst < numTrisToProcess; ++triDst)
		{
			bool triMatch = true;

			// Triangle vertices lie in memeory in succession
			vtxQ * vtxDst = &hullTrianglesQ[triDst*3];
			for (int vs = 0; vs < 3; ++vs)
			{
				bool vtxMatch = false;
				for (int vd = 0; vd < 3; ++vd)
				{
					if (vtxSrc[vs].x == vtxDst[vd].x &&
						vtxSrc[vs].y == vtxDst[vd].y &&
						vtxSrc[vs].z == vtxDst[vd].z)
					{
						vtxMatch = true;
						break;
					}
				}

				if (!vtxMatch)
				{
					triMatch = false;
					break;
				}
			}

			if (triMatch)
			{
				skipTriangle = true;
				break;
			}
		}

		if (!skipTriangle)
		{
			hullTriangles->push_back(preFilterHull[triSrc*3+0]);
			hullTriangles->push_back(preFilterHull[triSrc*3+1]);
			hullTriangles->push_back(preFilterHull[triSrc*3+2]);

			hullTrianglesQ.push_back(vtxSrc[0]);
			hullTrianglesQ.push_back(vtxSrc[1]);
			hullTrianglesQ.push_back(vtxSrc[2]);
		}
	}

	const size_t numVerticesFinal = hullTriangles->size();

	// Check remaining triangles, to see if any of them have wrong winding
	Vec3 geomCenter = Vec3C();
	for (size_t triVtx = 0; triVtx < numVerticesFinal; ++triVtx)
	{
		geomCenter += (*hullTriangles)[firstTriVertexIdx+triVtx];
	}
	geomCenter /= (float)numVerticesFinal;

	const size_t numTrianglesFinal = numVerticesFinal / 3;
	for (size_t triIdx = 0; triIdx < numTrianglesFinal; ++triIdx)
	{
		const size_t triOffset = firstTriVertexIdx+triIdx*3;
		const Vec3 & p0 = (*hullTriangles)[triOffset+0];
		const Vec3 & p1 = (*hullTriangles)[triOffset+1];
		const Vec3 & p2 = (*hullTriangles)[triOffset+2];
		Vec3 n = (p1 - p0).cross(p2 - p0);
		// If centroid is in the +ve halfspace of the triangle,
		//	then this triangle has incorrect winding
		if (n.dot(p0) < n.dot(geomCenter))
		{
			Vec3 tempVec = p1;
			(*hullTriangles)[triOffset+1] = (*hullTriangles)[triOffset+2];
			(*hullTriangles)[triOffset+2] = tempVec;
		}
	}
}

// Function builds hull mesh from recursive geosphere sampling, then filters duplicate/degenerate triangles
void buildHullTriangles(ConvexHullShape * hullShape, std::vector<math::Vec3> * hullTriangles)
{
	// Remove/uncomment if appending vertices is required
	hullTriangles->resize(0);

	size_t firstVertexIndex = hullTriangles->size();
	populateHullTriangles(hullShape, hullTriangles);
	filterHullTriangles(hullTriangles, firstVertexIndex);
}

// Calculates mass properties of a convex hull from its (closed) triangle mesh
void calcHullMassProperties(const std::vector<math::Vec3> & hullTriangles, float * volume, math::Vec3 * com, math::Mat33 * inertia)
{
	using namespace math;

	assert(volume && com && inertia);

	const float density = 1.0f;

	float accumM = 0.0f;
	Vec3 accumCoM = Vec3C();
	Mat33 accumC;
	accumC.zero();

	// Covariance matrix for canonical tetrahedron:
	//	(0, 0, 0) v0
	//	(1, 0, 0) v1
	//	(0, 1, 0) v2
	//	(0, 0, 1) v3
	const float Cxx = 1.0f / 60.0f;
	const float Cxy = 1.0f / 120.0f;
	const Mat33 canonicalC =
	{
		Cxx, Cxy, Cxy,
		Cxy, Cxx, Cxy,
		Cxy, Cxy, Cxx
	};

	Mat33 mA;
	mA.zero();

	auto outerProduct = [](const Vec3 & vec0, const Vec3 & vec1) -> Mat33
	{
		Mat33 result =
		{
			vec0.x*vec1.x, vec0.x*vec1.y, vec0.x*vec1.z,
			vec0.y*vec1.x, vec0.y*vec1.y, vec0.y*vec1.z,
			vec0.z*vec1.x, vec0.z*vec1.y, vec0.z*vec1.z
		};
		return result;
	};

	// TODO: a lot of simplifications if refPoint will actually be hardcoded to (0, 0, 0)
	const Vec3 & refPoint = Vec3C();
	const size_t numTriangles = hullTriangles.size() / 3;
	for (size_t triIdx = 0; triIdx < numTriangles; ++triIdx)
	{
		Vec3 p0 = hullTriangles[triIdx*3+0];
		Vec3 p1 = hullTriangles[triIdx*3+1];
		Vec3 p2 = hullTriangles[triIdx*3+2];

		mA.setBasis0(p0 - refPoint);
		mA.setBasis1(p1 - refPoint);
		mA.setBasis2(p2 - refPoint);

		float mADet = mA.det();

		float tetVol = (1.0f / 6.0f) * mADet;
		float tetM = density * tetVol;

		// In case tri was degenerate
		if (tetM == 0.0f)
			continue;

		Vec3 tetCoM = 0.25f * (p0 + p1 + p2 + refPoint);

		// w0 - v0, canonical tetrahedron v0 = (0, 0, 0)
		Vec3 tetShift = refPoint;

		// Rotate covariance matrix with matrix A: C' = det(A) * (A * C * A^T)
		Mat33 rotC = mADet * (mA * canonicalC * mA.getTransposed());
		// Translate covariance matrix with vector dx: C'' = C' + m(dx*CoM^T + CoM*dx^T + dx*dx^T)
		Mat33 trnC = rotC + tetM * (outerProduct(tetShift, tetCoM) + outerProduct(tetCoM, tetShift) + outerProduct(tetShift, tetShift));

		// Accumulate mass properties
		accumC += trnC;
		accumCoM = (accumCoM*accumM + tetCoM*tetM) / (accumM + tetM);
		accumM += tetM;
	}

	Vec3 bodyCoM = Vec3C();
	Vec3 finalShift = refPoint - bodyCoM;
	// Translate covariance matrix to the final CoM of the body
	Mat33 finalC = accumC + accumM * (outerProduct(finalShift, accumCoM) + outerProduct(accumCoM, finalShift) + outerProduct(finalShift, finalShift));

	Mat33 I33;
	I33.identity();
	float trFinalC = finalC._00 + finalC._11 + finalC._22;
	*inertia = trFinalC * I33 - finalC;
	*com = accumCoM;
	*volume = accumM;
};
#include "physics/joints.h"
#include "physics/phys_helpers.h"

namespace physics
{

//////////////////////////////////////////////////////////////////////////
// BallSocket
//////////////////////////////////////////////////////////////////////////
void BallSocket::init(float ERP, float CFM, const math::Vec3 & anchorPoint, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	math::Vec3 bodyOriginPos;

	// Calculating Anchor Points in Local Space (vs user-defined origin, not CoM)
	bodyOriginPos = m_body0L->m_pos + m_body0R->m_rot.rotate(-m_body0R->m_com);
	m_bodyOriginAnchorPoints[0] = m_body0R->m_rot.getConjugated().rotate(anchorPoint - bodyOriginPos);

	if (m_body1L != nullptr)
	{
		bodyOriginPos = m_body1L->m_pos + m_body1R->m_rot.rotate(-m_body1R->m_com);
		m_bodyOriginAnchorPoints[1] = m_body1R->m_rot.getConjugated().rotate(anchorPoint - bodyOriginPos);
	}
	else
	{
		m_bodyOriginAnchorPoints[1] = anchorPoint;
	}

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}

uint32_t BallSocket::getNumRowsVel() const
{
	return c_numRows;
}
uint32_t BallSocket::getNumRowsVelFriction() const
{
	return c_numRows;
}

void BallSocket::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	// Constraint equation:
	//	c = p_A + [R_A]a_AL - (p_B + [R_B]a_BL) = 0
	// where p_<A|B> - A/B center of masses,
	//	R_<A|B> - A/B rotation matrices,
	//	a_<A|B>L - local anchor point coordinates

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	// Additional tripples
	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx, Row1Idx, Row2Idx;

	Row0Idx = m_startIdx + 0;
	Row1Idx = m_startIdx + 1;
	Row2Idx = m_startIdx + 2;

	math::Vec3 wsAnchorPoints[2];

	// Linear Body1
	ptr_m_J_00[Row0Idx] =  1.0f;
	ptr_m_J_01[Row0Idx] =  0.0f;
	ptr_m_J_02[Row0Idx] =  0.0f;

	ptr_m_J_00[Row1Idx] =  0.0f;
	ptr_m_J_01[Row1Idx] =  1.0f;
	ptr_m_J_02[Row1Idx] =  0.0f;

	ptr_m_J_00[Row2Idx] =  0.0f;
	ptr_m_J_01[Row2Idx] =  0.0f;
	ptr_m_J_02[Row2Idx] =  1.0f;

	// Local anchor point in body1 wrt CoM
	wsAnchorPoints[0] = m_body0R->m_rot.rotate(m_bodyOriginAnchorPoints[0] - m_body0R->m_com);

	// -CrossProdMatrix(b1rot * b1anchor_ls)
	ptr_m_J_03[Row0Idx] = 0.0f;
	ptr_m_J_04[Row0Idx] =  wsAnchorPoints[0].z;
	ptr_m_J_05[Row0Idx] = -wsAnchorPoints[0].y;
	ptr_m_J_03[Row1Idx] = -wsAnchorPoints[0].z;
	ptr_m_J_04[Row1Idx] = 0.0f;
	ptr_m_J_05[Row1Idx] =  wsAnchorPoints[0].x;
	ptr_m_J_03[Row2Idx] =  wsAnchorPoints[0].y;
	ptr_m_J_04[Row2Idx] = -wsAnchorPoints[0].x;
	ptr_m_J_05[Row2Idx] = 0.0f;

	// World-space anchor point attached to body1
	wsAnchorPoints[0] += m_body0L->m_pos;

	if (m_body1L != nullptr)
	{
		// Linear Body2
		ptr_m_J_06[Row0Idx] = -1.0f;
		ptr_m_J_07[Row0Idx] =  0.0f;
		ptr_m_J_08[Row0Idx] =  0.0f;

		ptr_m_J_06[Row1Idx] =  0.0f;
		ptr_m_J_07[Row1Idx] = -1.0f;
		ptr_m_J_08[Row1Idx] =  0.0f;

		ptr_m_J_06[Row2Idx] =  0.0f;
		ptr_m_J_07[Row2Idx] =  0.0f;
		ptr_m_J_08[Row2Idx] = -1.0f;

		// Local anchor point in body2 wrt CoM
		wsAnchorPoints[1] = m_body1R->m_rot.rotate(m_bodyOriginAnchorPoints[1] - m_body1R->m_com);

		// +CrossProdMatrix(b2rot * b2anchor_ls)
		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = -wsAnchorPoints[1].z;
		ptr_m_J_11[Row0Idx] =  wsAnchorPoints[1].y;
		ptr_m_J_09[Row1Idx] =  wsAnchorPoints[1].z;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = -wsAnchorPoints[1].x;
		ptr_m_J_09[Row2Idx] = -wsAnchorPoints[1].y;
		ptr_m_J_10[Row2Idx] =  wsAnchorPoints[1].x;
		ptr_m_J_11[Row2Idx] = 0.0f;

		// World-space anchor point attached to body2
		wsAnchorPoints[1] += m_body1L->m_pos;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_06[Row0Idx] = 0.0f;
		ptr_m_J_06[Row1Idx] = 0.0f;
		ptr_m_J_06[Row2Idx] = 0.0f;
		ptr_m_J_07[Row0Idx] = 0.0f;
		ptr_m_J_07[Row1Idx] = 0.0f;
		ptr_m_J_07[Row2Idx] = 0.0f;
		ptr_m_J_08[Row0Idx] = 0.0f;
		ptr_m_J_08[Row1Idx] = 0.0f;
		ptr_m_J_08[Row2Idx] = 0.0f;
		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_09[Row1Idx] = 0.0f;
		ptr_m_J_09[Row2Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_10[Row2Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = 0.0f;
		ptr_m_J_11[Row2Idx] = 0.0f;

		// Anchor Point is in World Space already
		wsAnchorPoints[1] = m_bodyOriginAnchorPoints[1];
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_12[Row1Idx] = 0.0f;
	ptr_m_J_12[Row2Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_13[Row1Idx] = 0.0f;
	ptr_m_J_13[Row2Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;
	ptr_m_J_14[Row1Idx] = 0.0f;
	ptr_m_J_14[Row2Idx] = 0.0f;
	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_15[Row1Idx] = 0.0f;
	ptr_m_J_15[Row2Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_16[Row1Idx] = 0.0f;
	ptr_m_J_16[Row2Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;
	ptr_m_J_17[Row1Idx] = 0.0f;
	ptr_m_J_17[Row2Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	// Baumgarte Stabilization
	*(ptr_m_J_rhs++) = idt * m_ERP * (wsAnchorPoints[1].x - wsAnchorPoints[0].x);
	*(ptr_m_J_rhs++) = idt * m_ERP * (wsAnchorPoints[1].y - wsAnchorPoints[0].y);
	*(ptr_m_J_rhs++) = idt * m_ERP * (wsAnchorPoints[1].z - wsAnchorPoints[0].z);

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	const float ballsocketLimLo = -FLT_MAX;
	const float ballsocketLimHi =  FLT_MAX;
	*(ptr_m_Lo++) = ballsocketLimLo;
	*(ptr_m_Hi++) = ballsocketLimHi;
	*(ptr_m_Lo++) = ballsocketLimLo;
	*(ptr_m_Hi++) = ballsocketLimHi;
	*(ptr_m_Lo++) = ballsocketLimLo;
	*(ptr_m_Hi++) = ballsocketLimHi;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;

	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_numRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff * m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void BallSocket::updateCopyVel(float dt, SolverBase & solver)
{
	updateCopy(dt, solver, 0.5f);
}
void BallSocket::updateCopyVelFriction(float dt, SolverBase & solver)
{
	updateCopy(dt, solver, 1.0f);
}

void BallSocket::fetchLambdasVel(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
	m_lambda0[2] = solver.m_lambda[m_startIdx + 2];
}
void BallSocket::fetchLambdasVelFriction(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
	m_lambda0[2] = solver.m_lambda[m_startIdx + 2];
}

void BallSocket::getAnchorPoints_BS(math::Vec3 & bsAnchor0, math::Vec3 & bsAnchor1)
{
	bsAnchor0 = m_bodyOriginAnchorPoints[0];
	bsAnchor1 = m_bodyOriginAnchorPoints[1];
}
void BallSocket::setAnchorPoint0(const math::Vec3 & anchorPoint0)
{
	math::Vec3 bodyOriginPos = m_body0L->m_pos + m_body0R->m_rot.rotate(-m_body0R->m_com);
	m_bodyOriginAnchorPoints[0] = m_body0R->m_rot.getConjugated().rotate(anchorPoint0 - bodyOriginPos);
}
void BallSocket::setAnchorPoint1(const math::Vec3 & anchorPoint1)
{
	if (m_body1L != nullptr)
	{
		math::Vec3 bodyOriginPos = m_body1L->m_pos + m_body1R->m_rot.rotate(-m_body1R->m_com);
		m_bodyOriginAnchorPoints[1] = m_body1R->m_rot.getConjugated().rotate(anchorPoint1 - bodyOriginPos);
	}
	else
	{
		m_bodyOriginAnchorPoints[1] = anchorPoint1;
	}
}

//////////////////////////////////////////////////////////////////////////
// Slider
//////////////////////////////////////////////////////////////////////////
void Slider::init(float ERP, float CFM, const math::Vec3 & axis, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	using namespace math;

	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;

	m_ERP = ERP;

	// Calculating Anchor Points in Local Space (vs user-defined origin, not CoM)
	Vec3 worldOffset;
	worldOffset = m_body0L->m_pos;
	m_localAxes[0] = m_body0R->m_rot.getConjugated().rotate(axis);

	Vec3 anchorPoint = body0L->m_pos;
	Vec3 body0OriginPos, body1OriginPos;
	body0OriginPos = m_body0L->m_pos + m_body0R->m_rot.rotate(-m_body0R->m_com);
	m_bodyOriginAnchorPoints[0] = m_body0R->m_rot.getConjugated().rotate(anchorPoint - body0OriginPos);
	if (m_body1L != nullptr)
	{
		anchorPoint += body1L->m_pos;
		anchorPoint *= 0.5f;

		body1OriginPos = m_body1L->m_pos + m_body1R->m_rot.rotate(-m_body1R->m_com);
		m_bodyOriginAnchorPoints[1] = m_body1R->m_rot.getConjugated().rotate(anchorPoint - body1OriginPos);

		worldOffset -= m_body1L->m_pos;
		m_localAxes[1] = m_body1R->m_rot.getConjugated().rotate(axis);

		m_localOffsets[0] = m_body0R->m_rot.getConjugated().rotate(worldOffset);
		m_localOffsets[1] = m_body1R->m_rot.getConjugated().rotate(worldOffset);
	}
	else
	{
		m_localAxes[1] = axis;

		m_bodyOriginAnchorPoints[1] = anchorPoint;

		m_localOffsets[0] = worldOffset;
		m_localOffsets[1] = worldOffset;
	}

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}

uint32_t Slider::getNumRowsVel() const
{
	return c_numRows;
}
uint32_t Slider::getNumRowsVelFriction() const
{
	return c_numRows;
}

void Slider::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	using namespace math;

	// Constraint equation:
	//	c = [tan0 tan1]^T (p_A + [R_A]a_AL - (p_B + [R_B]a_BL)) = [0 0]^T
	// for definitions see BallSocket joint updateCopy,
	// anchor point for slider joint is a midpoint between bodies' CoM

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	// Additional tripples
	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx, Row1Idx;

	Row0Idx = m_startIdx + 0;
	Row1Idx = m_startIdx + 1;

	Vec3 worldOffset;

	Vec3 worldAxes[2];
	worldOffset = m_body0L->m_pos;
	worldAxes[0] = m_body0R->m_rot.rotate(m_localAxes[0]);

	Vec3 wsAnchorPoints[2];
	Vec3 worldLevers[2];
	worldLevers[0] = m_body0R->m_rot.rotate(m_bodyOriginAnchorPoints[0] - m_body0R->m_com);
	wsAnchorPoints[0] = worldLevers[0] + m_body0L->m_pos;

	Vec3 worldOffsetInitial;
	if (m_body1L != nullptr)
	{
		worldLevers[1] = m_body1R->m_rot.rotate(m_bodyOriginAnchorPoints[1] - m_body1R->m_com);
		wsAnchorPoints[1] = worldLevers[1] + m_body1L->m_pos;

		worldOffsetInitial = m_body1R->m_rot.rotate(m_localOffsets[1]);
		worldOffset -= m_body1L->m_pos;
		worldAxes[1] = m_body1R->m_rot.rotate(m_localAxes[1]);
	}
	else
	{
		wsAnchorPoints[1] = m_bodyOriginAnchorPoints[1];

		worldOffsetInitial = Vec3C();
		worldOffset -= m_localOffsets[0];
		worldAxes[1] = m_localAxes[1];
	}

	Vec3 tan0, tan1;
	if (m_body1L != nullptr)
	{
		Vec3 worldAxis = (0.5f * (worldAxes[0] + worldAxes[1])).getNormalized();//(wsAnchorPoints[0] - wsAnchorPoints[1]).getNormalized();
		worldAxis.tangentSpace(tan0, tan1);
	}
	else
	{
		// If there is no body 1, we'd like to keep world axis to maintain persistent world anchoring
		worldAxes[1].tangentSpace(tan0, tan1);
	}

	// Linear Body1
	ptr_m_J_00[Row0Idx] = tan0.x;
	ptr_m_J_01[Row0Idx] = tan0.y;
	ptr_m_J_02[Row0Idx] = tan0.z;

	ptr_m_J_00[Row1Idx] = tan1.x;
	ptr_m_J_01[Row1Idx] = tan1.y;
	ptr_m_J_02[Row1Idx] = tan1.z;

	// Local anchor point in body1 wrt CoM
	Vec3 moment00 = worldLevers[0].cross(tan0);
	Vec3 moment01 = worldLevers[0].cross(tan1);
	float body0AngCoeff = 1.0f;

	ptr_m_J_03[Row0Idx] = body0AngCoeff * moment00.x;
	ptr_m_J_04[Row0Idx] = body0AngCoeff * moment00.y;
	ptr_m_J_05[Row0Idx] = body0AngCoeff * moment00.z;

	ptr_m_J_03[Row1Idx] = body0AngCoeff * moment01.x;
	ptr_m_J_04[Row1Idx] = body0AngCoeff * moment01.y;
	ptr_m_J_05[Row1Idx] = body0AngCoeff * moment01.z;

	if (m_body1L != nullptr)
	{
		// Linear Body2
		ptr_m_J_06[Row0Idx] = -tan0.x;
		ptr_m_J_07[Row0Idx] = -tan0.y;
		ptr_m_J_08[Row0Idx] = -tan0.z;

		ptr_m_J_06[Row1Idx] = -tan1.x;
		ptr_m_J_07[Row1Idx] = -tan1.y;
		ptr_m_J_08[Row1Idx] = -tan1.z;

		// Local anchor point in body2 wrt CoM
		Vec3 moment10 = worldLevers[1].cross(tan0);
		Vec3 moment11 = worldLevers[1].cross(tan1);
		const float body1AngCoeff = -1.0f;

		ptr_m_J_09[Row0Idx] = body1AngCoeff * moment10.x;
		ptr_m_J_10[Row0Idx] = body1AngCoeff * moment10.y;
		ptr_m_J_11[Row0Idx] = body1AngCoeff * moment10.z;

		ptr_m_J_09[Row1Idx] = body1AngCoeff * moment11.x;
		ptr_m_J_10[Row1Idx] = body1AngCoeff * moment11.y;
		ptr_m_J_11[Row1Idx] = body1AngCoeff * moment11.z;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_06[Row0Idx] = 0.0f;
		ptr_m_J_07[Row0Idx] = 0.0f;
		ptr_m_J_08[Row0Idx] = 0.0f;
		ptr_m_J_06[Row1Idx] = 0.0f;
		ptr_m_J_07[Row1Idx] = 0.0f;
		ptr_m_J_08[Row1Idx] = 0.0f;

		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
		ptr_m_J_09[Row1Idx] = 0.0f;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = 0.0f;
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;
	ptr_m_J_12[Row1Idx] = 0.0f;
	ptr_m_J_13[Row1Idx] = 0.0f;
	ptr_m_J_14[Row1Idx] = 0.0f;

	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;
	ptr_m_J_15[Row1Idx] = 0.0f;
	ptr_m_J_16[Row1Idx] = 0.0f;
	ptr_m_J_17[Row1Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	// Baumgarte Stabilization
	*(ptr_m_J_rhs++) = idt * m_ERP * tan0.dot(wsAnchorPoints[1] - wsAnchorPoints[0]);
	*(ptr_m_J_rhs++) = idt * m_ERP * tan1.dot(wsAnchorPoints[1] - wsAnchorPoints[0]);

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	const float sliderLim = FLT_MAX;
	*(ptr_m_Lo++) = -sliderLim;
	*(ptr_m_Hi++) =  sliderLim;
	*(ptr_m_Lo++) = -sliderLim;
	*(ptr_m_Hi++) =  sliderLim;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;

	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_numRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff * m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void Slider::updateCopyVel(float dt, SolverBase & solver)
{
	updateCopy(dt, solver, 0.1f);
}
void Slider::updateCopyVelFriction(float dt, SolverBase & solver)
{
	updateCopy(dt, solver, 1.0f);
}

void Slider::fetchLambdasVel(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
}
void Slider::fetchLambdasVelFriction(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
}

math::Vec3 Slider::getWorldAxis0() const
{
	return m_body0R->m_rot.rotate(m_localAxes[0]);
}
math::Vec3 Slider::getWorldAxis1() const
{
	if (m_body1R)
		return m_body1R->m_rot.rotate(m_localAxes[1]);
	else
		return m_localAxes[1];
}

//////////////////////////////////////////////////////////////////////////
// FixedRotation
//////////////////////////////////////////////////////////////////////////
void FixedRotation::init(float ERP, float CFM, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	if (m_body1R)
		m_initialRelRot = m_body1R->m_rot.getConjugated() * m_body0R->m_rot;
	else
		m_initialRelRot = m_body0R->m_rot;
	m_initialRelRot.normalize();

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}

uint32_t FixedRotation::getNumRowsVel() const
{
	return c_numRows;
}
uint32_t FixedRotation::getNumRowsVelFriction() const
{
	return c_numRows;
}

void FixedRotation::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	using namespace math;

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx, Row1Idx, Row2Idx;

	Row0Idx = m_startIdx + 0;
	Row1Idx = m_startIdx + 1;
	Row2Idx = m_startIdx + 2;

	// Linear Body1
	ptr_m_J_00[Row0Idx] =  0.0f;
	ptr_m_J_01[Row0Idx] =  0.0f;
	ptr_m_J_02[Row0Idx] =  0.0f;

	ptr_m_J_00[Row1Idx] =  0.0f;
	ptr_m_J_01[Row1Idx] =  0.0f;
	ptr_m_J_02[Row1Idx] =  0.0f;

	ptr_m_J_00[Row2Idx] =  0.0f;
	ptr_m_J_01[Row2Idx] =  0.0f;
	ptr_m_J_02[Row2Idx] =  0.0f;

	// -CrossProdMatrix(b1rot * b1anchor_ls)
	ptr_m_J_03[Row0Idx] = 1.0f;
	ptr_m_J_04[Row0Idx] = 0.0f;
	ptr_m_J_05[Row0Idx] = 0.0f;
	ptr_m_J_03[Row1Idx] = 0.0f;
	ptr_m_J_04[Row1Idx] = 1.0f;
	ptr_m_J_05[Row1Idx] = 0.0f;
	ptr_m_J_03[Row2Idx] = 0.0f;
	ptr_m_J_04[Row2Idx] = 0.0f;
	ptr_m_J_05[Row2Idx] = 1.0f;

	if (m_body1L != nullptr)
	{
		// Linear Body2
		ptr_m_J_06[Row0Idx] =  0.0f;
		ptr_m_J_07[Row0Idx] =  0.0f;
		ptr_m_J_08[Row0Idx] =  0.0f;

		ptr_m_J_06[Row1Idx] =  0.0f;
		ptr_m_J_07[Row1Idx] =  0.0f;
		ptr_m_J_08[Row1Idx] =  0.0f;

		ptr_m_J_06[Row2Idx] =  0.0f;
		ptr_m_J_07[Row2Idx] =  0.0f;
		ptr_m_J_08[Row2Idx] =  0.0f;

		// +CrossProdMatrix(b2rot * b2anchor_ls)
		ptr_m_J_09[Row0Idx] = -1.0f;
		ptr_m_J_10[Row0Idx] =  0.0f;
		ptr_m_J_11[Row0Idx] =  0.0f;
		ptr_m_J_09[Row1Idx] =  0.0f;
		ptr_m_J_10[Row1Idx] = -1.0f;
		ptr_m_J_11[Row1Idx] =  0.0f;
		ptr_m_J_09[Row2Idx] =  0.0f;
		ptr_m_J_10[Row2Idx] =  0.0f;
		ptr_m_J_11[Row2Idx] = -1.0f;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_06[Row0Idx] = 0.0f;
		ptr_m_J_06[Row1Idx] = 0.0f;
		ptr_m_J_06[Row2Idx] = 0.0f;
		ptr_m_J_07[Row0Idx] = 0.0f;
		ptr_m_J_07[Row1Idx] = 0.0f;
		ptr_m_J_07[Row2Idx] = 0.0f;
		ptr_m_J_08[Row0Idx] = 0.0f;
		ptr_m_J_08[Row1Idx] = 0.0f;
		ptr_m_J_08[Row2Idx] = 0.0f;
		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_09[Row1Idx] = 0.0f;
		ptr_m_J_09[Row2Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_10[Row2Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = 0.0f;
		ptr_m_J_11[Row2Idx] = 0.0f;
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_12[Row1Idx] = 0.0f;
	ptr_m_J_12[Row2Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_13[Row1Idx] = 0.0f;
	ptr_m_J_13[Row2Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;
	ptr_m_J_14[Row1Idx] = 0.0f;
	ptr_m_J_14[Row2Idx] = 0.0f;
	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_15[Row1Idx] = 0.0f;
	ptr_m_J_15[Row2Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_16[Row1Idx] = 0.0f;
	ptr_m_J_16[Row2Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;
	ptr_m_J_17[Row1Idx] = 0.0f;
	ptr_m_J_17[Row2Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	Quat relRot;
	if (m_body1R)
		relRot = m_body1R->m_rot.getConjugated() * m_body0R->m_rot;
	else
		relRot = m_body0R->m_rot;
	relRot.normalize();

	Quat errRot;
	errRot = relRot.getConjugated() * m_initialRelRot;

	// Small angle approximation, sin(ang/2) ~= ang/2 at very small ang
	Vec3 errAxis = errRot.v;
	const float quatCoeff = 2.0f;

	// Baumgarte Stabilization
	*(ptr_m_J_rhs++) = idt * m_ERP * quatCoeff * errAxis.x;
	*(ptr_m_J_rhs++) = idt * m_ERP * quatCoeff * errAxis.y;
	*(ptr_m_J_rhs++) = idt * m_ERP * quatCoeff * errAxis.z;

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;


	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_numRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff*m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void FixedRotation::updateCopyVel(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 0.3f);
}
void FixedRotation::updateCopyVelFriction(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 1.0f);
}

void FixedRotation::fetchLambdasVel(SolverBase &solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
	m_lambda0[2] = solver.m_lambda[m_startIdx + 2];
}
void FixedRotation::fetchLambdasVelFriction(SolverBase &solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
	m_lambda0[2] = solver.m_lambda[m_startIdx + 2];
}

//////////////////////////////////////////////////////////////////////////
// AxisRotation
//////////////////////////////////////////////////////////////////////////
void AxisRotation::init(float ERP, float CFM, const math::Vec3 & axis, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	m_localAxes[0] = m_body0R->m_rot.getConjugated().rotate(axis);
	if (m_body1R)
	{
		m_localAxes[1] = m_body1R->m_rot.getConjugated().rotate(axis);
	}
	else
	{
		m_localAxes[1] = axis;
	}

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}

uint32_t AxisRotation::getNumRowsVel() const
{
	return c_numRows;
}
uint32_t AxisRotation::getNumRowsVelFriction() const
{
	return c_numRows;
}

void AxisRotation::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	using namespace math;

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx, Row1Idx;

	Row0Idx = m_startIdx + 0;
	Row1Idx = m_startIdx + 1;

	Vec3 worldAxes[2];
	worldAxes[0] = m_body0R->m_rot.rotate(m_localAxes[0]);
	if (m_body1R)
	{
		worldAxes[1] = m_body1R->m_rot.rotate(m_localAxes[1]);
	}
	else
	{
		worldAxes[1] = m_localAxes[1];
	}

	Vec3 tan0, tan1;
	worldAxes[0].tangentSpace(tan0, tan1);

	// Linear Body1
	ptr_m_J_00[Row0Idx] =  0.0f;
	ptr_m_J_01[Row0Idx] =  0.0f;
	ptr_m_J_02[Row0Idx] =  0.0f;

	ptr_m_J_00[Row1Idx] =  0.0f;
	ptr_m_J_01[Row1Idx] =  0.0f;
	ptr_m_J_02[Row1Idx] =  0.0f;

	// -CrossProdMatrix(b1rot * b1anchor_ls)
	ptr_m_J_03[Row0Idx] =  tan0.x;
	ptr_m_J_04[Row0Idx] =  tan0.y;
	ptr_m_J_05[Row0Idx] =  tan0.z;
	ptr_m_J_03[Row1Idx] =  tan1.x;
	ptr_m_J_04[Row1Idx] =  tan1.y;
	ptr_m_J_05[Row1Idx] =  tan1.z;

	if (m_body1L != nullptr)
	{
		// Linear Body2
		ptr_m_J_06[Row0Idx] =  0.0f;
		ptr_m_J_07[Row0Idx] =  0.0f;
		ptr_m_J_08[Row0Idx] =  0.0f;

		ptr_m_J_06[Row1Idx] =  0.0f;
		ptr_m_J_07[Row1Idx] =  0.0f;
		ptr_m_J_08[Row1Idx] =  0.0f;

		// +CrossProdMatrix(b2rot * b2anchor_ls)
		ptr_m_J_09[Row0Idx] = -tan0.x;
		ptr_m_J_10[Row0Idx] = -tan0.y;
		ptr_m_J_11[Row0Idx] = -tan0.z;
		ptr_m_J_09[Row1Idx] = -tan1.x;
		ptr_m_J_10[Row1Idx] = -tan1.y;
		ptr_m_J_11[Row1Idx] = -tan1.z;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_06[Row0Idx] = 0.0f;
		ptr_m_J_07[Row0Idx] = 0.0f;
		ptr_m_J_08[Row0Idx] = 0.0f;
		ptr_m_J_06[Row1Idx] = 0.0f;
		ptr_m_J_07[Row1Idx] = 0.0f;
		ptr_m_J_08[Row1Idx] = 0.0f;

		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
		ptr_m_J_09[Row1Idx] = 0.0f;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = 0.0f;
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;
	ptr_m_J_12[Row1Idx] = 0.0f;
	ptr_m_J_13[Row1Idx] = 0.0f;
	ptr_m_J_14[Row1Idx] = 0.0f;

	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;
	ptr_m_J_15[Row1Idx] = 0.0f;
	ptr_m_J_16[Row1Idx] = 0.0f;
	ptr_m_J_17[Row1Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	Vec3 crossAxes = worldAxes[0].cross(worldAxes[1]);

	const float corrCoeff = 1.0f;
	*(ptr_m_J_rhs++) = idt * m_ERP * corrCoeff * (tan0.dot(crossAxes));
	*(ptr_m_J_rhs++) = idt * m_ERP * corrCoeff * (tan1.dot(crossAxes));

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;


	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_numRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff*m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void AxisRotation::updateCopyVel(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 0.3f);
}
void AxisRotation::updateCopyVelFriction(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 1.0f);
}

void AxisRotation::fetchLambdasVel(SolverBase &solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
}
void AxisRotation::fetchLambdasVelFriction(SolverBase &solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
}

//////////////////////////////////////////////////////////////////////////
// AxisRotationLimit
//////////////////////////////////////////////////////////////////////////
void AxisRotationLimit::init(float ERP, float CFM, const math::Vec3 & axis, float angLo, float angHi, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	memset(m_lambda0, 0, c_maxNumRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	m_localAxes[0] = m_body0R->m_rot.getConjugated().rotate(axis);
	if (m_body1R)
	{
		m_initialRelRot = m_body1R->m_rot.getConjugated() * m_body0R->m_rot;
		m_localAxes[1] = m_body1R->m_rot.getConjugated().rotate(axis);
	}
	else
	{
		m_initialRelRot = m_body0R->m_rot;
		m_localAxes[1] = axis;
	}
	m_initialRelRot.normalize();

	m_sinHalfLo = sinf(0.5f * angLo);
	m_sinHalfHi = sinf(0.5f * angHi);

	// 5 degree tolerance
	setToleranceAngle(Deg2RadMul * 1.0f);

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}

void AxisRotationLimit::onStepStart(float dt)
{
	using namespace math;

	if (m_body1R)
		m_precompute.relRot = m_body1R->m_rot.getConjugated() * m_body0R->m_rot;
	else
		m_precompute.relRot = m_body0R->m_rot;
	m_precompute.relRot.normalize();

	m_precompute.errRot = m_initialRelRot.getConjugated() * m_precompute.relRot;

	if (m_precompute.errRot.s < 0.0f)
	{
		m_precompute.errRot.s = -m_precompute.errRot.s;
		m_precompute.errRot.v = -m_precompute.errRot.v;
	}

	m_precompute.worldAxes[0] = m_body0R->m_rot.rotate(m_localAxes[0]);
	if (m_body1R)
	{
		m_precompute.worldAxes[1] = m_body1R->m_rot.rotate(m_localAxes[1]);
	}
	else
	{
		m_precompute.worldAxes[1] = m_localAxes[1];
	}
}

void AxisRotationLimit::calcLimitSinHalfAngle(float * limitSinHalfAngle) const
{
	*limitSinHalfAngle = m_precompute.errRot.v.dot(m_precompute.worldAxes[0]);
}

AxisRotationLimit::LimitState AxisRotationLimit::testLimit(float limitSinHalfAngle) const
{
	if (limitSinHalfAngle < m_sinHalfLo)
		return LimitState::eLoViolation;
	else if (limitSinHalfAngle > m_sinHalfHi)
		return LimitState::eHiViolation;

	return LimitState::eFree;
}

uint32_t AxisRotationLimit::getNumRowsVel() const
{
	float limitSinHalfAngle;
	calcLimitSinHalfAngle(&limitSinHalfAngle);

	LimitState limitState = testLimit(limitSinHalfAngle);
	if (limitState == LimitState::eFree)
		return 0;
	else
		return c_maxNumRows;
}
uint32_t AxisRotationLimit::getNumRowsVelFriction() const
{
	float limitSinHalfAngle;
	calcLimitSinHalfAngle(&limitSinHalfAngle);

	LimitState limitState = testLimit(limitSinHalfAngle);
	if (limitState == LimitState::eFree)
		return 0;
	else
		return c_maxNumRows;
}

void AxisRotationLimit::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	using namespace math;

	float limitSinHalfAngle;
	calcLimitSinHalfAngle(&limitSinHalfAngle);

	m_activeLimitState = testLimit(limitSinHalfAngle);
	if (m_activeLimitState == LimitState::eFree)
		return;

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_maxNumRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_maxNumRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx;

	Row0Idx = m_startIdx;

	Vec3 & worldAxis0 = m_precompute.worldAxes[0];
	Vec3 & worldAxis1 = m_precompute.worldAxes[1];

	// Linear Body1
	ptr_m_J_00[Row0Idx] =  0.0f;
	ptr_m_J_01[Row0Idx] =  0.0f;
	ptr_m_J_02[Row0Idx] =  0.0f;

	// -CrossProdMatrix(b1rot * b1anchor_ls)
	ptr_m_J_03[Row0Idx] =  worldAxis0.x;
	ptr_m_J_04[Row0Idx] =  worldAxis0.y;
	ptr_m_J_05[Row0Idx] =  worldAxis0.z;

	if (m_body1L != nullptr)
	{
		// Linear Body2
		ptr_m_J_06[Row0Idx] =  0.0f;
		ptr_m_J_07[Row0Idx] =  0.0f;
		ptr_m_J_08[Row0Idx] =  0.0f;

		// +CrossProdMatrix(b2rot * b2anchor_ls)
		ptr_m_J_09[Row0Idx] = -worldAxis0.x;
		ptr_m_J_10[Row0Idx] = -worldAxis0.y;
		ptr_m_J_11[Row0Idx] = -worldAxis0.z;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_06[Row0Idx] = 0.0f;
		ptr_m_J_07[Row0Idx] = 0.0f;
		ptr_m_J_08[Row0Idx] = 0.0f;

		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;

	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	const float corrCoeff = 1.0f;
	if (m_activeLimitState == LimitState::eLoViolation)
	{
		float violation = m_sinHalfLo - limitSinHalfAngle;
		if (violation < m_sinHalfTolerance)
		{
			*(ptr_m_J_rhs++) = 0.0f;
		}
		else
		{
			*(ptr_m_J_rhs++) = idt * m_ERP * corrCoeff * (violation - m_sinHalfTolerance);
		}
	}
	else
	{
		float violation = m_sinHalfHi - limitSinHalfAngle;
		if (violation > -m_sinHalfTolerance)
		{
			*(ptr_m_J_rhs++) = 0.0f;
		}
		else
		{
			*(ptr_m_J_rhs++) = idt * m_ERP * corrCoeff * (violation + m_sinHalfTolerance);
		}
	}

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	if (m_activeLimitState == LimitState::eLoViolation)
	{
		*(ptr_m_Lo++) =  0.0f;
		*(ptr_m_Hi++) =  FLT_MAX;
	}
	else
	{
		*(ptr_m_Lo++) = -FLT_MAX;
		*(ptr_m_Hi++) =  0.0f;
	}

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;


	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_maxNumRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff*m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void AxisRotationLimit::updateCopyVel(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 0.3f);
}
void AxisRotationLimit::updateCopyVelFriction(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 1.0f);
}

void AxisRotationLimit::fetchLambdasVel(SolverBase &solver)
{
	if (m_activeLimitState == LimitState::eFree)
		m_lambda0[0] = 0.0f;
	else
		m_lambda0[0] = solver.m_lambda[m_startIdx];
}
void AxisRotationLimit::fetchLambdasVelFriction(SolverBase &solver)
{
	if (m_activeLimitState == LimitState::eFree)
		m_lambda0[0] = 0.0f;
	else
		m_lambda0[0] = solver.m_lambda[m_startIdx];
}

//////////////////////////////////////////////////////////////////////////
// AxisLinearLimit
//////////////////////////////////////////////////////////////////////////
void AxisLinearLimit::init(float ERP, float CFM, const math::Vec3 & axis, float distLo, float distHi, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	memset(m_lambda0, 0, c_maxNumRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	m_localAxes[0] = m_body0R->m_rot.getConjugated().rotate(axis);
	if (m_body1R)
	{
		m_localAxes[1] = m_body1R->m_rot.getConjugated().rotate(axis);
	}
	else
	{
		m_localAxes[1] = axis;
	}
	if (m_body1L)
	{
		m_worldPos = math::Vec3C(0.0f, 0.0f, 0.0f);
	}
	else
	{
		m_worldPos = m_body0L->m_pos;
	}

	m_distLo = distLo;
	m_distHi = distHi;

	setToleranceDistance(0.05f);

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}

void AxisLinearLimit::onStepStart(float dt)
{
	using namespace math;

	m_precompute.worldPos[0] = m_body0L->m_pos;
	m_precompute.worldAxes[0] = m_body0R->m_rot.rotate(m_localAxes[0]);
	if (m_body1L)
		m_precompute.worldPos[1] = m_body1L->m_pos;
	else
		m_precompute.worldPos[1] = m_worldPos;

	if (m_body1R)
	{
		m_precompute.worldAxes[1] = m_body1R->m_rot.rotate(m_localAxes[1]);
	}
	else
	{
		m_precompute.worldAxes[1] = m_localAxes[1];
	}
}

void AxisLinearLimit::calcLimitDist(float * limitDist) const
{
	*limitDist = m_precompute.worldAxes[0].dot(m_precompute.worldPos[0] - m_precompute.worldPos[1]);
}

AxisLinearLimit::LimitState AxisLinearLimit::testLimit(float limitSinHalfAngle) const
{
	if (limitSinHalfAngle < m_distLo)
		return LimitState::eLoViolation;
	else if (limitSinHalfAngle > m_distHi)
		return LimitState::eHiViolation;

	return LimitState::eFree;
}

uint32_t AxisLinearLimit::getNumRowsVel() const
{
	float limitDist;
	calcLimitDist(&limitDist);

	LimitState limitState = testLimit(limitDist);
	if (limitState == LimitState::eFree)
		return 0;
	else
		return c_maxNumRows;
}
uint32_t AxisLinearLimit::getNumRowsVelFriction() const
{
	float limitDist;
	calcLimitDist(&limitDist);

	LimitState limitState = testLimit(limitDist);
	if (limitState == LimitState::eFree)
		return 0;
	else
		return c_maxNumRows;
}

void AxisLinearLimit::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	using namespace math;

	float limitDist;
	calcLimitDist(&limitDist);

	m_activeLimitState = testLimit(limitDist);
	if (m_activeLimitState == LimitState::eFree)
		return;

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_maxNumRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_maxNumRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx;

	Row0Idx = m_startIdx;

	Vec3 & worldAxis0 = m_precompute.worldAxes[0];
	Vec3 & worldAxis1 = m_precompute.worldAxes[1];

	// Linear Body1
	ptr_m_J_00[Row0Idx] =  worldAxis0.x;
	ptr_m_J_01[Row0Idx] =  worldAxis0.y;
	ptr_m_J_02[Row0Idx] =  worldAxis0.z;

	ptr_m_J_03[Row0Idx] =  0.0f;
	ptr_m_J_04[Row0Idx] =  0.0f;
	ptr_m_J_05[Row0Idx] =  0.0f;

	if (m_body1L != nullptr)
	{
		// Linear Body2
		ptr_m_J_06[Row0Idx] = -worldAxis1.x;
		ptr_m_J_07[Row0Idx] = -worldAxis1.y;
		ptr_m_J_08[Row0Idx] = -worldAxis1.z;

		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_06[Row0Idx] = 0.0f;
		ptr_m_J_07[Row0Idx] = 0.0f;
		ptr_m_J_08[Row0Idx] = 0.0f;

		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;

	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	const float corrCoeff = 1.0f;
	if (m_activeLimitState == LimitState::eLoViolation)
	{
		float violation = m_distLo - limitDist;
		if (violation < m_distTolerance)
		{
			*(ptr_m_J_rhs++) = 0.0f;
		}
		else
		{
			*(ptr_m_J_rhs++) = idt * m_ERP * corrCoeff * (violation - m_distTolerance);
		}
	}
	else
	{
		float violation = m_distHi - limitDist;
		if (violation > -m_distTolerance)
		{
			*(ptr_m_J_rhs++) = 0.0f;
		}
		else
		{
			*(ptr_m_J_rhs++) = idt * m_ERP * corrCoeff * (violation + m_distTolerance);
		}
	}

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	if (m_activeLimitState == LimitState::eLoViolation)
	{
		*(ptr_m_Lo++) =  0.0f;
		*(ptr_m_Hi++) =  FLT_MAX;
	}
	else
	{
		*(ptr_m_Lo++) = -FLT_MAX;
		*(ptr_m_Hi++) =  0.0f;
	}

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
		*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;


	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_maxNumRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff*m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void AxisLinearLimit::updateCopyVel(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 0.3f);
}
void AxisLinearLimit::updateCopyVelFriction(float dt, SolverBase &solver)
{
	updateCopy(dt, solver, 1.0f);
}

void AxisLinearLimit::fetchLambdasVel(SolverBase &solver)
{
	if (m_activeLimitState == LimitState::eFree)
		m_lambda0[0] = 0.0f;
	else
		m_lambda0[0] = solver.m_lambda[m_startIdx];
}
void AxisLinearLimit::fetchLambdasVelFriction(SolverBase &solver)
{
	if (m_activeLimitState == LimitState::eFree)
		m_lambda0[0] = 0.0f;
	else
		m_lambda0[0] = solver.m_lambda[m_startIdx];
}

//////////////////////////////////////////////////////////////////////////
// MotorRotation
//////////////////////////////////////////////////////////////////////////
void MotorRotation::init(float CFM, const math::Vec3 & axis, float vel, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_CFM[0] = CFM;

	// Calculating Anchor Points in Local Space (vs user-defined origin, not CoM)
	m_localAxes[0] = m_body0R->m_rot.getConjugated().rotate(axis);

	m_desiredVelocity = vel;
	m_forceMin = -FLT_MAX;
	m_forceMax =  FLT_MAX;

	if (m_body1L != nullptr)
	{
		m_localAxes[1] = m_body1R->m_rot.getConjugated().rotate(axis);
	}
	else
	{
		m_localAxes[1] = axis;
	}

	// No local mass scaling by default
	m_invMassScales[0] = 1.0f;
	m_invMassScales[1] = 1.0f;
}
uint32_t MotorRotation::getNumRowsVel() const
{
	return c_numRows;
}
uint32_t MotorRotation::getNumRowsVelFriction() const
{
	return c_numRows;
}

void MotorRotation::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];
	for (int i = 0; i < c_numRows; ++i)
	{
		*(ptr_m_CFM++) = m_CFM[i];
	}

	// Fill Jacobi matrices
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	// Additional tripples
	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx;

	Row0Idx = m_startIdx;

	// Linear Body0
	ptr_m_J_00[Row0Idx] =  0.0f;
	ptr_m_J_01[Row0Idx] =  0.0f;
	ptr_m_J_02[Row0Idx] =  0.0f;

	// Local anchor point in body1 wrt CoM
	math::Vec3 worldAxes[2];

	worldAxes[0] = m_body0R->m_rot.rotate(m_localAxes[0]);
	ptr_m_J_03[Row0Idx] = -worldAxes[0].x;
	ptr_m_J_04[Row0Idx] = -worldAxes[0].y;
	ptr_m_J_05[Row0Idx] = -worldAxes[0].z; 

	// Linear Body1
	ptr_m_J_06[Row0Idx] = 0.0f;
	ptr_m_J_07[Row0Idx] = 0.0f;
	ptr_m_J_08[Row0Idx] = 0.0f;

	if (m_body1L != nullptr)
	{
		// Local anchor point in body2 wrt CoM
		worldAxes[1] = m_body1R->m_rot.rotate(m_localAxes[1]);

		// +CrossProdMatrix(b2rot * b2anchor_ls)
		ptr_m_J_09[Row0Idx] =  worldAxes[0].x;
		ptr_m_J_10[Row0Idx] =  worldAxes[0].y;
		ptr_m_J_11[Row0Idx] =  worldAxes[0].z;
	}
	else
	{
		// Null out Jacobians (if attached to world)
		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
	}

	// Null remaining Jacobian
	ptr_m_J_12[Row0Idx] = 0.0f;
	ptr_m_J_13[Row0Idx] = 0.0f;
	ptr_m_J_14[Row0Idx] = 0.0f;
	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	// Baumgarte Stabilization
	*(ptr_m_J_rhs++) = idt * m_desiredVelocity;

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	*(ptr_m_Lo++) = m_forceMin;
	*(ptr_m_Hi++) = m_forceMax;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx];

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = m_body0L->m_idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = m_body0R->m_idx;

	// Third Triple Index
	if (m_body1L != nullptr)
	{
		*(ptr_m_JtdNodes_02++) = m_body1L->m_idx;
	}
	else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_02++) = SOLVER_NO_BODY;
	}

	// Fourth Triple Index
	if (m_body1R != nullptr)
	{
		*(ptr_m_JtdNodes_03++) = m_body1R->m_idx;
	} else
	{
		// No triple -- "world" anchor
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
	}

	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;

	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[Row0Idx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[Row0Idx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[Row0Idx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[Row0Idx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[Row0Idx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[Row0Idx];

	for (unsigned int i = 0; i < c_numRows; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff * m_lambda0[i];

		*(ptr_m_invMassScale_00++) = m_invMassScales[0];
		*(ptr_m_invMassScale_01++) = m_invMassScales[0];
		*(ptr_m_invMassScale_02++) = m_invMassScales[1];
		*(ptr_m_invMassScale_03++) = m_invMassScales[1];
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void MotorRotation::updateCopyVel(float dt, SolverBase & solver)
{
	updateCopy(dt, solver, 0.5f);
}
void MotorRotation::updateCopyVelFriction(float dt, SolverBase & solver)
{
	updateCopy(dt, solver, 1.0f);
}

void MotorRotation::fetchLambdasVel(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx];
}
void MotorRotation::fetchLambdasVelFriction(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx];
}


//////////////////////////////////////////////////////////////////////////
// Contact
//////////////////////////////////////////////////////////////////////////
void Contact::init(float ERP, float CFM, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R)
{
	m_cache = nullptr;

	memset(m_lambda0, 0, c_numRowsMax * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_body1L = body1L;
	m_body1R = body1R;

	m_skinWidth = m_min(m_body0L->m_skinWidth, m_body1L->m_skinWidth);
	m_restitutionCoeff = 0.5f * (m_body0L->m_restitutionCoeff + m_body1L->m_restitutionCoeff);
	m_frictionCoeff = sqrtf(m_body0L->m_frictionCoeff * m_body1L->m_frictionCoeff);

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP[0] = ERP;
	m_ERP[1] = 0.1f * ERP;
	m_ERP[2] = 0.1f * ERP;

	// Local mass scaling is disabled by default
	m_localMassScalingParams.isEnabled = false;
}

void Contact::setContactInfo(const math::Vec3 & cp0, const math::Vec3 & cp1, const math::Vec3 & cn)
{
	m_contactPoint0W = cp0;
	m_contactPoint1W = cp1;
	m_contactNormal = cn;

	m_cp0_b0o = physics::transformWorldToOrigin(m_body0L, m_body0R, m_contactPoint0W);
	m_cp1_b0o = physics::transformWorldToOrigin(m_body0L, m_body0R, m_contactPoint1W);

	if (m_body1L)
	{
		m_cp0_b1o = physics::transformWorldToOrigin(m_body1L, m_body1R, m_contactPoint0W);
		m_cp1_b1o = physics::transformWorldToOrigin(m_body1L, m_body1R, m_contactPoint1W);
	}
	else
	{
		m_cp0_b1o = m_contactPoint0W;
		m_cp1_b1o = m_contactPoint1W;
	}
}

void Contact::resetLambdas()
{
	m_lambda0[0] = 0.0f;
	m_lambda0[1] = 0.0f;
	m_lambda0[2] = 0.0f;
}

uint32_t Contact::getNumRowsVel() const
{
	return c_numRowsNormal;
}
uint32_t Contact::getNumRowsVelFriction() const
{
	return c_numRowsFriction;
}

void Contact::updateCopyVelInternal(float dt, SolverBase & solver, float initialGuessMul)
{
	using namespace math;

	Vec3 com0 = m_body0L->m_pos;
	Vec3 relCP0(m_contactPoint0W - com0), perp0(relCP0.cross(m_contactNormal));

	Vec3 com1 = Vec3C();
	Vec3 relCP1 = Vec3C(), perp1 = Vec3C();

	Vec3 vRel = m_body0L->m_vel;
	if (m_body0R)
		m_body0R->m_vel.cross(relCP0);

	m_bActive = true;

	//////////////////////////////////////////////////////////////////////////
	// Standard Jacobi-fill and stuff
	//////////////////////////////////////////////////////////////////////////

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRowsNormal;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	solver.m_CFM[m_startIdx] = m_CFM[0];

	// Pointer to Jacobi row
	float	*ptr_m_J_00 = &solver.m_J_00[m_startIdx], *ptr_m_J_01 = &solver.m_J_01[m_startIdx], *ptr_m_J_02 = &solver.m_J_02[m_startIdx],
			*ptr_m_J_03 = &solver.m_J_03[m_startIdx], *ptr_m_J_04 = &solver.m_J_04[m_startIdx], *ptr_m_J_05 = &solver.m_J_05[m_startIdx],
			*ptr_m_J_06 = &solver.m_J_06[m_startIdx], *ptr_m_J_07 = &solver.m_J_07[m_startIdx], *ptr_m_J_08 = &solver.m_J_08[m_startIdx],
			*ptr_m_J_09 = &solver.m_J_09[m_startIdx], *ptr_m_J_10 = &solver.m_J_10[m_startIdx], *ptr_m_J_11 = &solver.m_J_11[m_startIdx];

	float	*ptr_m_J_12 = &solver.m_J_12[m_startIdx], *ptr_m_J_13 = &solver.m_J_13[m_startIdx], *ptr_m_J_14 = &solver.m_J_14[m_startIdx],
			*ptr_m_J_15 = &solver.m_J_15[m_startIdx], *ptr_m_J_16 = &solver.m_J_16[m_startIdx], *ptr_m_J_17 = &solver.m_J_17[m_startIdx];


	// Fill Jacobi matrix
	*ptr_m_J_00++ = -m_contactNormal.x;
	*ptr_m_J_01++ = -m_contactNormal.y;
	*ptr_m_J_02++ = -m_contactNormal.z;
	*ptr_m_J_03++ = -perp0.x;
	*ptr_m_J_04++ = -perp0.y;
	*ptr_m_J_05++ = -perp0.z;

	if (m_body1L)
	{
		com1 = m_body1L->m_pos;
		relCP1 = m_contactPoint1W - com1;
		perp1 = relCP1.cross(m_contactNormal);

		vRel -= m_body1L->m_vel;
		if (m_body1R)
			vRel -= m_body1R->m_vel.cross(relCP1);

		*ptr_m_J_06++ = m_contactNormal.x;
		*ptr_m_J_07++ = m_contactNormal.y;
		*ptr_m_J_08++ = m_contactNormal.z;
		*ptr_m_J_09++ = perp1.x;
		*ptr_m_J_10++ = perp1.y;
		*ptr_m_J_11++ = perp1.z;
	}
	else
	{
		*ptr_m_J_06++ = 0.0f;
		*ptr_m_J_07++ = 0.0f;
		*ptr_m_J_08++ = 0.0f;
		*ptr_m_J_09++ = 0.0f;
		*ptr_m_J_10++ = 0.0f;
		*ptr_m_J_11++ = 0.0f;
	}

	for (int i = 0; i < 1; ++i)
	{
		// Null remaining Jacobian
		*ptr_m_J_12++ = 0.0f;
		*ptr_m_J_13++ = 0.0f;
		*ptr_m_J_14++ = 0.0f;
		*ptr_m_J_15++ = 0.0f;
		*ptr_m_J_16++ = 0.0f;
		*ptr_m_J_17++ = 0.0f;
	}

	scalar expectedVel = m_restitutionCoeff * vRel.dot(m_contactNormal);

	// Baumgarte Stabilization
	float depth = (m_contactPoint0W - m_contactPoint1W).dot(m_contactNormal);
	scalar contactResponseVel;
	if (depth < m_skinWidth)
		contactResponseVel = 0.0f;
	else
		contactResponseVel = idt * m_ERP[0] * (depth - m_skinWidth);

	// In case penetration resolve velocity is small and the object is set up to be bouncy,
	//	set desired velocity to the expected bounce velocity
	if (contactResponseVel < expectedVel)
		contactResponseVel = expectedVel;

	solver.m_J_rhs[m_startIdx] = contactResponseVel;

	// Limits
	solver.m_Lo[m_startIdx] = 0.0f;
	solver.m_Hi[m_startIdx] = FLT_MAX;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[m_startIdx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[m_startIdx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[m_startIdx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[m_startIdx];

	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[m_startIdx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[m_startIdx];

	for (int i = 0; i < c_numRowsNormal; ++i)
	{
		*ptr_m_JtdNodes_00++ = m_body0L->m_idx;
		*ptr_m_JtdNodes_01++ = m_body0R ? m_body0R->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_02++ = m_body1L ? m_body1L->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_03++ = m_body1R ? m_body1R->m_idx : SOLVER_NO_BODY;

		// Null out remaining indices
		*ptr_m_JtdNodes_04++ = SOLVER_NO_BODY;
		*ptr_m_JtdNodes_05++ = SOLVER_NO_BODY;
	}

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	// Lambda Initial Guess
	solver.m_lambda[m_startIdx] = initialGuessMul * m_lambda0[0];

	float b0invMassScale = 1.0f;
	float b1invMassScale = 1.0f;
	if (m_localMassScalingParams.isEnabled)
	{
		if (m_body1L)
		{
			// Maximum mass adjustment for one node
			const float massScaleMax = m_localMassScalingParams.maxMassScale;
			// Angle at wich mass scale is not applied anymore (e.g. if bodies are aligned horizontally, no mass scaling should be applied)
			const float cosAngMin = 0.5f;
			// Angle at which mass scale is maximum
			const float cosAngMax = 0.75f;

			Vec3 delta = m_body1L->m_pos - m_body0L->m_pos;
			float deltaDotGrav = delta.dot(m_localMassScalingParams.m_gravityDirection);
			// If body1 is below body0
			if (deltaDotGrav > cosAngMin)
			{
				// Body0 is considered "lighter" in this constraint
				float interp = clamp((deltaDotGrav - cosAngMin) / (cosAngMax - cosAngMin), 0.0f, 1.0f);
				float massScale = 1.0f * (1.0f - interp) + massScaleMax * interp;
				b0invMassScale = massScale;
				b1invMassScale = 1.0f / massScale;
			}
			else if (deltaDotGrav < -cosAngMin)
			{
				// Body1 is considered "heavier" in this constraint
				float interp = clamp(((-deltaDotGrav) - cosAngMin) / (cosAngMax - cosAngMin), 0.0f, 1.0f);
				float massScale = 1.0f * (1.0f - interp) + massScaleMax * interp;
				b0invMassScale = 1.0f / massScale;
				b1invMassScale = massScale;
			}
		}
	}
	*(ptr_m_invMassScale_00++) = b0invMassScale;
	*(ptr_m_invMassScale_01++) = b0invMassScale;
	*(ptr_m_invMassScale_02++) = b1invMassScale;
	*(ptr_m_invMassScale_03++) = b1invMassScale;
	*(ptr_m_invMassScale_04++) = 1.0f;
	*(ptr_m_invMassScale_05++) = 1.0f;

	m_precalcVRel = vRel;
}
void Contact::updateCopyVel(float dt, SolverBase & solver)
{
	// We want to avoid overshooting on the first velocity solve, hence
	//	we only accept 50% of the initial guess that is coming from the
	//	previous frame
	const float initialGuessMul = 0.5f;
	updateCopyVelInternal(dt, solver, initialGuessMul);
}
void Contact::updateCopyVelFriction(float dt, SolverBase & solver)
{
	using namespace math;

	// VelocityFriction update follows immediately after velocity update, which is
	//	estimating the normal force for friction limits calculation => we need
	//	to accept 100% of the initial guess vector, because it is basically a
	//	distributed solve
	const float initialGuessMulVel = 1.0f;
	updateCopyVelInternal(dt, solver, initialGuessMulVel);

	//////////////////////////////////////////////////////////////////////////
	// Standard Jacobi-fill and stuff
	//////////////////////////////////////////////////////////////////////////

	uint32_t numRowsFrictionOnly = c_numRowsFriction - c_numRowsNormal;

	// Receive Number for this joint && Update Solver information about number of joints
	// m_startIdx was obtained in the updateCopyVel
	uint32_t frictionStartIdx = solver.m_numJoints;
	solver.m_numJoints += numRowsFrictionOnly;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	solver.m_CFM[frictionStartIdx  ] = m_CFM[1];
	solver.m_CFM[frictionStartIdx+1] = m_CFM[2];

	// Pointer to Jacobi row
	float	*ptr_m_J_00 = &solver.m_J_00[frictionStartIdx], *ptr_m_J_01 = &solver.m_J_01[frictionStartIdx], *ptr_m_J_02 = &solver.m_J_02[frictionStartIdx],
			*ptr_m_J_03 = &solver.m_J_03[frictionStartIdx], *ptr_m_J_04 = &solver.m_J_04[frictionStartIdx], *ptr_m_J_05 = &solver.m_J_05[frictionStartIdx],
			*ptr_m_J_06 = &solver.m_J_06[frictionStartIdx], *ptr_m_J_07 = &solver.m_J_07[frictionStartIdx], *ptr_m_J_08 = &solver.m_J_08[frictionStartIdx],
			*ptr_m_J_09 = &solver.m_J_09[frictionStartIdx], *ptr_m_J_10 = &solver.m_J_10[frictionStartIdx], *ptr_m_J_11 = &solver.m_J_11[frictionStartIdx];

	float	*ptr_m_J_12 = &solver.m_J_12[frictionStartIdx], *ptr_m_J_13 = &solver.m_J_13[frictionStartIdx], *ptr_m_J_14 = &solver.m_J_14[frictionStartIdx],
			*ptr_m_J_15 = &solver.m_J_15[frictionStartIdx], *ptr_m_J_16 = &solver.m_J_16[frictionStartIdx], *ptr_m_J_17 = &solver.m_J_17[frictionStartIdx];

	math::Vec3 tan0, tan1;
	bool tangentsValid = false;
	const float tanSufficientVRelSq = 1.0f;
	if (m_precalcVRel.sqLen() > tanSufficientVRelSq)
	{
		tan0 = m_precalcVRel.cross(m_contactNormal);
		if (tan0.sqLen() > 1e-5f)
		{
			tan0.normalize();
			tan1 = m_contactNormal.cross(tan0);
			tan1.normalize();
			tangentsValid = true;
		}
	}
	if (!tangentsValid)
	{
		m_contactNormal.tangentSpace(tan0, tan1);
		tangentsValid = true;
	}

	Vec3 com0 = m_body0L->m_pos, com1;
	Vec3 relCP0(m_contactPoint0W - com0), relCP1;
	Vec3 fPerp00(relCP0.cross(tan0));
	Vec3 fPerp01(relCP0.cross(tan1));

	// Friction row 0, body 0
	*ptr_m_J_00++ = -tan0.x;
	*ptr_m_J_01++ = -tan0.y;
	*ptr_m_J_02++ = -tan0.z;
	*ptr_m_J_03++ = -fPerp00.x;
	*ptr_m_J_04++ = -fPerp00.y;
	*ptr_m_J_05++ = -fPerp00.z;

	// Friction row 1, body 0
	*ptr_m_J_00++ = -tan1.x;
	*ptr_m_J_01++ = -tan1.y;
	*ptr_m_J_02++ = -tan1.z;
	*ptr_m_J_03++ = -fPerp01.x;
	*ptr_m_J_04++ = -fPerp01.y;
	*ptr_m_J_05++ = -fPerp01.z;

	if (m_body1L)
	{
		com1 = m_body1L->m_pos;
		relCP1 = m_contactPoint1W - com1;
		Vec3 fPerp10(relCP1.cross(tan0));
		Vec3 fPerp11(relCP1.cross(tan1));

		// Friction row 0, body 1
		*ptr_m_J_06++ =  tan0.x;
		*ptr_m_J_07++ =  tan0.y;
		*ptr_m_J_08++ =  tan0.z;
		*ptr_m_J_09++ =  fPerp10.x;
		*ptr_m_J_10++ =  fPerp10.y;
		*ptr_m_J_11++ =  fPerp10.z;

		// Friction row 1, body 1
		*ptr_m_J_06++ =  tan1.x;
		*ptr_m_J_07++ =  tan1.y;
		*ptr_m_J_08++ =  tan1.z;
		*ptr_m_J_09++ =  fPerp11.x;
		*ptr_m_J_10++ =  fPerp11.y;
		*ptr_m_J_11++ =  fPerp11.z;
	}
	else
	{
		com1 = Vec3C();
		relCP1 = Vec3C();

		// tan0
		// Friction row 0, body 1
		*ptr_m_J_06++ = 0.0f;
		*ptr_m_J_07++ = 0.0f;
		*ptr_m_J_08++ = 0.0f;
		*ptr_m_J_09++ = 0.0f;
		*ptr_m_J_10++ = 0.0f;
		*ptr_m_J_11++ = 0.0f;

		// tan1
		// Friction row 1, body 1
		*ptr_m_J_06++ = 0.0f;
		*ptr_m_J_07++ = 0.0f;
		*ptr_m_J_08++ = 0.0f;
		*ptr_m_J_09++ = 0.0f;
		*ptr_m_J_10++ = 0.0f;
		*ptr_m_J_11++ = 0.0f;
	}

	for (uint32_t i = 0; i < numRowsFrictionOnly; ++i)
	{
		// Null remaining Jacobian
		*ptr_m_J_12++ = 0.0f;
		*ptr_m_J_13++ = 0.0f;
		*ptr_m_J_14++ = 0.0f;
		*ptr_m_J_15++ = 0.0f;
		*ptr_m_J_16++ = 0.0f;
		*ptr_m_J_17++ = 0.0f;
	}

	bool anchorFriction = false;
	Vec3 worldAnchor00, worldAnchor01;
	Vec3 worldAnchor10, worldAnchor11;
	Vec3 cpDelta;
	if (m_cache)
	{
		anchorFriction = true;
		worldAnchor00 = physics::transformOriginToWorld(m_body0L, m_body0R, m_cache->cp0_b0o);
		worldAnchor01 = physics::transformOriginToWorld(m_body0L, m_body0R, m_cache->cp1_b0o);
		if (m_body1L)
		{
			worldAnchor10 = physics::transformOriginToWorld(m_body1L, m_body1R, m_cache->cp0_b1o);
			worldAnchor11 = physics::transformOriginToWorld(m_body1L, m_body1R, m_cache->cp1_b1o);
		}
		else
		{
			worldAnchor10 = m_cache->cp0_b1o;
			worldAnchor11 = m_cache->cp1_b1o;
		}

		const float frictionBreakDistEps = 0.05f;
		const float frictionBreakVelEps = 0.1f;
		cpDelta = (worldAnchor00 - worldAnchor10);
		if (cpDelta.sqLen() > frictionBreakDistEps*frictionBreakDistEps || m_precalcVRel.sqLen() > frictionBreakVelEps*frictionBreakVelEps)
		{
			anchorFriction = false;
			getContactInfoOrigin(&m_cache->cp0_b0o, &m_cache->cp1_b0o, &m_cache->cp0_b1o, &m_cache->cp1_b1o);
		}
	}

	if (anchorFriction)
	{
		solver.m_J_rhs[frictionStartIdx  ] = idt * m_ERP[1] * tan0.dot(cpDelta);
		solver.m_J_rhs[frictionStartIdx+1] = idt * m_ERP[2] * tan1.dot(cpDelta);
	}
	else
	{
		solver.m_J_rhs[frictionStartIdx  ] = 0.0f;
		solver.m_J_rhs[frictionStartIdx+1] = 0.0f;
	}

	float friction_limit = m_frictionCoeff * m_lambda0[0];
	if (anchorFriction)
	{
		friction_limit *= 10.0f;
	}

	solver.m_Lo[frictionStartIdx  ] = -friction_limit;
	solver.m_Hi[frictionStartIdx  ] =  friction_limit;
	solver.m_Lo[frictionStartIdx+1] = -friction_limit;
	solver.m_Hi[frictionStartIdx+1] =  friction_limit;


	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[frictionStartIdx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[frictionStartIdx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[frictionStartIdx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[frictionStartIdx];

	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[frictionStartIdx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[frictionStartIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[frictionStartIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[frictionStartIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[frictionStartIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[frictionStartIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[frictionStartIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[frictionStartIdx];

	for (uint32_t i = 0; i < numRowsFrictionOnly; ++i)
	{
		*ptr_m_JtdNodes_00++ = m_body0L->m_idx;
		*ptr_m_JtdNodes_01++ = m_body0R ? m_body0R->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_02++ = m_body1L ? m_body1L->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_03++ = m_body1R ? m_body1R->m_idx : SOLVER_NO_BODY;

		// Null out remaining indices
		*ptr_m_JtdNodes_04++ = SOLVER_NO_BODY;
		*ptr_m_JtdNodes_05++ = SOLVER_NO_BODY;

		*(ptr_m_invMassScale_00++) = 1.0f;
		*(ptr_m_invMassScale_01++) = 1.0f;
		*(ptr_m_invMassScale_02++) = 1.0f;
		*(ptr_m_invMassScale_03++) = 1.0f;
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}

	// Lambda Initial Guess
	const float initialGuessMul = 1.0f;
	solver.m_lambda[frictionStartIdx  ] = initialGuessMul*m_lambda0[1];
	solver.m_lambda[frictionStartIdx+1] = initialGuessMul*m_lambda0[2];
}

void Contact::fetchLambdasVel(SolverBase & solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
}
void Contact::fetchLambdasVelFriction(SolverBase & solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
	m_lambda0[1] = solver.m_lambda[m_startIdx+1];
	m_lambda0[2] = solver.m_lambda[m_startIdx+2];
}

//////////////////////////////////////////////////////////////////////////
// PlaneConstraint
//////////////////////////////////////////////////////////////////////////

#if (PLANE_FRICTION == 1)

void PlaneConstraint::init(float ERP, float CFM, const math::Vec3 &PlanePoint, const math::Vec3 &PlaneNormal, NodeTranslational * bodyL, float Tolerance)
{
	m_Tolerance = Tolerance;

	m_bActive = false;

	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_bodyL = bodyL;

	m_CFM[0] = m_CFM[1] = m_CFM[2] = CFM;
	m_ERP[0] = m_ERP[1] = m_ERP[2] = ERP;

	m_PlaneNormal = PlaneNormal.getNormalized();
	m_PlaneD = PlanePoint.dot(m_PlaneNormal);
}

uint32_t PlaneConstraint::getNumRowsVel() const
{
	return c_numRows;
}
uint32_t PlaneConstraint::getNumRowsVelFriction() const
{
	return c_numRows;
}

void PlaneConstraint::updateCopy(float dt, SolverBase &solver)
{
	// Determine, if plane is in active
	float Violation = m_bodyL->m_pos.dot(m_PlaneNormal) - m_PlaneD;
	if (Violation > PLANE_THRESHOLD)
	{
		m_bActive = false;

		m_lambda0[0] = 0.0f;
		m_lambda0[1] = 0.0f;
		m_lambda0[2] = 0.0f;

		return;
	}

	m_bActive = true;

	//////////////////////////////////////////////////////////////////////////
	// Standard Jacobi-fill and stuff
	//////////////////////////////////////////////////////////////////////////

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	solver.m_CFM[m_startIdx  ] = m_CFM[0];
	solver.m_CFM[m_startIdx+1] = m_CFM[1];
	solver.m_CFM[m_startIdx+2] = m_CFM[2];

	// Pointer to Jacobi row
	float	*ptr_m_J_00 = &solver.m_J_00[m_startIdx], *ptr_m_J_01 = &solver.m_J_01[m_startIdx], *ptr_m_J_02 = &solver.m_J_02[m_startIdx],
			*ptr_m_J_03 = &solver.m_J_03[m_startIdx], *ptr_m_J_04 = &solver.m_J_04[m_startIdx], *ptr_m_J_05 = &solver.m_J_05[m_startIdx],
			*ptr_m_J_06 = &solver.m_J_06[m_startIdx], *ptr_m_J_07 = &solver.m_J_07[m_startIdx], *ptr_m_J_08 = &solver.m_J_08[m_startIdx],
			*ptr_m_J_09 = &solver.m_J_09[m_startIdx], *ptr_m_J_10 = &solver.m_J_10[m_startIdx], *ptr_m_J_11 = &solver.m_J_11[m_startIdx];

	float	*ptr_m_J_12 = &solver.m_J_12[m_startIdx], *ptr_m_J_13 = &solver.m_J_13[m_startIdx], *ptr_m_J_14 = &solver.m_J_14[m_startIdx],
			*ptr_m_J_15 = &solver.m_J_15[m_startIdx], *ptr_m_J_16 = &solver.m_J_16[m_startIdx], *ptr_m_J_17 = &solver.m_J_17[m_startIdx];


	// Fill Jacobi matrix
	*ptr_m_J_00++ = m_PlaneNormal.x;
	*ptr_m_J_01++ = m_PlaneNormal.y;
	*ptr_m_J_02++ = m_PlaneNormal.z;

	math::Vec3 Tan1, Tan2;
	m_PlaneNormal.tangentSpace(Tan1, Tan2);

	*ptr_m_J_00++ = Tan1.x;
	*ptr_m_J_01++ = Tan1.y;
	*ptr_m_J_02++ = Tan1.z;

	*ptr_m_J_00++ = Tan2.x;
	*ptr_m_J_01++ = Tan2.y;
	*ptr_m_J_02++ = Tan2.z;

	for (int i = 0; i < c_numRows; ++i)
	{
		*ptr_m_J_03++ = 0.0f;
		*ptr_m_J_04++ = 0.0f;
		*ptr_m_J_05++ = 0.0f;
		*ptr_m_J_06++ = 0.0f;
		*ptr_m_J_07++ = 0.0f;
		*ptr_m_J_08++ = 0.0f;
		*ptr_m_J_09++ = 0.0f;
		*ptr_m_J_10++ = 0.0f;
		*ptr_m_J_11++ = 0.0f;

		// Null remaining Jacobian
		*ptr_m_J_12++ = 0.0f;
		*ptr_m_J_13++ = 0.0f;
		*ptr_m_J_14++ = 0.0f;
		*ptr_m_J_15++ = 0.0f;
		*ptr_m_J_16++ = 0.0f;
		*ptr_m_J_17++ = 0.0f;
	}

	// Allow some penetration for stability
	Violation += m_Tolerance;

	if (Violation > 0.0f)
		Violation = 0.0f;

	// Baumgarte Stabilization
	solver.m_J_rhs[m_startIdx  ] = (Violation < 0.0f) ? (-idt * m_ERP[0] * Violation) : 0.0f;
	solver.m_J_rhs[m_startIdx+1] = 0.0f;
	solver.m_J_rhs[m_startIdx+2] = 0.0f;

	// Limits
	solver.m_Lo[m_startIdx  ] = 0.0f;
	solver.m_Hi[m_startIdx  ] = FLT_MAX;

	float friction_limit = 10.1f;

	solver.m_Lo[m_startIdx+1] = -friction_limit;
	solver.m_Hi[m_startIdx+1] =  friction_limit;
	solver.m_Lo[m_startIdx+2] = -friction_limit;
	solver.m_Hi[m_startIdx+2] =  friction_limit;

// 	solver.m_Lo[m_startIdx+1] = -FLT_MAX;
// 	solver.m_Hi[m_startIdx+1] =  FLT_MAX;
// 	solver.m_Lo[m_startIdx+2] = -FLT_MAX;
// 	solver.m_Hi[m_startIdx+2] =  FLT_MAX;


	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[m_startIdx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[m_startIdx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[m_startIdx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[m_startIdx];

	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[m_startIdx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
	{
		*ptr_m_JtdNodes_00++ = m_bodyL->m_idx;

		*ptr_m_JtdNodes_01++ = SOLVER_NO_BODY;
		*ptr_m_JtdNodes_02++ = SOLVER_NO_BODY;
		*ptr_m_JtdNodes_03++ = SOLVER_NO_BODY;

		// Null out remaining indices
		*ptr_m_JtdNodes_04++ = SOLVER_NO_BODY;
		*ptr_m_JtdNodes_05++ = SOLVER_NO_BODY;
	}


	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	// Lambda Initial Guess
	for (uint32_t i = 0; i < c_numRows; ++i)
	{
		solver.m_lambda[m_startIdx+i] = m_lambda0[i];

		*(ptr_m_invMassScale_00++) = 1.0f;
		*(ptr_m_invMassScale_01++) = 1.0f;
		*(ptr_m_invMassScale_02++) = 1.0f;
		*(ptr_m_invMassScale_03++) = 1.0f;
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void PlaneConstraint::updateCopyVel(float dt, SolverBase &solver)
{
	updateCopy(dt, solver);
}
void PlaneConstraint::updateCopyVelFriction(float dt, SolverBase &solver)
{
	updateCopy(dt, solver);
}

void PlaneConstraint::fetchLambdasVel(SolverBase &solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
	m_lambda0[1] = solver.m_lambda[m_startIdx+1];
	m_lambda0[2] = solver.m_lambda[m_startIdx+2];
}
void PlaneConstraint::fetchLambdasVelFriction(SolverBase &solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
	m_lambda0[1] = solver.m_lambda[m_startIdx+1];
	m_lambda0[2] = solver.m_lambda[m_startIdx+2];
}

//////////////////////////////////////////////////////////////////////////
#else
//////////////////////////////////////////////////////////////////////////

void PlaneConstraint::init(float ERP, float CFM, const math::Vec3 &PlanePoint, const math::Vec3 &PlaneNormal, NodeTranslational * bodyL, float Tolerance)
{
	m_Tolerance = Tolerance;

	m_bActive = false;

	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_bodyL = bodyL;

	m_CFM[0] = CFM;
	m_ERP[0] = ERP;

	m_PlaneNormal = PlaneNormal.getNormalized();
	m_PlaneD = PlanePoint.dot(m_PlaneNormal);
}

uint32_t PlaneConstraint::getNumRows() const
{
	return c_numRows;
}

void PlaneConstraint::updateCopy(float dt, SolverBase &solver)
{
	// Determine, if plane is in active
	float Violation = m_bodyL->m_pos.dot(m_PlaneNormal) - m_PlaneD;
	if (Violation > PLANE_THRESHOLD)
	{
		m_bActive = false;

		m_lambda0[0] = 0.0f;

		return;
	}

	m_bActive = true;

	//////////////////////////////////////////////////////////////////////////
	// Standard Jacobi-fill and stuff
	//////////////////////////////////////////////////////////////////////////

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += 1;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	solver.m_CFM[m_startIdx  ] = m_CFM[0];
	solver.m_CFM[m_startIdx+1] = m_CFM[1];
	solver.m_CFM[m_startIdx+2] = m_CFM[2];

	// Pointer to Jacobi row
	float	*ptr_m_J_00 = &solver.m_J_00[m_startIdx], *ptr_m_J_01 = &solver.m_J_01[m_startIdx], *ptr_m_J_02 = &solver.m_J_02[m_startIdx],
			*ptr_m_J_03 = &solver.m_J_03[m_startIdx], *ptr_m_J_04 = &solver.m_J_04[m_startIdx], *ptr_m_J_05 = &solver.m_J_05[m_startIdx],
			*ptr_m_J_06 = &solver.m_J_06[m_startIdx], *ptr_m_J_07 = &solver.m_J_07[m_startIdx], *ptr_m_J_08 = &solver.m_J_08[m_startIdx],
			*ptr_m_J_09 = &solver.m_J_09[m_startIdx], *ptr_m_J_10 = &solver.m_J_10[m_startIdx], *ptr_m_J_11 = &solver.m_J_11[m_startIdx];

	float	*ptr_m_J_12 = &solver.m_J_12[m_startIdx], *ptr_m_J_13 = &solver.m_J_13[m_startIdx], *ptr_m_J_14 = &solver.m_J_14[m_startIdx],
			*ptr_m_J_15 = &solver.m_J_15[m_startIdx], *ptr_m_J_16 = &solver.m_J_16[m_startIdx], *ptr_m_J_17 = &solver.m_J_17[m_startIdx];


	// Fill Jacobi matrix
	*ptr_m_J_00++ = m_PlaneNormal.x;
	*ptr_m_J_01++ = m_PlaneNormal.y;
	*ptr_m_J_02++ = m_PlaneNormal.z;

	*ptr_m_J_03++ = 0.0f;
	*ptr_m_J_04++ = 0.0f;
	*ptr_m_J_05++ = 0.0f;
	*ptr_m_J_06++ = 0.0f;
	*ptr_m_J_07++ = 0.0f;
	*ptr_m_J_08++ = 0.0f;
	*ptr_m_J_09++ = 0.0f;
	*ptr_m_J_10++ = 0.0f;
	*ptr_m_J_11++ = 0.0f;

	// Null remaining Jacobian
	*ptr_m_J_12++ = 0.0f;
	*ptr_m_J_13++ = 0.0f;
	*ptr_m_J_14++ = 0.0f;
	*ptr_m_J_15++ = 0.0f;
	*ptr_m_J_16++ = 0.0f;
	*ptr_m_J_17++ = 0.0f;

	// Allow some penetration for stability
	Violation += m_Tolerance;

	if (Violation > 0.0f)
		Violation = 0.0f;

	// Baumgarte Stabilization
	solver.m_J_rhs[m_startIdx  ] = (Violation < 0.0f) ? (-idt * m_ERP[0] * Violation) : 0.0f;

	// Limits
	solver.m_Lo[m_startIdx  ] = 0.0f;
	solver.m_Hi[m_startIdx  ] = FLT_MAX;

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[m_startIdx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[m_startIdx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[m_startIdx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[m_startIdx];

	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[m_startIdx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[m_startIdx];

	*ptr_m_JtdNodes_00++ = m_bodyL->m_idx;

	*ptr_m_JtdNodes_01++ = SOLVER_NO_BODY;
	*ptr_m_JtdNodes_02++ = SOLVER_NO_BODY;
	*ptr_m_JtdNodes_03++ = SOLVER_NO_BODY;


	// Null out remaining indices
	*ptr_m_JtdNodes_04++ = SOLVER_NO_BODY;
	*ptr_m_JtdNodes_05++ = SOLVER_NO_BODY;

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	// Lambda Initial Guess
	solver.m_lambda[m_startIdx  ] = m_lambda0[0];

	*(ptr_m_invMassScale_00++) = 1.0f;
	*(ptr_m_invMassScale_01++) = 1.0f;
	*(ptr_m_invMassScale_02++) = 1.0f;
	*(ptr_m_invMassScale_03++) = 1.0f;
	*(ptr_m_invMassScale_04++) = 1.0f;
	*(ptr_m_invMassScale_05++) = 1.0f;
}

void PlaneConstraint::fetchLambdas(SolverBase &solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
}

#endif

}

#pragma once

#include <assert.h>
#include <vector>
#include "math/AuxMath.h"
#include "math/Vec3.h"

// Node 0 should be reserved for static body
// As alternative, make this index (uint)-1 and lift the reserved node requirement
#define SOLVER_NO_BODY	0

#include <float.h>
#include <math/Quat.h>

namespace physics
{

class Node
{
protected:

public:

	static const uint32_t c_idxInvalid = 0xFFffFFff;
	uint32_t m_idxPrev;
	uint32_t m_idx;

	math::Vec3 m_vel;
	math::Vec3 m_f;

	float m_dampingMul;	// Multiplicative damping, vel' = dampingMul * vel
	float m_dampingSub; // Subtractive damping, vel' = vel - dampingSub * (vel / |vel|)

	enum class Type
	{
		eTranslational,
		eRotational,

		eNUM_ENTRIES
	};
	virtual Type getType() const = 0;

	void initIndex()
	{
		m_idxPrev = (uint32_t)c_idxInvalid;
		m_idx = (uint32_t)c_idxInvalid;
	}

	bool updateIndex(uint32_t newIndex)
	{
		bool indexMatch = (newIndex == m_idx);
		m_idxPrev = m_idx;
		m_idx = newIndex;
		return indexMatch;
	}
};

class NodeTranslational : public Node
{
protected:
public:

	// General properties
	math::Vec3 m_pos;
	float m_invMass;

	// Contact properties
	float m_skinWidth;			// Skin width, amount of allowed interpenetration for this body, for stability (see Contact joint)
	float m_frictionCoeff;		// Dynamic friction coefficient, i.e. when bodies have significant relative movement
	float m_restitutionCoeff;	// Approximate resitution coefficient

	virtual Type getType() const { return Type::eTranslational; }
};

class NodeRotational : public Node
{
protected:
public:

	enum class Flags
	{
		eNoGyroscopic = 0b1,
	};
	Flags m_flags;

	math::Quat m_rot;
	math::Mat33 m_invInertia;
	// Used to store CoM of the whole rigid body once it gets rotational component,
	//	linear nodes do not need that when used without rotation, as they are point
	//	masses.
	// However this particular vector shouldn't be used very often, as it represents
	//	the shift from the origin where the rigid body was placed to the center of
	//	its linear node. This is useful to e.g. store the local coordinates vs origin
	//	and not CoM, but not much elsewhere.
	math::Vec3 m_com;

	virtual Type getType() const { return Type::eRotational; }
};

const float PLANE_THRESHOLD = 0.00001f;
const float GRAVITY_THRESHOLD = 0.000001f;

// Uncomment to enable cycle multiplication (also enables corresponding arrays)
// !!!!!!!!!!!!!!!!!!!!!!!!!!DO NOT USE WITH SOA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define JOINT_TRIPLES_CYCLE 0
// Number of maximum triples per joint (linear/rotational parts)
#define NUM_JOINT_TRIPLES 6

// Additional calculations for testing purposes (gradient/residual vector)
#define SOLVERS_ANALYZE_RESIDUAL 0
// Enable stopping criteria in solvers (w/o it iterations are fixed)
#define SOLVER_STOPPING_CRITERIA 1

// [ DEBUG ] Output objective function to log (CG/LCPCG only)
#define SOLVER_OBJECTIVE_FUNCTION_OUTPUT 0


// Switches Stiffness Warping basis calculation to Polar Decomposition
// (turned off - calculation based on FE edges, Mueller's approach)
#define WARPING_POLAR_DECOMPOSITION 0

#define inc_arr(arr) (*(ptr_##arr)++)
#define val_arr(arr) (*(ptr_##arr))



// Common macros for solvers

//////////////////////////////////////////////////////////////////////////
// INIT A SPARSE
//////////////////////////////////////////////////////////////////////////
#define INIT_A_SPARSE_PTRS(idx)\
		ptr_m_Asp_00 = &m_Asp_00[(idx)];\
		ptr_m_Asp_01 = &m_Asp_01[(idx)];\
		ptr_m_Asp_02 = &m_Asp_02[(idx)];\
		ptr_m_Asp_03 = &m_Asp_03[(idx)];\
		ptr_m_Asp_04 = &m_Asp_04[(idx)];\
		ptr_m_Asp_05 = &m_Asp_05[(idx)];\
		ptr_m_Asp_06 = &m_Asp_06[(idx)];\
		ptr_m_Asp_07 = &m_Asp_07[(idx)];\
		ptr_m_Asp_08 = &m_Asp_08[(idx)];\
		ptr_m_Asp_09 = &m_Asp_09[(idx)];\
		ptr_m_Asp_10 = &m_Asp_10[(idx)];\
		ptr_m_Asp_11 = &m_Asp_11[(idx)];\
		ptr_m_Asp_12 = &m_Asp_12[(idx)];\
		ptr_m_Asp_13 = &m_Asp_13[(idx)];\
		ptr_m_Asp_14 = &m_Asp_14[(idx)];\
		ptr_m_Asp_15 = &m_Asp_15[(idx)];\
		ptr_m_Asp_16 = &m_Asp_16[(idx)];\
		ptr_m_Asp_17 = &m_Asp_17[(idx)];

//////////////////////////////////////////////////////////////////////////
// INIT J SPARSE
//////////////////////////////////////////////////////////////////////////
#define INIT_J_SPARSE_PTRS(idx)\
	ptr_m_J_00 = &m_J_00[(idx)];\
	ptr_m_J_01 = &m_J_01[(idx)];\
	ptr_m_J_02 = &m_J_02[(idx)];\
	ptr_m_J_03 = &m_J_03[(idx)];\
	ptr_m_J_04 = &m_J_04[(idx)];\
	ptr_m_J_05 = &m_J_05[(idx)];\
	ptr_m_J_06 = &m_J_06[(idx)];\
	ptr_m_J_07 = &m_J_07[(idx)];\
	ptr_m_J_08 = &m_J_08[(idx)];\
	ptr_m_J_09 = &m_J_09[(idx)];\
	ptr_m_J_10 = &m_J_10[(idx)];\
	ptr_m_J_11 = &m_J_11[(idx)];\
	ptr_m_J_12 = &m_J_12[(idx)];\
	ptr_m_J_13 = &m_J_13[(idx)];\
	ptr_m_J_14 = &m_J_14[(idx)];\
	ptr_m_J_15 = &m_J_15[(idx)];\
	ptr_m_J_16 = &m_J_16[(idx)];\
	ptr_m_J_17 = &m_J_17[(idx)];

//////////////////////////////////////////////////////////////////////////
// INIT INV MASS SCALE POINTERS
//////////////////////////////////////////////////////////////////////////
#define INIT_INV_MASS_SCALE_PTRS(idx)\
	ptr_m_invMassScale_00 = &m_invMassScale_00[(idx)];\
	ptr_m_invMassScale_01 = &m_invMassScale_01[(idx)];\
	ptr_m_invMassScale_02 = &m_invMassScale_02[(idx)];\
	ptr_m_invMassScale_03 = &m_invMassScale_03[(idx)];\
	ptr_m_invMassScale_04 = &m_invMassScale_04[(idx)];\
	ptr_m_invMassScale_05 = &m_invMassScale_05[(idx)];

//////////////////////////////////////////////////////////////////////////
// INIT JOINTED NODES INDICES
//////////////////////////////////////////////////////////////////////////
#define INIT_NODES_IDX_PTRS(idx)\
	ptr_m_JtdNodes_00 = &m_JtdNodes_00[(idx)];\
	ptr_m_JtdNodes_01 = &m_JtdNodes_01[(idx)];\
	ptr_m_JtdNodes_02 = &m_JtdNodes_02[(idx)];\
	ptr_m_JtdNodes_03 = &m_JtdNodes_03[(idx)];\
	ptr_m_JtdNodes_04 = &m_JtdNodes_04[(idx)];\
	ptr_m_JtdNodes_05 = &m_JtdNodes_05[(idx)];

//////////////////////////////////////////////////////////////////////////
// CALCULATE VECTOR A
//////////////////////////////////////////////////////////////////////////
#define CALC_VECTOR_A(vec, j, t0, t1, t2)\
			idx = inc_arr( m_JtdNodes_0##j );\
			\
			ptr_m_a_x[idx] += inc_arr(m_Asp_##t0) * val_arr(vec);\
			ptr_m_a_y[idx] += inc_arr(m_Asp_##t1) * val_arr(vec);\
			ptr_m_a_z[idx] += inc_arr(m_Asp_##t2) * val_arr(vec);


//////////////////////////////////////////////////////////////////////////
// CALCULATE VECTOR A -- JACOBIAN PRODUCT
//////////////////////////////////////////////////////////////////////////
#define MUL_J_A_T0_T1() \
	val_arr(m_J_00) * ptr_m_a_x[t0Idx] + val_arr(m_J_01) * ptr_m_a_y[t0Idx] + val_arr(m_J_02) * ptr_m_a_z[t0Idx] + \
	val_arr(m_J_03) * ptr_m_a_x[t1Idx] + val_arr(m_J_04) * ptr_m_a_y[t1Idx] + val_arr(m_J_05) * ptr_m_a_z[t1Idx];

#define MUL_J_A_T2() \
	val_arr(m_J_06) * ptr_m_a_x[t2Idx] + val_arr(m_J_07) * ptr_m_a_y[t2Idx] + val_arr(m_J_08) * ptr_m_a_z[t2Idx];
#define MUL_J_A_T3() \
	val_arr(m_J_09) * ptr_m_a_x[t3Idx] + val_arr(m_J_10) * ptr_m_a_y[t3Idx] + val_arr(m_J_11) * ptr_m_a_z[t3Idx];
#define MUL_J_A_T4() \
	val_arr(m_J_12) * ptr_m_a_x[t4Idx] + val_arr(m_J_13) * ptr_m_a_y[t4Idx] + val_arr(m_J_14) * ptr_m_a_z[t4Idx];
#define MUL_J_A_T5() \
	val_arr(m_J_15) * ptr_m_a_x[t5Idx] + val_arr(m_J_16) * ptr_m_a_y[t5Idx] + val_arr(m_J_17) * ptr_m_a_z[t5Idx];

#define ADVANCE_J() \
	inc_arr(m_J_00); \
	inc_arr(m_J_01); \
	inc_arr(m_J_02); \
	inc_arr(m_J_03); \
	inc_arr(m_J_04); \
	inc_arr(m_J_05); \
	inc_arr(m_J_06); \
	inc_arr(m_J_07); \
	inc_arr(m_J_08); \
	inc_arr(m_J_09); \
	inc_arr(m_J_10); \
	inc_arr(m_J_11); \
	inc_arr(m_J_12); \
	inc_arr(m_J_13); \
	inc_arr(m_J_14); \
	inc_arr(m_J_15); \
	inc_arr(m_J_16); \
	inc_arr(m_J_17);

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////



class SolverBase
{
public:

	virtual void solve(float dt) = 0;

	float calcSylvesterCriterion();

	unsigned int getEffectiveIterations() const
	{
		return m_effectiveIterations;
	}

	void setMaxIterations(unsigned int MaxIterations)
	{
		m_maxIterations = MaxIterations;
	}
	unsigned int getMaxIterations() const
	{
		return m_maxIterations;
	}

	void setPrecision(float Precision)
	{
		m_precision = Precision;
	}
	float getPrecision(void) const
	{
		return m_precision;
	}

	void setInitialGuessCutFlag(bool Flag)
	{
		m_IG_CutFlag = Flag;
	}
	bool getInitialGuessCutFlag(void) const
	{
		return m_IG_CutFlag;
	}

	void setInitialGuessCutThreshold(float Threshold)
	{
		m_IG_CutThreshold = Threshold;
	}
	float getInitialGuessCutThreshold(void) const
	{
		return m_IG_CutThreshold;
	}

#if (SOLVERS_ANALYZE_RESIDUAL == 1)
	// Interface for Analysis
	float getLambdaNormSq() const
	{
		return m_LambdaNormSq;
	}

	float getGradNormSq() const
	{
		return m_GradNormSq;
	}

	float getDotLambdaGrad() const
	{
		return m_DotLambdaGrad;
	}
#else
	// Interface for Analysis [EMPTY]
	float getLambdaNormSq() const
	{
		return 0.0f;
	}

	float getGradNormSq() const
	{
		return 0.0f;
	}

	float getDotLambdaGrad() const
	{
		return 0.0f;
	}
#endif

	virtual void setNumAdditionalRows(uint32_t numRowsTotal) = 0;
	void setNumRows(uint32_t numRowsTotal)
	{
		m_J_00.resize(numRowsTotal);
		m_J_01.resize(numRowsTotal);
		m_J_02.resize(numRowsTotal);
		m_J_03.resize(numRowsTotal);
		m_J_04.resize(numRowsTotal);
		m_J_05.resize(numRowsTotal);
		m_J_06.resize(numRowsTotal);
		m_J_07.resize(numRowsTotal);
		m_J_08.resize(numRowsTotal);
		m_J_09.resize(numRowsTotal);
		m_J_10.resize(numRowsTotal);
		m_J_11.resize(numRowsTotal);
		m_J_12.resize(numRowsTotal);
		m_J_13.resize(numRowsTotal);
		m_J_14.resize(numRowsTotal);
		m_J_15.resize(numRowsTotal);
		m_J_16.resize(numRowsTotal);
		m_J_17.resize(numRowsTotal);

		m_J_rhs.resize(numRowsTotal);

		m_Asp_00.resize(numRowsTotal);
		m_Asp_01.resize(numRowsTotal);
		m_Asp_02.resize(numRowsTotal);
		m_Asp_03.resize(numRowsTotal);
		m_Asp_04.resize(numRowsTotal);
		m_Asp_05.resize(numRowsTotal);
		m_Asp_06.resize(numRowsTotal);
		m_Asp_07.resize(numRowsTotal);
		m_Asp_08.resize(numRowsTotal);
		m_Asp_09.resize(numRowsTotal);
		m_Asp_10.resize(numRowsTotal);
		m_Asp_11.resize(numRowsTotal);
		m_Asp_12.resize(numRowsTotal);
		m_Asp_13.resize(numRowsTotal);
		m_Asp_14.resize(numRowsTotal);
		m_Asp_15.resize(numRowsTotal);
		m_Asp_16.resize(numRowsTotal);
		m_Asp_17.resize(numRowsTotal);

		m_CFM.resize(numRowsTotal);

		m_invMassScale_00.resize(numRowsTotal);
		m_invMassScale_01.resize(numRowsTotal);
		m_invMassScale_02.resize(numRowsTotal);
		m_invMassScale_03.resize(numRowsTotal);
		m_invMassScale_04.resize(numRowsTotal);
		m_invMassScale_05.resize(numRowsTotal);

		m_JtdNodes_00.resize(numRowsTotal);
		m_JtdNodes_01.resize(numRowsTotal);
		m_JtdNodes_02.resize(numRowsTotal);
		m_JtdNodes_03.resize(numRowsTotal);
		m_JtdNodes_04.resize(numRowsTotal);
		m_JtdNodes_05.resize(numRowsTotal);

		m_lambda.resize(numRowsTotal);
		m_resid.resize(numRowsTotal);
		m_RHS.resize(numRowsTotal);

		m_Lo.resize(numRowsTotal);
		m_Hi.resize(numRowsTotal);

		m_invDiag.resize(numRowsTotal);

		setNumAdditionalRows(numRowsTotal);
	}

	void setNumNodes(uint32_t numNodesTotal)
	{
		m_numNodes = numNodesTotal;

		m_nodeInvMass_00.resize(numNodesTotal);
		m_nodeInvMass_01.resize(numNodesTotal);
		m_nodeInvMass_02.resize(numNodesTotal);
		m_nodeInvMass_10.resize(numNodesTotal);
		m_nodeInvMass_11.resize(numNodesTotal);
		m_nodeInvMass_12.resize(numNodesTotal);
		m_nodeInvMass_20.resize(numNodesTotal);
		m_nodeInvMass_21.resize(numNodesTotal);
		m_nodeInvMass_22.resize(numNodesTotal);

		m_Ftot_x.resize(numNodesTotal);
		m_Ftot_y.resize(numNodesTotal);
		m_Ftot_z.resize(numNodesTotal);
		m_nodeVel_x.resize(numNodesTotal);
		m_nodeVel_y.resize(numNodesTotal);
		m_nodeVel_z.resize(numNodesTotal);

		m_a_x.resize(numNodesTotal);
		m_a_y.resize(numNodesTotal);
		m_a_z.resize(numNodesTotal);
	}

//protected:
	unsigned int m_numNodes, m_numJoints;

	// Jacobian - matrix of derivatives [dC/dx]
	std::vector<float>	m_J_00, m_J_01, m_J_02, m_J_03, m_J_04, m_J_05,
						m_J_06, m_J_07, m_J_08, m_J_09, m_J_10, m_J_11,
						m_J_12, m_J_13, m_J_14, m_J_15, m_J_16, m_J_17;

	std::vector<float> m_J_rhs;								// dC / dt - J * (V + dt * M^-1 * Fext)

	// Asp = dt * M^-1 * J^T
	std::vector<float>	m_Asp_00, m_Asp_01, m_Asp_02, m_Asp_03, m_Asp_04, m_Asp_05,
						m_Asp_06, m_Asp_07, m_Asp_08, m_Asp_09, m_Asp_10, m_Asp_11,
						m_Asp_12, m_Asp_13, m_Asp_14, m_Asp_15, m_Asp_16, m_Asp_17;

	std::vector<float> m_CFM;									// Regularization, aka Constraint Force Mixing

	// Local joint mass scale - determines inv mass madification for a certain constrain (row);
	//	the conventional solver code would have each entry of these arrays set to 1.0
	// WARNING: the local mass scaling is disabled for LCPCG solver by default, as it decreases the convergence,
	//	please see ALLOW_LOCAL_MASS_SCALING preprocessor definition
	std::vector<float>	m_invMassScale_00, m_invMassScale_01, m_invMassScale_02,
						m_invMassScale_03, m_invMassScale_04, m_invMassScale_05;
	// Jointed nodes indices
	std::vector<unsigned int>	m_JtdNodes_00, m_JtdNodes_01, m_JtdNodes_02,
								m_JtdNodes_03, m_JtdNodes_04, m_JtdNodes_05;

	std::vector<float> m_lambda, m_resid;						// Lambda, Initial Guess [lambda from prev solve]
	std::vector<float> m_RHS;									// Right Hand Side, Ax = b <- THIS

	std::vector<float> m_Lo, m_Hi;								// Lo && Hi limits

	std::vector<float> m_invDiag;								// D^-1 from L + D + U


	// Per-node
	std::vector<float>	m_nodeInvMass_00, m_nodeInvMass_01, m_nodeInvMass_02,
						m_nodeInvMass_10, m_nodeInvMass_11, m_nodeInvMass_12,
						m_nodeInvMass_20, m_nodeInvMass_21, m_nodeInvMass_22;

	// total force V + dt * M^-1 * Fext - M^-1 * K(x - x0)
	std::vector<float> m_Ftot_x, m_Ftot_y, m_Ftot_z;
	std::vector<float> m_nodeVel_x, m_nodeVel_y, m_nodeVel_z;

	// Odd-job vector
	// PGS: a = Asp * lambda = dt * M^-1 * J^T * lambda
	std::vector<float> m_a_x, m_a_y, m_a_z;

	void emptyAVec()
	{
		// TODO: replace with memset
		for (size_t iNode = 0, iNodeEnd = m_a_x.size(); iNode < iNodeEnd; ++iNode)
		{
			m_a_x[iNode] = 0.0f;
			m_a_y[iNode] = 0.0f;
			m_a_z[iNode] = 0.0f;
		}
	}

protected:

	SolverBase(unsigned int maxIterations, float precision);

	// Common solver pre-step: zeroing out a-vector
	void prestep(float dt);

	// Cut initial guess when needed
	void cutLambdas();

#if (SOLVERS_ANALYZE_RESIDUAL == 1)
	// Solver Analysis
	float m_LambdaNormSq, m_GradNormSq, m_DotLambdaGrad;
#endif

	// Used for statistics measurement
	unsigned int m_effectiveIterations;

	bool m_IG_CutFlag;
	float m_IG_CutThreshold;

	// Precision of solver's convergence [ if ||residual|| / NumJoints < Precision ==> STOP ]
	float m_precision;

	// Maximum number of iterations
	unsigned int m_maxIterations;
};

}

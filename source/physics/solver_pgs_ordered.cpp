#include "physics/solver_pgs_ordered.h"
//#include "helpers/timer.h"

namespace physics
{

#define FORM_A_SPARSE(j, t0, t1, t2)\
			b_idx = ptr_m_JtdNodes_0##j[i];\
			invMassScaleNode = ptr_m_invMassScale_0##j[i];\
			if (b_idx != SOLVER_NO_BODY) {\
			\
			val_arr(m_RHS) -= (	ptr_m_J_##t0[i] * ptr_m_Ftot_x[b_idx] + \
								ptr_m_J_##t1[i] * ptr_m_Ftot_y[b_idx] +\
								ptr_m_J_##t2[i] * ptr_m_Ftot_z[b_idx] );\
			\
			val_arr( m_Asp_##t0 ) = dt * \
				( invMassScaleNode * ptr_m_NodeInvMass_00[b_idx] * ptr_m_J_##t0[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_01[b_idx] * ptr_m_J_##t1[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_02[b_idx] * ptr_m_J_##t2[i] );\
			val_arr(m_invDiag) += ptr_m_J_##t0[i] * val_arr( m_Asp_##t0 );\
			\
			val_arr( m_Asp_##t1 ) = dt * \
				( invMassScaleNode * ptr_m_NodeInvMass_10[b_idx] * ptr_m_J_##t0[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_11[b_idx] * ptr_m_J_##t1[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_12[b_idx] * ptr_m_J_##t2[i] );\
			val_arr(m_invDiag) += ptr_m_J_##t1[i] * val_arr( m_Asp_##t1 );\
			\
			val_arr( m_Asp_##t2 ) = dt * \
				( invMassScaleNode * ptr_m_NodeInvMass_20[b_idx] * ptr_m_J_##t0[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_21[b_idx] * ptr_m_J_##t1[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_22[b_idx] * ptr_m_J_##t2[i] );\
			\
			val_arr(m_invDiag) += ptr_m_J_##t2[i] * val_arr( m_Asp_##t2 );\
			} \
			inc_arr( m_Asp_##t0 ); \
			inc_arr( m_Asp_##t1 ); \
			inc_arr( m_Asp_##t2 );

#define CALC_VECTOR_A_ORDER(vec, j, t0, t1, t2)\
			b_idx = inc_arr( m_JtdNodes_0##j );\
			\
			ptr_m_a_x[b_idx] += inc_arr(m_Asp_##t0) * val_arr(vec);\
			ptr_m_a_y[b_idx] += inc_arr(m_Asp_##t1) * val_arr(vec);\
			ptr_m_a_z[b_idx] += inc_arr(m_Asp_##t2) * val_arr(vec);

#define MUL_J_A_T0_T1_ORDER(c_idx) \
	ptr_m_J_00[c_idx] * ptr_m_a_x[t0Idx] + ptr_m_J_01[c_idx] * ptr_m_a_y[t0Idx] + ptr_m_J_02[c_idx] * ptr_m_a_z[t0Idx] + \
	ptr_m_J_03[c_idx] * ptr_m_a_x[t1Idx] + ptr_m_J_04[c_idx] * ptr_m_a_y[t1Idx] + ptr_m_J_05[c_idx] * ptr_m_a_z[t1Idx];

#define MUL_J_A_T2_ORDER(c_idx) \
	ptr_m_J_06[c_idx] * ptr_m_a_x[t2Idx] + ptr_m_J_07[c_idx] * ptr_m_a_y[t2Idx] + ptr_m_J_08[c_idx] * ptr_m_a_z[t2Idx];
#define MUL_J_A_T3_ORDER(c_idx) \
	ptr_m_J_09[c_idx] * ptr_m_a_x[t3Idx] + ptr_m_J_10[c_idx] * ptr_m_a_y[t3Idx] + ptr_m_J_11[c_idx] * ptr_m_a_z[t3Idx];
#define MUL_J_A_T4_ORDER(c_idx) \
	ptr_m_J_12[c_idx] * ptr_m_a_x[t4Idx] + ptr_m_J_13[c_idx] * ptr_m_a_y[t4Idx] + ptr_m_J_14[c_idx] * ptr_m_a_z[t4Idx];
#define MUL_J_A_T5_ORDER(c_idx) \
	ptr_m_J_15[c_idx] * ptr_m_a_x[t5Idx] + ptr_m_J_16[c_idx] * ptr_m_a_y[t5Idx] + ptr_m_J_17[c_idx] * ptr_m_a_z[t5Idx];

void SolverPGSOrdered::solve(float dt)
{
	if (dt < 0.00001f)
	{
		return;
	}

	prestep(dt);

	if (m_numJoints == 0)
		return;

	uint32_t i, b_idx;
	float invMassScaleNode;

	float *ptr_m_a_x;
	float *ptr_m_a_y;
	float *ptr_m_a_z;

	float	*ptr_m_Asp_00, *ptr_m_Asp_01, *ptr_m_Asp_02, *ptr_m_Asp_03, *ptr_m_Asp_04, *ptr_m_Asp_05,
			*ptr_m_Asp_06, *ptr_m_Asp_07, *ptr_m_Asp_08, *ptr_m_Asp_09, *ptr_m_Asp_10, *ptr_m_Asp_11;
	float	*ptr_m_Asp_12, *ptr_m_Asp_13, *ptr_m_Asp_14, *ptr_m_Asp_15, *ptr_m_Asp_16, *ptr_m_Asp_17;

	float	*ptr_m_J_00, *ptr_m_J_01, *ptr_m_J_02, *ptr_m_J_03, *ptr_m_J_04, *ptr_m_J_05,
			*ptr_m_J_06, *ptr_m_J_07, *ptr_m_J_08, *ptr_m_J_09, *ptr_m_J_10, *ptr_m_J_11;
	float	*ptr_m_J_12, *ptr_m_J_13, *ptr_m_J_14, *ptr_m_J_15, *ptr_m_J_16, *ptr_m_J_17;

	float *ptr_m_lambda;

	float	*ptr_m_invMassScale_00, *ptr_m_invMassScale_01, *ptr_m_invMassScale_02, *ptr_m_invMassScale_03,
			*ptr_m_invMassScale_04, *ptr_m_invMassScale_05;

	unsigned int *ptr_m_JtdNodes_00, *ptr_m_JtdNodes_01, *ptr_m_JtdNodes_02, *ptr_m_JtdNodes_03;
	unsigned int *ptr_m_JtdNodes_04, *ptr_m_JtdNodes_05;

	float *ptr_m_J_rhs;
	float *ptr_m_RHS;
	float *ptr_m_Lo, *ptr_m_Hi;
	float *ptr_m_invDiag;

	float idt = 1.0f / dt;

	// SOR coefficient omega
	float w = 1.0f;

	INIT_A_SPARSE_PTRS(0);

	ptr_m_J_rhs = &m_J_rhs[0];
	ptr_m_invDiag = &m_invDiag[0];

	INIT_INV_MASS_SCALE_PTRS(0);

	INIT_NODES_IDX_PTRS(0);

	ptr_m_RHS = &m_RHS[0];

	INIT_J_SPARSE_PTRS(0);

	float	*ptr_m_NodeInvMass_00 = &m_nodeInvMass_00[0],
			*ptr_m_NodeInvMass_01 = &m_nodeInvMass_01[0],
			*ptr_m_NodeInvMass_02 = &m_nodeInvMass_02[0],
			*ptr_m_NodeInvMass_10 = &m_nodeInvMass_10[0],
			*ptr_m_NodeInvMass_11 = &m_nodeInvMass_11[0],
			*ptr_m_NodeInvMass_12 = &m_nodeInvMass_12[0],
			*ptr_m_NodeInvMass_20 = &m_nodeInvMass_20[0],
			*ptr_m_NodeInvMass_21 = &m_nodeInvMass_21[0],
			*ptr_m_NodeInvMass_22 = &m_nodeInvMass_22[0];

	float	*ptr_m_Ftot_x = &m_Ftot_x[0],
			*ptr_m_Ftot_y = &m_Ftot_y[0],
			*ptr_m_Ftot_z = &m_Ftot_z[0];

	// Ftot = v_{t-1} + dt * M^{-1} * Fext
	// RHS = dc/dt - J*Ftot
	// Asp = dt * M^{-1} * J^T
	// invDiag = w / (J * Asp + dt*CFM) = w / (dt*J*M^{-1}*J^T + dt*CFM)
	for (i = 0; i < m_numJoints; ++i)
	{
		m_CFM[i] *= dt;

		val_arr(m_RHS) = inc_arr(m_J_rhs);

		val_arr(m_invDiag) = 0.0f;

		FORM_A_SPARSE(0, 00, 01, 02);
		FORM_A_SPARSE(1, 03, 04, 05);
		FORM_A_SPARSE(2, 06, 07, 08);
		FORM_A_SPARSE(3, 09, 10, 11);

		FORM_A_SPARSE(4, 12, 13, 14);
		FORM_A_SPARSE(5, 15, 16, 17);

		val_arr(m_invDiag) = w / (val_arr(m_invDiag) + m_CFM[i]);

		++ptr_m_invDiag;
		++ptr_m_RHS;
	}

	INIT_A_SPARSE_PTRS(0);

	ptr_m_lambda = &m_lambda[0];

	INIT_NODES_IDX_PTRS(0);

	ptr_m_a_x = &m_a_x[0];
	ptr_m_a_y = &m_a_y[0];
	ptr_m_a_z = &m_a_z[0];

	// a = Asp * lambda = dt * M^{-1} * J^T * lambda
	for (i = 0; i < m_numJoints; ++i)
	{
		CALC_VECTOR_A_ORDER(m_lambda, 0, 00, 01, 02);
		CALC_VECTOR_A_ORDER(m_lambda, 1, 03, 04, 05);
		CALC_VECTOR_A_ORDER(m_lambda, 2, 06, 07, 08);
		CALC_VECTOR_A_ORDER(m_lambda, 3, 09, 10, 11);

		CALC_VECTOR_A_ORDER(m_lambda, 4, 12, 13, 14);
		CALC_VECTOR_A_ORDER(m_lambda, 5, 15, 16, 17);

		++ptr_m_lambda;
	}

	float dLam;

	m_effectiveIterations = m_maxIterations;

	uint32_t * ptr_m_permutIdx = &m_permutIdx[0];
	if (m_order == Order::eRandom)
	{
		for (uint32_t permIdx = 0; permIdx < m_numJoints; ++permIdx)
		{
			ptr_m_permutIdx[permIdx] = permIdx;
		}
	}
	for (unsigned k = 0; k < m_maxIterations; ++k)
	{
		INIT_A_SPARSE_PTRS(0);

		ptr_m_lambda = &m_lambda[0];

		INIT_NODES_IDX_PTRS(0);

		ptr_m_a_x = &m_a_x[0];
		ptr_m_a_y = &m_a_y[0];
		ptr_m_a_z = &m_a_z[0];

		INIT_J_SPARSE_PTRS(0);

		ptr_m_Lo = &m_Lo[0];
		ptr_m_Hi = &m_Hi[0];

		ptr_m_RHS = &m_RHS[0];

#if (SOLVER_STOPPING_CRITERIA == 1)
		float grad_norm = 0.0f;
#endif

		// Fisher-Yates permutation
		if (m_order == Order::eRandom)
		{
			for (uint32_t permIdx = 0; permIdx < m_numJoints; ++permIdx)
			{
				uint32_t newPermIdx = (rand()%m_numJoints);
				uint32_t temp = ptr_m_permutIdx[newPermIdx];
				ptr_m_permutIdx[newPermIdx] = ptr_m_permutIdx[permIdx];
				ptr_m_permutIdx[permIdx] = temp;
			}
		}

		// dLam = D^-1*(RHS - J*a - CFM*lambda) = D^{-1}*(RHS - (J*M^{-1}*J^T)*lambda - CFM*lambda)
		// lambda' = clamp(lambda + dLam, lo, hi)
		// dLam' = lambda' - lambda
		// a' = a + Asp * dLam'
		for (i = 0; i < m_numJoints; ++i)
		{
			uint32_t c_idx = i;
			if (m_order == Order::eBackward)
				c_idx = m_numJoints - i - 1;
			else if (m_order == Order::eRandom || m_order == Order::eCustom)
				c_idx = ptr_m_permutIdx[i];
			else if (m_order == Order::eLocalShuffle2)
			{
				if (k&1)
				{
					// Each odd iteration shuffle (even, odd) constraint rows pair 
					if (i&1)
					{
						c_idx = i-1;
					}
					else
					{
						c_idx = i+1;
						if (c_idx == m_numJoints)
							c_idx = m_numJoints - 1;
					}
				}
				assert(c_idx < m_numJoints);
			}
			else if (m_order == Order::eSplit2)
			{
				if (k&1)
				{
					// Each odd iteration solve even constraint rows first, then all odd rows
					c_idx = i*2;
					if (c_idx >= m_numJoints)
					{
						c_idx = (c_idx-m_numJoints)+1;
					}
				}
				assert(c_idx < m_numJoints);
			}

			// Current Row of Matrix B
			dLam = ptr_m_RHS[c_idx];

			float	LamPrev = ptr_m_lambda[c_idx],
					LamCurrent;

			// dLam -= <(J*M^{-1}*J^T)*lambda>
			// dLam -= <J*Asp*lambda>
			// dLam -= <J*a>

			unsigned int t0Idx = ptr_m_JtdNodes_00[c_idx];
			unsigned int t1Idx = ptr_m_JtdNodes_01[c_idx];
			unsigned int t2Idx = ptr_m_JtdNodes_02[c_idx];
			unsigned int t3Idx = ptr_m_JtdNodes_03[c_idx];

			unsigned int t4Idx = ptr_m_JtdNodes_04[c_idx];
			unsigned int t5Idx = ptr_m_JtdNodes_05[c_idx];

			// J * a
			dLam -= MUL_J_A_T0_T1_ORDER(c_idx);

			if (t2Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T2_ORDER(c_idx);
			}
			if (t3Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T3_ORDER(c_idx);
			}
			if (t4Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T4_ORDER(c_idx);
			}
			if (t5Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T5_ORDER(c_idx);
			}

			// Add regularization
			dLam -= m_CFM[c_idx] * ptr_m_lambda[c_idx];

#if (SOLVER_STOPPING_CRITERIA == 1)
			// dLambda = clamp< D^-1*(b - (L+D+U)*x) >
			// so grad is w/o D^-1 and clamping and with other sign
			grad_norm += dLam * dLam;
#endif

			dLam *= m_invDiag[c_idx];

			LamCurrent = LamPrev + dLam;

			// PROJECTION GOES HERE
			// Cut LamCurrent according to loLim, hiLim
			if (LamCurrent < ptr_m_Lo[c_idx])
			{
				LamCurrent = ptr_m_Lo[c_idx];
			}
			if (LamCurrent > ptr_m_Hi[c_idx])
			{
				LamCurrent = ptr_m_Hi[c_idx];
			}

			dLam = LamCurrent - LamPrev;
			ptr_m_lambda[c_idx] = LamCurrent;

			ptr_m_a_x[t0Idx] += ptr_m_Asp_00[c_idx] * dLam;
			ptr_m_a_y[t0Idx] += ptr_m_Asp_01[c_idx] * dLam;
			ptr_m_a_z[t0Idx] += ptr_m_Asp_02[c_idx] * dLam;
			ptr_m_a_x[t1Idx] += ptr_m_Asp_03[c_idx] * dLam;
			ptr_m_a_y[t1Idx] += ptr_m_Asp_04[c_idx] * dLam;
			ptr_m_a_z[t1Idx] += ptr_m_Asp_05[c_idx] * dLam;
			if (t2Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t2Idx] += ptr_m_Asp_06[c_idx] * dLam;
				ptr_m_a_y[t2Idx] += ptr_m_Asp_07[c_idx] * dLam;
				ptr_m_a_z[t2Idx] += ptr_m_Asp_08[c_idx] * dLam;
			}
			if (t3Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t3Idx] += ptr_m_Asp_09[c_idx] * dLam;
				ptr_m_a_y[t3Idx] += ptr_m_Asp_10[c_idx] * dLam;
				ptr_m_a_z[t3Idx] += ptr_m_Asp_11[c_idx] * dLam;
			}

			if (t4Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t4Idx] += ptr_m_Asp_12[c_idx] * dLam;
				ptr_m_a_y[t4Idx] += ptr_m_Asp_13[c_idx] * dLam;
				ptr_m_a_z[t4Idx] += ptr_m_Asp_14[c_idx] * dLam;
			}
			if (t5Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t5Idx] += ptr_m_Asp_15[c_idx] * dLam;
				ptr_m_a_y[t5Idx] += ptr_m_Asp_16[c_idx] * dLam;
				ptr_m_a_z[t5Idx] += ptr_m_Asp_17[c_idx] * dLam;
			}
		}

#if (SOLVER_STOPPING_CRITERIA == 1)

		// Stopping criteria
		if (grad_norm / m_numJoints < m_precision)
		{
			m_effectiveIterations = k;
			break;
		}

#endif
	}

#if (SOLVERS_ANALYZE_RESIDUAL == 1)
	m_GradNormSq = 0.0f;
	m_LambdaNormSq = 0.0f;
	m_DotLambdaGrad = 0.0f;

	ptr_m_lambda = &m_lambda[0];

	INIT_NODES_IDX_PTRS(0);

	ptr_m_a_x = &m_a_x[0];
	ptr_m_a_y = &m_a_y[0];
	ptr_m_a_z = &m_a_z[0];

	INIT_J_SPARSE_PTRS(0);

	ptr_m_Lo = &m_Lo[0];
	ptr_m_Hi = &m_Hi[0];

	ptr_m_RHS = &m_RHS[0];

	// Calculate residual vector
	for (i = 0; i < m_numJoints; ++i)
	{
		unsigned int t0Idx = inc_arr(m_JtdNodes_00);
		unsigned int t1Idx = inc_arr(m_JtdNodes_01);
		unsigned int t2Idx = inc_arr(m_JtdNodes_02);
		unsigned int t3Idx = inc_arr(m_JtdNodes_03);

		unsigned int t4Idx = inc_arr(m_JtdNodes_04);
		unsigned int t5Idx = inc_arr(m_JtdNodes_05);

		// J * a
		dLam = MUL_J_A_T0_T1();

		if (t2Idx != SOLVER_NO_BODY)
		{
			dLam += MUL_J_A_T2();
		}
		if (t3Idx != SOLVER_NO_BODY)
		{
			dLam += MUL_J_A_T3();
		}
		if (t4Idx != SOLVER_NO_BODY)
		{
			dLam += MUL_J_A_T4();
		}
		if (t5Idx != SOLVER_NO_BODY)
		{
			dLam += MUL_J_A_T5();
		}

		ADVANCE_J();

		// Add regularization
		dLam +=	m_CFM[i] * ptr_m_lambda[i];

		dLam -= ptr_m_RHS[i];

		m_resid[i] = dLam;

		// dLam = Ax - b = gradient
		m_GradNormSq += dLam * dLam;
		m_LambdaNormSq += ptr_m_lambda[i] * ptr_m_lambda[i];
		m_DotLambdaGrad += dLam * ptr_m_lambda[i];		
	}
#endif
}

#undef FORM_A_SPARSE

}

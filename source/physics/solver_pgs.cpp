#include "physics/solver_pgs.h"
//#include "helpers/timer.h"

namespace physics
{

#define FORM_A_SPARSE(j, t0, t1, t2)\
			idx = ptr_m_JtdNodes_0##j[i];\
			invMassScaleNode = ptr_m_invMassScale_0##j[i];\
			if (idx != SOLVER_NO_BODY) {\
			\
			val_arr(m_RHS) -= (	ptr_m_J_##t0[i] * ptr_m_Ftot_x[idx] + \
								ptr_m_J_##t1[i] * ptr_m_Ftot_y[idx] +\
								ptr_m_J_##t2[i] * ptr_m_Ftot_z[idx] );\
			\
			val_arr( m_Asp_##t0 ) = dt * \
				( invMassScaleNode * ptr_m_NodeInvMass_00[idx] * ptr_m_J_##t0[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_01[idx] * ptr_m_J_##t1[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_02[idx] * ptr_m_J_##t2[i] );\
			val_arr(m_invDiag) += ptr_m_J_##t0[i] * val_arr( m_Asp_##t0 );\
			\
			val_arr( m_Asp_##t1 ) = dt * \
				( invMassScaleNode * ptr_m_NodeInvMass_10[idx] * ptr_m_J_##t0[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_11[idx] * ptr_m_J_##t1[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_12[idx] * ptr_m_J_##t2[i] );\
			val_arr(m_invDiag) += ptr_m_J_##t1[i] * val_arr( m_Asp_##t1 );\
			\
			val_arr( m_Asp_##t2 ) = dt * \
				( invMassScaleNode * ptr_m_NodeInvMass_20[idx] * ptr_m_J_##t0[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_21[idx] * ptr_m_J_##t1[i] \
				+ invMassScaleNode * ptr_m_NodeInvMass_22[idx] * ptr_m_J_##t2[i] );\
			\
			val_arr(m_invDiag) += ptr_m_J_##t2[i] * val_arr( m_Asp_##t2 );\
			} \
			inc_arr( m_Asp_##t0 ); \
			inc_arr( m_Asp_##t1 ); \
			inc_arr( m_Asp_##t2 );

void SolverPGS::solve(float dt)
{
	if (dt < 0.00001f)
	{
		return;
	}

	prestep(dt);

	if (m_numJoints == 0)
		return;

	unsigned int i, idx;
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
		CALC_VECTOR_A(m_lambda, 0, 00, 01, 02);
		CALC_VECTOR_A(m_lambda, 1, 03, 04, 05);
		CALC_VECTOR_A(m_lambda, 2, 06, 07, 08);
		CALC_VECTOR_A(m_lambda, 3, 09, 10, 11);

		CALC_VECTOR_A(m_lambda, 4, 12, 13, 14);
		CALC_VECTOR_A(m_lambda, 5, 15, 16, 17);

		++ptr_m_lambda;
	}

	float dLam;

	m_effectiveIterations = m_maxIterations;

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

		// dLam = D^-1*(RHS - J*a - CFM*lambda) = D^{-1}*(RHS - (J*M^{-1}*J^T)*lambda - CFM*lambda)
		// lambda' = clamp(lambda + dLam, lo, hi)
		// dLam' = lambda' - lambda
		// a' = a + Asp * dLam'
		for (i = 0; i < m_numJoints; ++i)
		{
			// Current Row of Matrix B
			dLam = inc_arr(m_RHS);

			float	LamPrev = val_arr(m_lambda),
					LamCurrent;

			// dLam -= <(J*M^{-1}*J^T)*lambda>
			// dLam -= <J*Asp*lambda>
			// dLam -= <J*a>

			unsigned int t0Idx = inc_arr(m_JtdNodes_00);
			unsigned int t1Idx = inc_arr(m_JtdNodes_01);
			unsigned int t2Idx = inc_arr(m_JtdNodes_02);
			unsigned int t3Idx = inc_arr(m_JtdNodes_03);

			unsigned int t4Idx = inc_arr(m_JtdNodes_04);
			unsigned int t5Idx = inc_arr(m_JtdNodes_05);

			// J * a
			dLam -= MUL_J_A_T0_T1();

			if (t2Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T2();
			}
			if (t3Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T3();
			}
			if (t4Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T4();
			}
			if (t5Idx != SOLVER_NO_BODY)
			{
				dLam -= MUL_J_A_T5();
			}

			ADVANCE_J();

			// Add regularization
			dLam -= m_CFM[i] * val_arr(m_lambda);

#if (SOLVER_STOPPING_CRITERIA == 1)
			// dLambda = clamp< D^-1*(b - (L+D+U)*x) >
			// so grad is w/o D^-1 and clamping and with other sign
			grad_norm += dLam * dLam;
#endif

			dLam *= m_invDiag[i];

			LamCurrent = LamPrev + dLam;

			// PROJECTION GOES HERE
			// Cut LamCurrent according to loLim, hiLim
			if (LamCurrent < val_arr(m_Lo))
			{
				LamCurrent = val_arr(m_Lo);
			}
			if (LamCurrent > val_arr(m_Hi))
			{
				LamCurrent = val_arr(m_Hi);
			}

			++ptr_m_Lo;
			++ptr_m_Hi;

			dLam = LamCurrent - LamPrev;
			inc_arr(m_lambda) = LamCurrent;

			ptr_m_a_x[t0Idx] += inc_arr(m_Asp_00) * dLam;
			ptr_m_a_y[t0Idx] += inc_arr(m_Asp_01) * dLam;
			ptr_m_a_z[t0Idx] += inc_arr(m_Asp_02) * dLam;
			ptr_m_a_x[t1Idx] += inc_arr(m_Asp_03) * dLam;
			ptr_m_a_y[t1Idx] += inc_arr(m_Asp_04) * dLam;
			ptr_m_a_z[t1Idx] += inc_arr(m_Asp_05) * dLam;
			if (t2Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t2Idx] += val_arr(m_Asp_06) * dLam;
				ptr_m_a_y[t2Idx] += val_arr(m_Asp_07) * dLam;
				ptr_m_a_z[t2Idx] += val_arr(m_Asp_08) * dLam;
			}
			if (t3Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t3Idx] += val_arr(m_Asp_09) * dLam;
				ptr_m_a_y[t3Idx] += val_arr(m_Asp_10) * dLam;
				ptr_m_a_z[t3Idx] += val_arr(m_Asp_11) * dLam;
			}
			inc_arr(m_Asp_06);
			inc_arr(m_Asp_07);
			inc_arr(m_Asp_08);
			inc_arr(m_Asp_09);
			inc_arr(m_Asp_10);
			inc_arr(m_Asp_11);

			if (t4Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t4Idx] += val_arr(m_Asp_12) * dLam;
				ptr_m_a_y[t4Idx] += val_arr(m_Asp_13) * dLam;
				ptr_m_a_z[t4Idx] += val_arr(m_Asp_14) * dLam;
			}
			if (t5Idx != SOLVER_NO_BODY)
			{
				ptr_m_a_x[t5Idx] += val_arr(m_Asp_15) * dLam;
				ptr_m_a_y[t5Idx] += val_arr(m_Asp_16) * dLam;
				ptr_m_a_z[t5Idx] += val_arr(m_Asp_17) * dLam;
			}
			inc_arr(m_Asp_12);
			inc_arr(m_Asp_13);
			inc_arr(m_Asp_14);
			inc_arr(m_Asp_15);
			inc_arr(m_Asp_16);
			inc_arr(m_Asp_17);
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

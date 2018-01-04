#include "physics/solver_base.h"

namespace physics
{

SolverBase::SolverBase(unsigned int MaxIterations, float Precision)
{
	m_IG_CutFlag = false;
	m_IG_CutThreshold = 10000.0f;

	m_precision = Precision;
	m_maxIterations = MaxIterations;

#if (SOLVERS_ANALYZE_RESIDUAL == 1)
	m_LambdaNormSq = m_GradNormSq = m_DotLambdaGrad = 0.0f;
#endif

	m_numJoints = 0;
	m_numNodes = 1;
}

void SolverBase::prestep(float dt)
{
	float *ptr_m_a_x = &m_a_x[0];
	float *ptr_m_a_y = &m_a_y[0];
	float *ptr_m_a_z = &m_a_z[0];

	float *ptr_m_Ftot_x = &m_Ftot_x[0];
	float *ptr_m_Ftot_y = &m_Ftot_y[0];
	float *ptr_m_Ftot_z = &m_Ftot_z[0];

	float *ptr_m_NodeVel_x = &m_nodeVel_x[0];
	float *ptr_m_NodeVel_y = &m_nodeVel_y[0];
	float *ptr_m_NodeVel_z = &m_nodeVel_z[0];

	float *ptr_m_NodeInvMass_00 = &m_nodeInvMass_00[0];
	float *ptr_m_NodeInvMass_01 = &m_nodeInvMass_01[0];
	float *ptr_m_NodeInvMass_02 = &m_nodeInvMass_02[0];
	float *ptr_m_NodeInvMass_10 = &m_nodeInvMass_10[0];
	float *ptr_m_NodeInvMass_11 = &m_nodeInvMass_11[0];
	float *ptr_m_NodeInvMass_12 = &m_nodeInvMass_12[0];
	float *ptr_m_NodeInvMass_20 = &m_nodeInvMass_20[0];
	float *ptr_m_NodeInvMass_21 = &m_nodeInvMass_21[0];
	float *ptr_m_NodeInvMass_22 = &m_nodeInvMass_22[0];

	cutLambdas();

	for (unsigned int i = 0; i < m_numNodes; ++i)
	{
		inc_arr(m_a_x) = 0.0f;
		inc_arr(m_a_y) = 0.0f;
		inc_arr(m_a_z) = 0.0f;
	}
}

void SolverBase::cutLambdas()
{
#if (SOLVERS_ANALYZE_RESIDUAL == 1)
	if (m_IG_CutFlag)
	{
		if (m_LambdaNormSq / m_numJoints > m_IG_CutThreshold)
		{
			for (unsigned int i = 0; i < m_numJoints; ++i)
			{
				m_lambda[i] = 0.0f;
			}
		}
	}
#endif
}

float SolverBase::calcSylvesterCriterion()
{
	// J[CDoFs x 3*n] * M^-1 [3*n x 3*n] * J^T [3*n x CDoFs]
	float *A_Matrix = new float[m_numJoints * m_numJoints];

	float	*L = new float [m_numJoints * m_numJoints],
			*D = new float [m_numJoints];

#define AMat(i, j) (A_Matrix[(i) * m_numJoints + (j)])
#define LMat(i, j) (L[(i) * m_numJoints + (j)])

	unsigned int i, j, k;

	// Set A_Matrix to zero
	memset(A_Matrix, 0, m_numJoints * m_numJoints * sizeof(float));

	// Asp already calculated on "Solve" step
	unsigned int	*ptr_JtdNodes_00 = &m_JtdNodes_00[0],
					*ptr_JtdNodes_01 = &m_JtdNodes_01[0],
					*ptr_JtdNodes_02 = &m_JtdNodes_02[0],
					*ptr_JtdNodes_03 = &m_JtdNodes_03[0];
	unsigned int	*ptr_JtdNodes_04 = &m_JtdNodes_04[0],
					*ptr_JtdNodes_05 = &m_JtdNodes_05[0];

	float	*ptr_m_Asp_00, *ptr_m_Asp_01, *ptr_m_Asp_02, *ptr_m_Asp_03, *ptr_m_Asp_04, *ptr_m_Asp_05,
			*ptr_m_Asp_06, *ptr_m_Asp_07, *ptr_m_Asp_08, *ptr_m_Asp_09, *ptr_m_Asp_10, *ptr_m_Asp_11;
	float	*ptr_m_Asp_12, *ptr_m_Asp_13, *ptr_m_Asp_14, *ptr_m_Asp_15, *ptr_m_Asp_16, *ptr_m_Asp_17;

	float	*ptr_m_J_00, *ptr_m_J_01, *ptr_m_J_02, *ptr_m_J_03, *ptr_m_J_04, *ptr_m_J_05,
			*ptr_m_J_06, *ptr_m_J_07, *ptr_m_J_08, *ptr_m_J_09, *ptr_m_J_10, *ptr_m_J_11;
	float	*ptr_m_J_12, *ptr_m_J_13, *ptr_m_J_14, *ptr_m_J_15, *ptr_m_J_16, *ptr_m_J_17;


	ptr_m_Asp_00 = &m_Asp_00[0];
	ptr_m_Asp_01 = &m_Asp_01[0];
	ptr_m_Asp_02 = &m_Asp_02[0];
	ptr_m_Asp_03 = &m_Asp_03[0];
	ptr_m_Asp_04 = &m_Asp_04[0];
	ptr_m_Asp_05 = &m_Asp_05[0];
	ptr_m_Asp_06 = &m_Asp_06[0];
	ptr_m_Asp_07 = &m_Asp_07[0];
	ptr_m_Asp_08 = &m_Asp_08[0];
	ptr_m_Asp_09 = &m_Asp_09[0];
	ptr_m_Asp_10 = &m_Asp_10[0];
	ptr_m_Asp_11 = &m_Asp_11[0];

	ptr_m_Asp_12 = &m_Asp_12[0];
	ptr_m_Asp_13 = &m_Asp_13[0];
	ptr_m_Asp_14 = &m_Asp_14[0];
	ptr_m_Asp_15 = &m_Asp_15[0];
	ptr_m_Asp_16 = &m_Asp_16[0];
	ptr_m_Asp_17 = &m_Asp_17[0];


	ptr_m_J_00 = &m_J_00[0];
	ptr_m_J_01 = &m_J_01[0];
	ptr_m_J_02 = &m_J_02[0];
	ptr_m_J_03 = &m_J_03[0];
	ptr_m_J_04 = &m_J_04[0];
	ptr_m_J_05 = &m_J_05[0];
	ptr_m_J_06 = &m_J_06[0];
	ptr_m_J_07 = &m_J_07[0];
	ptr_m_J_08 = &m_J_08[0];
	ptr_m_J_09 = &m_J_09[0];
	ptr_m_J_10 = &m_J_10[0];
	ptr_m_J_11 = &m_J_11[0];

	ptr_m_J_12 = &m_J_12[0];
	ptr_m_J_13 = &m_J_13[0];
	ptr_m_J_14 = &m_J_14[0];
	ptr_m_J_15 = &m_J_15[0];
	ptr_m_J_16 = &m_J_16[0];
	ptr_m_J_17 = &m_J_17[0];


	for (i = 0; i < m_numJoints; ++i)
	{
		for (j = 0; j < m_numJoints; ++j)
		{
			AMat(i, j) = 0.0f;

#define CALCULATE_JWJ(k, l, kt0, kt1, kt2, lt0, lt1, lt2)\
					if (ptr_JtdNodes_0##k[i] == ptr_JtdNodes_0##l[j])\
					{\
						int node_idx = ptr_JtdNodes_0##k[i];\
						\
						AMat(i, j) += ptr_m_J_##kt0[i] * ptr_m_Asp_##lt0[j]\
									+ ptr_m_J_##kt1[i] * ptr_m_Asp_##lt1[j]\
									+ ptr_m_J_##kt2[i] * ptr_m_Asp_##lt2[j];\
					}

#define CALCULATE_JWJ_L(k, kt0, kt1, kt2)\
					CALCULATE_JWJ(k, 0, kt0, kt1, kt2, 00, 01, 02);\
					CALCULATE_JWJ(k, 1, kt0, kt1, kt2, 03, 04, 05);\
					CALCULATE_JWJ(k, 2, kt0, kt1, kt2, 06, 07, 08);\
					CALCULATE_JWJ(k, 3, kt0, kt1, kt2, 09, 10, 11);\
					CALCULATE_JWJ(k, 4, kt0, kt1, kt2, 12, 13, 14);\
					CALCULATE_JWJ(k, 5, kt0, kt1, kt2, 15, 16, 17);

			CALCULATE_JWJ_L(0, 00, 01, 02);
			CALCULATE_JWJ_L(1, 03, 04, 05);
			CALCULATE_JWJ_L(2, 06, 07, 08);
			CALCULATE_JWJ_L(3, 09, 10, 11);
			CALCULATE_JWJ_L(4, 12, 13, 14);
			CALCULATE_JWJ_L(5, 15, 16, 17);

#undef CALCULATE_JWJ_L

#undef CALCULATE_JWJ

		}
	}

	// Add regularization
	float *CFM_ptr = &m_CFM[0];
	for (i = 0; i < m_numJoints; ++i)
	{
		AMat(i, i) += CFM_ptr[i];
	}

	for (i = 0; i < m_numJoints; ++i)
	{
		for (j = 0; j < m_numJoints; ++j)
		{
			AMat(i, j) *= m_invDiag[i];
		}
	}

	// Symmetry testing
	/* DBG: PRINT */
	int Sym = 1;

//	FILE *fp_matrix_log;

//	fp_matrix_log = fopen("matrix.log", "w");

//	fprintf(fp_matrix_log, "{");
	for (i = 0; i < m_numJoints; ++i)
	{
//		fprintf(fp_matrix_log, "{");
		for (j = 0; j < m_numJoints; ++j)
		{
			//if (j != m_numJoints - 1)
			//	fprintf(fp_matrix_log, "%f,", AMat(i, j));
			//else
			//	fprintf(fp_matrix_log, "%f", AMat(i, j));
		}
		//if (i != m_numJoints - 1)
		//	fprintf(fp_matrix_log, "},");
		//else
		//	fprintf(fp_matrix_log, "}");

		for (j = i + 1; j < m_numJoints; ++j)
		{
			float tmp1 = AMat(i, j);
			float tmp2 = AMat(j, i);
			if (fast_abs(AMat(i, j) - AMat(j, i)) > 0.01f)
			{
				assert(false);
				Sym = 0;
				break;
			}
		}
	}
	//fprintf(fp_matrix_log, "}\n\n");

	//fclose(fp_matrix_log);

	if (Sym == 0)
	{
		//log.print("Matrix is not self-conjugate!");
	}
	else
	{
		//log.print("Matrix IS self-conjugate!");
	}

	int PD = 1;
	int CurDetDimension;
	float lastDet = 0.0f;

	// Determinant, Sylvester Criterion
	// We must find NumJoints determinants for the upper left corner of matrix
	// [ Determinant is found via Cholesky Decomposition and U-matrix diagonal mult ]

	// Cholesky Decomposition [ WIKI version ]

	//log.print("Diagonal in Cholesky Decomposition:");

	lastDet = 1.0f;

	// General case: i = 0 .. n
	for (i = 0; i < m_numJoints; ++i)
	{
		float temp_var;

		for (j = 0; j < i; ++j)
		{
			temp_var = AMat(i, j);
			for (k = 0; k < j; ++k)
			{
				temp_var -= LMat(i, k) * LMat(j, k) * D[k];
			}

			LMat(i, j) = temp_var / D[j];
			LMat(j, i) = 0.0f;
		}

		temp_var = AMat(i, i);
		for (k = 0; k < i; ++k)
		{
			temp_var -= LMat(i, k) * LMat(i, k) * D[k];
		}

		D[i] = temp_var;
		lastDet *= temp_var;
		//log.print("%d: %f; [%f]", i, temp_var, lastDet);
		LMat(i, i) = 1.0f;
	}

	//log.print("\n\n");

	// Determinant: det(L * D * L^T) = det L * det D * det L = (MUL Lii) ^2 * (MUL Dii)
	lastDet = 1.0f;
	for (i = 0; i < m_numJoints; ++i)
	{
		lastDet *= LMat(i, i) * D[i] * LMat(i, i);

		if (D[i] < 0.0f)
		{
			CurDetDimension = i;
			PD = 0;
			break;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Test if A == L * D * L^T
	float *LD = new float [m_numJoints * m_numJoints];

#define LDMat(i, j) (LD[(i) * m_numJoints + (j)])

	for (i = 0; i < m_numJoints; ++i)
	{
		for (j = 0; j < m_numJoints; ++j)
		{
			LDMat(i, j) = LMat(i, j) * D[j];
		}
	}

	int DecompEq = 1;
	for (i = 0; i < m_numJoints; ++i)
	{
		for (j = 0; j < m_numJoints; ++j)
		{
			float Res = 0.0f;
			for (k = 0; k < m_numJoints; ++k)
			{
				// LD * L^T
				Res += LDMat(i, k) * LMat(j, k);
			}

			float tmp = AMat(i, j);
			if (fast_abs(Res - tmp) > 0.1f)
			{
				assert(false);
				DecompEq = 0;
				break;
			}
		}
	}

	if (DecompEq == 0)
	{
		//log.print("Decomposition is INCORRECT!");
	}
	else
	{
		//log.print("Decomposition is correct!");
	}

#undef LDMat

	delete [] LD;
	//////////////////////////////////////////////////////////////////////////

#undef RMat
#undef AMat

	delete [] A_Matrix;

	delete [] L;
	delete [] D;

	if (PD == 0)
	{
		//log.print("Sylvester's criterion failed (step %d, det * 1000.0 = %.16f)!", CurDetDimension, lastDet);
	}
	else
	{
		//log.print("Sylvester's criterion OK, det A * 1000.0 = %.16f;", lastDet);
	}

	return lastDet;
}

}

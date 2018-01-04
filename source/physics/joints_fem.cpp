#include "physics/joints_fem.h"
#include "physics/phys_helpers.h"

namespace physics
{

//////////////////////////////////////////////////////////////////////////
// FEMJoint
//////////////////////////////////////////////////////////////////////////

void FEMJoint::setYoungPoisson(float Young, float Poisson)
{
	m_YoungModulus = Young;
	m_PoissonRatio = Poisson;

	float D_loc[6 * 6];
	memset(D_loc, 0, 6 * 6 * sizeof(float));

	// D
	float sigma1 = m_PoissonRatio / (1 - m_PoissonRatio);
	float premult = m_YoungModulus / ((1 + m_PoissonRatio) * ( 1 - sigma1));
	float coef1 = premult * sigma1;
	float coef2 = 0.5f * premult * (1 - sigma1);

	D_loc[0 * 6 + 0] = premult * (1.0f - sigma1);
	D_loc[1 * 6 + 1] = premult * (1.0f + 2.0f * sigma1);
	D_loc[2 * 6 + 2] = premult * (1.0f - sigma1);

	D_loc[3 * 6 + 3] = coef2;
	D_loc[4 * 6 + 4] = coef2;
	D_loc[5 * 6 + 5] = coef2;

	float CFM_mult = 1.0f / m_initialFEVolume;

	m_CFM[0] = CFM_mult / D_loc[0 * 6 + 0];
	m_CFM[1] = CFM_mult / D_loc[1 * 6 + 1];
	m_CFM[2] = CFM_mult / D_loc[2 * 6 + 2];
	m_CFM[3] = CFM_mult / D_loc[3 * 6 + 3];
	m_CFM[4] = CFM_mult / D_loc[4 * 6 + 4];
	m_CFM[5] = CFM_mult / D_loc[5 * 6 + 5];
}

void FEMJoint::init(
	SolverBase & solver,
	NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2, NodeTranslational * node3, 
	float YoungModulus, float PoissonRatio, float plasticYield, float plasticCreep,
	float maxPlasticStrain, float dampingBeta, float regularization
	)
{
	using namespace math;

	// See the original paper, "Real-time FEM continuum mechanics coupled with articulated rigid body dynamics"
	//	by F. Benevolensky and A. Voroshilov
	// on how this joint quantities are calculated.
	// On high-level - this joint constraints 4 linear nodes, and describes reduced-order tensor in Voigt notation,
	//	thus has Jacobian matrix of size 6*12.
	//	similar to conventional fully-fixed rigid body joint, since the rigid body is represented with 2
	//	nodes (linear+rotational) and removes 6 DoFs from the pair.

	m_strainNorm = 0.0f;

	m_YoungModulus = YoungModulus;
	m_PoissonRatio = PoissonRatio;

	m_plasticYield = plasticYield;
	m_plasticCreep = plasticCreep;
	m_maxPlasticStrain = maxPlasticStrain;
	m_dampingBeta = dampingBeta;

	memset(m_lambda0, 0, 6 * sizeof(float));

	float D_loc[6 * 6];
	memset(D_loc, 0, 6 * 6 * sizeof(float));
	memset(m_B_loc, 0, 6 * 12 * sizeof(float));
	memset(m_E_plastic, 0, 6 * sizeof(float));

	m_node0 = node0;
	m_node1 = node1;
	m_node2 = node2;
	m_node3 = node3;

	// Calculate B for Tetra [Old Pos]
	const Vec3 & n0pos = m_node0->m_pos;
	const Vec3 & n1pos = m_node1->m_pos;
	const Vec3 & n2pos = m_node2->m_pos;
	const Vec3 & n3pos = m_node3->m_pos;

	float det =	(n1pos.x - n0pos.x) * ((n2pos.y - n0pos.y) * (n3pos.z - n0pos.z) - (n3pos.y - n0pos.y) * (n2pos.z - n0pos.z)) +
				(n2pos.x - n0pos.x) * ((n3pos.y - n0pos.y) * (n1pos.z - n0pos.z) - (n1pos.y - n0pos.y) * (n3pos.z - n0pos.z)) +
				(n3pos.x - n0pos.x) * ((n1pos.y - n0pos.y) * (n2pos.z - n0pos.z) - (n2pos.y - n0pos.y) * (n1pos.z - n0pos.z));

	float invDet = 1.0f / det;

	float dNBdx = invDet * (  (n2pos.y - n0pos.y) * (n3pos.z - n0pos.z)
							- (n3pos.y - n0pos.y) * (n2pos.z - n0pos.z) );
	float dNBdy = invDet * (  (n3pos.x - n0pos.x) * (n2pos.z - n0pos.z)
							- (n2pos.x - n0pos.x) * (n3pos.z - n0pos.z) );
	float dNBdz = invDet * (  (n2pos.x - n0pos.x) * (n3pos.y - n0pos.y)
							- (n3pos.x - n0pos.x) * (n2pos.y - n0pos.y) );

	float dNCdx = invDet * (  (n3pos.y - n0pos.y) * (n1pos.z - n0pos.z)
							- (n1pos.y - n0pos.y) * (n3pos.z - n0pos.z) );
	float dNCdy = invDet * (  (n1pos.x - n0pos.x) * (n3pos.z - n0pos.z)
							- (n3pos.x - n0pos.x) * (n1pos.z - n0pos.z) );
	float dNCdz = invDet * (  (n3pos.x - n0pos.x) * (n1pos.y - n0pos.y)
							- (n1pos.x - n0pos.x) * (n3pos.y - n0pos.y) );

	float dNDdx = invDet * (  (n1pos.y - n0pos.y) * (n2pos.z - n0pos.z)
							- (n2pos.y - n0pos.y) * (n1pos.z - n0pos.z) );
	float dNDdy = invDet * (  (n2pos.x - n0pos.x) * (n1pos.z - n0pos.z)
							- (n1pos.x - n0pos.x) * (n2pos.z - n0pos.z) );
	float dNDdz = invDet * (  (n1pos.x - n0pos.x) * (n2pos.y - n0pos.y)
							- (n2pos.x - n0pos.x) * (n1pos.y - n0pos.y) );

	float dNAdx = -(dNBdx + dNCdx + dNDdx);
	float dNAdy = -(dNBdy + dNCdy + dNDdy);
	float dNAdz = -(dNBdz + dNCdz + dNDdz);

	float sqrt_2 = 0.5f * sqrtf(2.0f);
	float sqrt_3 = sqrtf(3.0f) / 3.0f;
	float sqrt_6 = sqrtf(6.0f) / 6.0f;

	// B1
	m_B_loc[0*12+ 0] =  sqrt_2 * dNAdx;
	m_B_loc[0*12+ 2] = -sqrt_2 * dNAdz;

	m_B_loc[1*12+ 0] = sqrt_3 * dNAdx;
	m_B_loc[1*12+ 1] = sqrt_3 * dNAdy;
	m_B_loc[1*12+ 2] = sqrt_3 * dNAdz;

	m_B_loc[2*12+ 0] = sqrt_6 * dNAdx;
	m_B_loc[2*12+ 1] = -2.0f * sqrt_6 * dNAdy;
	m_B_loc[2*12+ 2] = sqrt_6 * dNAdz;

	m_B_loc[3*12+ 1] = dNAdz;
	m_B_loc[3*12+ 2] = dNAdy;
	m_B_loc[4*12+ 0] = dNAdz;
	m_B_loc[4*12+ 2] = dNAdx;
	m_B_loc[5*12+ 0] = dNAdy;
	m_B_loc[5*12+ 1] = dNAdx;

	// B2
	m_B_loc[0*12+ 3] =  sqrt_2 * dNBdx;
	m_B_loc[0*12+ 5] = -sqrt_2 * dNBdz;

	m_B_loc[1*12+ 3] = sqrt_3 * dNBdx;
	m_B_loc[1*12+ 4] = sqrt_3 * dNBdy;
	m_B_loc[1*12+ 5] = sqrt_3 * dNBdz;

	m_B_loc[2*12+ 3] = sqrt_6 * dNBdx;
	m_B_loc[2*12+ 4] = -2.0f * sqrt_6 * dNBdy;
	m_B_loc[2*12+ 5] = sqrt_6 * dNBdz;

	m_B_loc[3*12+ 4] = dNBdz;
	m_B_loc[3*12+ 5] = dNBdy;
	m_B_loc[4*12+ 3] = dNBdz;
	m_B_loc[4*12+ 5] = dNBdx;
	m_B_loc[5*12+ 3] = dNBdy;
	m_B_loc[5*12+ 4] = dNBdx;

	// B3
	m_B_loc[0*12+ 6] =  sqrt_2 * dNCdx;
	m_B_loc[0*12+ 8] = -sqrt_2 * dNCdz;

	m_B_loc[1*12+ 6] = sqrt_3 * dNCdx;
	m_B_loc[1*12+ 7] = sqrt_3 * dNCdy;
	m_B_loc[1*12+ 8] = sqrt_3 * dNCdz;

	m_B_loc[2*12+ 6] = sqrt_6 * dNCdx;
	m_B_loc[2*12+ 7] = -2.0f * sqrt_6 * dNCdy;
	m_B_loc[2*12+ 8] = sqrt_6 * dNCdz;

	m_B_loc[3*12+ 7] = dNCdz;
	m_B_loc[3*12+ 8] = dNCdy;
	m_B_loc[4*12+ 6] = dNCdz;
	m_B_loc[4*12+ 8] = dNCdx;
	m_B_loc[5*12+ 6] = dNCdy;
	m_B_loc[5*12+ 7] = dNCdx;

	// B4
	m_B_loc[0*12+ 9] =  sqrt_2 * dNDdx;
	m_B_loc[0*12+11] = -sqrt_2 * dNDdz;

	m_B_loc[1*12+ 9] = sqrt_3 * dNDdx;
	m_B_loc[1*12+10] = sqrt_3 * dNDdy;
	m_B_loc[1*12+11] = sqrt_3 * dNDdz;

	m_B_loc[2*12+ 9] = sqrt_6 * dNDdx;
	m_B_loc[2*12+10] = -2.0f * sqrt_6 * dNDdy;
	m_B_loc[2*12+11] = sqrt_6 * dNDdz;

	m_B_loc[3*12+10] = dNDdz;
	m_B_loc[3*12+11] = dNDdy;
	m_B_loc[4*12+ 9] = dNDdz;
	m_B_loc[4*12+11] = dNDdx;
	m_B_loc[5*12+ 9] = dNDdy;
	m_B_loc[5*12+10] = dNDdx;

	// D
	float sigma1 = m_PoissonRatio / (1 - m_PoissonRatio);
	float premult = m_YoungModulus / ((1 + m_PoissonRatio) * (1 - sigma1));
	float coef1 = premult * sigma1;
	float coef2 = 0.5f * premult * (1 - sigma1);

	D_loc[0 * 6 + 0] = premult * (1.0f - sigma1);
	D_loc[1 * 6 + 1] = premult * (1.0f + 2.0f * sigma1);
	D_loc[2 * 6 + 2] = premult * (1.0f - sigma1);

	D_loc[3 * 6 + 3] = coef2;
	D_loc[4 * 6 + 4] = coef2;
	D_loc[5 * 6 + 5] = coef2;

	m_initialFEVolume = (1.0f / 6.0f) * (n3pos - n0pos).dot((n2pos - n0pos).cross(n1pos - n0pos));

	float CFM_mult = 1.0f / m_initialFEVolume;

	m_CFM[0] = CFM_mult / D_loc[0 * 6 + 0];
	m_CFM[1] = CFM_mult / D_loc[1 * 6 + 1];
	m_CFM[2] = CFM_mult / D_loc[2 * 6 + 2];
	m_CFM[3] = CFM_mult / D_loc[3 * 6 + 3];
	m_CFM[4] = CFM_mult / D_loc[4 * 6 + 4];
	m_CFM[5] = CFM_mult / D_loc[5 * 6 + 5];

	m_regularization = regularization;

#if (WARPING_POLAR_DECOMPOSITION == 1)

	m_mp0_inv.t[0][0] = n1pos.x - n0pos.x; 
	m_mp0_inv.t[1][0] = n1pos.y - n0pos.y; 
	m_mp0_inv.t[2][0] = n1pos.z - n0pos.z; 

	m_mp0_inv.t[0][1] = n2pos.x - n0pos.x; 
	m_mp0_inv.t[1][1] = n2pos.y - n0pos.y; 
	m_mp0_inv.t[2][1] = n2pos.z - n0pos.z; 

	m_mp0_inv.t[0][2] = n3pos.x - n0pos.x; 
	m_mp0_inv.t[1][2] = n3pos.y - n0pos.y; 
	m_mp0_inv.t[2][2] = n3pos.z - n0pos.z; 

	m_mp0_inv.invert();

#else

	// Calculating matrix N
	Vec3 edge0 = n1pos - n0pos;

	Vec3 N1 = edge0 + (n2pos - n0pos) + (n3pos - n0pos);
	N1.normalize();

	Vec3 N2 = N1.cross(edge0);
	N2.normalize();

	Vec3 N3 = N2.cross(N1);
	N3.normalize();

	m_N.setBasis0(N1);
	m_N.setBasis1(N2);
	m_N.setBasis2(N3);

#endif

	for (unsigned int i = 0; i < 6; ++i)
	{
		m_Jp0[i] = 
			n0pos.x * m_B_loc[i * 12 + 0] +
			n0pos.y * m_B_loc[i * 12 + 1] +
			n0pos.z * m_B_loc[i * 12 + 2] +

			n1pos.x * m_B_loc[i * 12 + 3] +
			n1pos.y * m_B_loc[i * 12 + 4] +
			n1pos.z * m_B_loc[i * 12 + 5] +

			n2pos.x * m_B_loc[i * 12 + 6] +
			n2pos.y * m_B_loc[i * 12 + 7] +
			n2pos.z * m_B_loc[i * 12 + 8] +

			n3pos.x * m_B_loc[i * 12 + 9] +
			n3pos.y * m_B_loc[i * 12 + 10] +
			n3pos.z * m_B_loc[i * 12 + 11];

	}
}

void FEMJoint::onStepStart(float dt)
{
	using namespace math;

	if (dt == 0.0f)
		return;

	const Vec3 & n0pos = m_node0->m_pos;
	const Vec3 & n1pos = m_node1->m_pos;
	const Vec3 & n2pos = m_node2->m_pos;
	const Vec3 & n3pos = m_node3->m_pos;

	Vec3 v1, v2, v3;
	v1 = n3pos - n0pos;
	v2 = n2pos - n0pos;
	v3 = n1pos - n0pos;

#if (WARPING_POLAR_DECOMPOSITION == 1)

	Mat33 mpt, a,  p, u;

	mpt.setBasis0(n1pos - n0pos);
	mpt.setBasis1(n2pos - n0pos);
	mpt.setBasis2(n3pos - n0pos);

	a = mpt * m_mp0_inv;
	a.polarDecompose(p, u);

#else

	// Calculating new matrix N

	Mat33 N_new;

	Vec3 edge0 = n1pos - n0pos;

	Vec3 N1;
	N1 = edge0 + (n2pos - n0pos) + (n3pos - n0pos);
	N1.normalize();

	Vec3 N2 = N1.cross(edge0);
	N2.normalize();

	Vec3 N3 = N2.cross(N1);
	N3.normalize();

	N_new.setBasis0(N1);
	N_new.setBasis1(N2);
	N_new.setBasis2(N3);

	Mat33 u = N_new * m_N.getTransposed();

#endif	

	if (u.det() < 0.0f)
		u = u * -1.0f;

	unsigned int Jrow0, Jrow1, Jrow2;
	unsigned int Brow0, Brow1, Brow2;

#define TRANSFER_B_SUBMATRIX(i, j, t0, t1, t2) \
			Jrow0 = (i * 3 + 0);\
			Jrow1 = (i * 3 + 1);\
			Jrow2 = (i * 3 + 2);\
			\
			Brow0 = (i * 3 + 0) * 12 + j * 3;\
			Brow1 = (i * 3 + 1) * 12 + j * 3;\
			Brow2 = (i * 3 + 2) * 12 + j * 3;\
			\
			m_precompute.J[t0][Jrow0] =	m_B_loc[Brow0 + 0] * u._00 + m_B_loc[Brow0 + 1] * u._01 + \
										m_B_loc[Brow0 + 2] * u._02;\
			\
			m_precompute.J[t1][Jrow0] =	m_B_loc[Brow0 + 0] * u._10 + m_B_loc[Brow0 + 1] * u._11 + \
										m_B_loc[Brow0 + 2] * u._12;\
			\
			m_precompute.J[t2][Jrow0] =	m_B_loc[Brow0 + 0] * u._20 + m_B_loc[Brow0 + 1] * u._21 + \
										m_B_loc[Brow0 + 2] * u._22;\
			\
			m_precompute.J[t0][Jrow1] =	m_B_loc[Brow1 + 0] * u._00 + m_B_loc[Brow1 + 1] * u._01 + \
										m_B_loc[Brow1 + 2] * u._02;\
			\
			m_precompute.J[t1][Jrow1] =	m_B_loc[Brow1 + 0] * u._10 + m_B_loc[Brow1 + 1] * u._11 + \
										m_B_loc[Brow1 + 2] * u._12;\
			\
			m_precompute.J[t2][Jrow1] =	m_B_loc[Brow1 + 0] * u._20 + m_B_loc[Brow1 + 1] * u._21 + \
										m_B_loc[Brow1 + 2] * u._22;\
			\
			m_precompute.J[t0][Jrow2] =	m_B_loc[Brow2 + 0] * u._00 + m_B_loc[Brow2 + 1] * u._01 + \
										m_B_loc[Brow2 + 2] * u._02;\
			\
			m_precompute.J[t1][Jrow2] =	m_B_loc[Brow2 + 0] * u._10 + m_B_loc[Brow2 + 1] * u._11 + \
										m_B_loc[Brow2 + 2] * u._12;\
			\
			m_precompute.J[t2][Jrow2] =	m_B_loc[Brow2 + 0] * u._20 + m_B_loc[Brow2 + 1] * u._21 + \
										m_B_loc[Brow2 + 2] * u._22;	

	TRANSFER_B_SUBMATRIX(0, 0,  0,  1,  2);
	TRANSFER_B_SUBMATRIX(0, 1,  3,  4,  5);
	TRANSFER_B_SUBMATRIX(0, 2,  6,  7,  8);
	TRANSFER_B_SUBMATRIX(0, 3,  9, 10, 11);
	TRANSFER_B_SUBMATRIX(1, 0,  0,  1,  2);
	TRANSFER_B_SUBMATRIX(1, 1,  3,  4,  5);
	TRANSFER_B_SUBMATRIX(1, 2,  6,  7,  8);
	TRANSFER_B_SUBMATRIX(1, 3,  9, 10, 11);

#undef TRANSFER_B_SUBMATRIX

	// Calculate Elastic Strain = Total Strain - Plastic Strain [ Bq - e_plastic ]
	for (int i = 0; i < 6; ++i)
	{
		m_precompute.e_elastic[i] = (
					n0pos.x * m_precompute.J[ 0][i] +
					n0pos.y * m_precompute.J[ 1][i] +
					n0pos.z * m_precompute.J[ 2][i] +

					n1pos.x * m_precompute.J[ 3][i] +
					n1pos.y * m_precompute.J[ 4][i] +
					n1pos.z * m_precompute.J[ 5][i] +

					n2pos.x * m_precompute.J[ 6][i] +
					n2pos.y * m_precompute.J[ 7][i] +
					n2pos.z * m_precompute.J[ 8][i] +

					n3pos.x * m_precompute.J[ 9][i] +
					n3pos.y * m_precompute.J[10][i] +
					n3pos.z * m_precompute.J[11][i]
			- m_Jp0[i] - m_E_plastic[i]);
	}

	float * e_elastic = m_precompute.e_elastic;

	float el_norm = sqrtf(
		e_elastic[0] * e_elastic[0] + e_elastic[1] * e_elastic[1] + e_elastic[2] * e_elastic[2] +
		e_elastic[3] * e_elastic[3] + e_elastic[4] * e_elastic[4] + e_elastic[5] * e_elastic[5]
		);

	m_strainNorm = el_norm;

	float idt = 1.0f / dt;

	// plasticCreep is in [0; 1/dt]
	float Creep = m_plasticCreep * idt;
	if (el_norm > m_plasticYield)
	{
		for (int i = 0; i < 6; ++i)
			m_E_plastic[i] += dt * Creep * e_elastic[i];
	}

	float pl_norm = sqrtf(
		m_E_plastic[0] * m_E_plastic[0] + m_E_plastic[1] * m_E_plastic[1] + m_E_plastic[2] * m_E_plastic[2] +
		m_E_plastic[3] * m_E_plastic[3] + m_E_plastic[4] * m_E_plastic[4] + m_E_plastic[5] * m_E_plastic[5]
		);

	if (pl_norm > m_maxPlasticStrain)
	{
		for (int i = 0; i < 6; ++i)
		{
			m_E_plastic[i] *= m_maxPlasticStrain / pl_norm;
		}
	}
}

void FEMJoint::updateCopy(float dt, SolverBase & solver, float initialGuessCoeff)
{
	using namespace math;

	if (dt == 0.0f)
		return;

	// See "init()" function of this joint for brief description and reference

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	m_fetchIndices[0] = m_startIdx;
	m_fetchIndices[1] = m_startIdx + 1;
	m_fetchIndices[2] = m_startIdx + 2;
	m_fetchIndices[3] = m_startIdx + 3;
	m_fetchIndices[4] = m_startIdx + 4;
	m_fetchIndices[5] = m_startIdx + 5;

	float idt = 1.0f / dt;

	float gamma = 1.0f / (1.0f + idt * m_dampingBeta);

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < 6; ++i)
		*(ptr_m_CFM++) = (idt * gamma * m_CFM[i] + m_regularization);

	const Vec3 & n0pos = m_node0->m_pos;
	const Vec3 & n1pos = m_node1->m_pos;
	const Vec3 & n2pos = m_node2->m_pos;
	const Vec3 & n3pos = m_node3->m_pos;

	// Jacobian pointers setup
	float	*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
			*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
			*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
			*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0];

	float	*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
			*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	for (int i = 0; i < 6; ++i)
	{
		ptr_m_J_00[m_startIdx + i] = m_precompute.J[ 0][i];
		ptr_m_J_01[m_startIdx + i] = m_precompute.J[ 1][i];
		ptr_m_J_02[m_startIdx + i] = m_precompute.J[ 2][i];
		ptr_m_J_03[m_startIdx + i] = m_precompute.J[ 3][i];
		ptr_m_J_04[m_startIdx + i] = m_precompute.J[ 4][i];
		ptr_m_J_05[m_startIdx + i] = m_precompute.J[ 5][i];
		ptr_m_J_06[m_startIdx + i] = m_precompute.J[ 6][i];
		ptr_m_J_07[m_startIdx + i] = m_precompute.J[ 7][i];
		ptr_m_J_08[m_startIdx + i] = m_precompute.J[ 8][i];
		ptr_m_J_09[m_startIdx + i] = m_precompute.J[ 9][i];
		ptr_m_J_10[m_startIdx + i] = m_precompute.J[10][i];
		ptr_m_J_11[m_startIdx + i] = m_precompute.J[11][i];
	}

	// Null remaining Jacobian
	for (int i = 0; i < 6; ++i)
	{
		ptr_m_J_12[m_startIdx + i] = 0.0f;
		ptr_m_J_13[m_startIdx + i] = 0.0f;
		ptr_m_J_14[m_startIdx + i] = 0.0f;
		ptr_m_J_15[m_startIdx + i] = 0.0f;
		ptr_m_J_16[m_startIdx + i] = 0.0f;
		ptr_m_J_17[m_startIdx + i] = 0.0f;
	}

	unsigned int i;

	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	for (i = 0; i < 6; ++i)
	{
		*(ptr_m_J_rhs++) = -idt * gamma * m_precompute.e_elastic[i];
	}

	// Limits
	float liForces = FLT_MAX;

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
			*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	for (i = 0; i < 6; ++i)
	{
		*(ptr_m_Lo++) = -liForces;
		*(ptr_m_Hi++) =  liForces;
	}

	// Calculate Jointed Nodes Indices
	unsigned int	*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[m_startIdx],
					*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[m_startIdx],
					*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[m_startIdx],
					*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[m_startIdx];

	uint32_t n0idx = m_node0->m_idx;
	uint32_t n1idx = m_node1->m_idx;
	uint32_t n2idx = m_node2->m_idx;
	uint32_t n3idx = m_node3->m_idx;

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;

	// Third Triple Index
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;

	// Fourth Triple Index
	*(ptr_m_JtdNodes_03++) = n3idx;
	*(ptr_m_JtdNodes_03++) = n3idx;
	*(ptr_m_JtdNodes_03++) = n3idx;
	*(ptr_m_JtdNodes_03++) = n3idx;
	*(ptr_m_JtdNodes_03++) = n3idx;
	*(ptr_m_JtdNodes_03++) = n3idx;


	// Null out remaining indices
	unsigned int	*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[m_startIdx],
					*ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[m_startIdx];

	// Fifth Triple Index (should be empty)
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;


	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	for (unsigned int i = 0; i < 6; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = initialGuessCoeff * m_lambda0[i];

		*(ptr_m_invMassScale_00++) = 1.0f;
		*(ptr_m_invMassScale_01++) = 1.0f;
		*(ptr_m_invMassScale_02++) = 1.0f;
		*(ptr_m_invMassScale_03++) = 1.0f;
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void FEMJoint::fetchLambdas(SolverBase &solver)
{
	/*
#if (HARDWARE_SOLVER == 0)
	m_lambda0[0] = solver.m_lambda[m_startIdx    ];
	m_lambda0[1] = solver.m_lambda[m_startIdx + 1];
	m_lambda0[2] = solver.m_lambda[m_startIdx + 2];
	m_lambda0[3] = solver.m_lambda[m_startIdx + 3];
	m_lambda0[4] = solver.m_lambda[m_startIdx + 4];
	m_lambda0[5] = solver.m_lambda[m_startIdx + 5];
#else
	*/
	m_lambda0[0] = solver.m_lambda[m_fetchIndices[0]];
	m_lambda0[1] = solver.m_lambda[m_fetchIndices[1]];
	m_lambda0[2] = solver.m_lambda[m_fetchIndices[2]];
	m_lambda0[3] = solver.m_lambda[m_fetchIndices[3]];
	m_lambda0[4] = solver.m_lambda[m_fetchIndices[4]];
	m_lambda0[5] = solver.m_lambda[m_fetchIndices[5]];
}



//////////////////////////////////////////////////////////////////////////
// BallSocket_FEM
//////////////////////////////////////////////////////////////////////////

void BallSocket_FEM::init(SolverBase & solver, float ERP, float CFM, const math::Vec3 & anchorPoint, NodeTranslational * bodyL, NodeRotational * bodyR, NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2)
{
	using namespace math;

	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_bodyL = bodyL;
	m_bodyR = bodyR;
	m_node0 = node0;
	m_node1 = node1;
	m_node2 = node2;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	// FE setup

	// Calculating Barycentric for
	Vec3 v_p1p0, v_p2p0, p0;
	p0 = m_node0->m_pos;

	v_p1p0 = m_node1->m_pos - p0;
	v_p2p0 = m_node2->m_pos - p0;

	Vec3 normal = v_p1p0.cross(v_p2p0);
	normal.normalize();

	m_delta = (anchorPoint - p0).dot(normal);

	// Anchor point, projected onto the triangle comprised of node0, node1, node2
	Vec3 anchorPointProj = anchorPoint - m_delta * normal;
	computeBarycentric(m_node0->m_pos, m_node1->m_pos, m_node2->m_pos, anchorPointProj, &m_alpha, &m_beta, &m_gamma);

	if (m_bodyL != nullptr)
	{
		// Calculating Anchor Points in Local Space
		m_bodyAnchorPoint = physics::transformWorldToOrigin(m_bodyL, m_bodyR, anchorPoint);
	}
	else
	{
		m_bodyAnchorPoint = anchorPoint;
	}
}

void BallSocket_FEM::updateCopy(float dt, SolverBase & solver)
{
	using namespace math;

	// Constraint equation:
	//	c = p_B + [R_B]a_BL - (alpha * p_0 + beta * p_1 + gamma * p_2 + delta * n) = 0
	//	n = (p_1 - p_0) x (p2 - p_0) / |(p_1 - p_0) x (p2 - p_0)|
	// See the original paper, "Real-time FEM continuum mechanics coupled with articulated rigid body dynamics"
	//	by F. Benevolensky and A. Voroshilov
	// for derivation details

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
		*(ptr_m_CFM++) = m_CFM[i];

	// Jacobian pointers
	float
		*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
		*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
		*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
		*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0],
		*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
		*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx, Row1Idx, Row2Idx;

	Row0Idx = m_startIdx + 0;
	Row1Idx = m_startIdx + 1;
	Row2Idx = m_startIdx + 2;

	// Filling Jacobians for FE-nodes
	const Vec3 & n0pos = m_node0->m_pos;
	const Vec3 & n1pos = m_node1->m_pos;
	const Vec3 & n2pos = m_node2->m_pos;

	// Node position difference vectors
	//	(p1 - p0), (p2 - p0)
	Vec3 v_p1p0, v_p2p0, v_p2p1;
	v_p1p0 = n1pos - n0pos;
	v_p2p0 = n2pos - n0pos;
	v_p2p1 = n2pos - n1pos;

	// Non-normalized orthogonal vector (n_u/|n_u| = conventional unit-length normal)
	Vec3 n_unnorm = v_p1p0.cross(v_p2p0);

	// due to internal formulation, MOD = |n|
	float mod = n_unnorm.len();

	// This vector will be used to calculate the special outer product matrix:
	//	O_ij = 1/|n_u|^3 * [n_u][n_u x (pn_i - pn_j)]^T =
	//		negN_mod3 * n_unnorm_cross_diff^T
	Vec3 n_unnorm_cross_diff, negN_mod3;
	negN_mod3 = -(1.0f / (mod * mod * mod)) * n_unnorm;

	// This vector will be used to add the [skew-symmetric cross product matrix]x contribution:
	//	C_ij = 1/|n_u| * [pn_i - pn_j]x
	Vec3 crossProdMatVec;

	// J_p0 [ starts from 6 ] = -( alpha*I3 + delta*([C_21] - [O_21]) )
	crossProdMatVec = v_p2p1 / mod;
	n_unnorm_cross_diff = n_unnorm.cross(v_p2p1);

	// row1
	ptr_m_J_00[Row0Idx] = -m_delta *  negN_mod3.x * n_unnorm_cross_diff.x - m_alpha;
	ptr_m_J_01[Row0Idx] = -m_delta * (negN_mod3.x * n_unnorm_cross_diff.y - crossProdMatVec.z);
	ptr_m_J_02[Row0Idx] = -m_delta * (negN_mod3.x * n_unnorm_cross_diff.z + crossProdMatVec.y);

	// row2
	ptr_m_J_00[Row1Idx] = -m_delta * (negN_mod3.y * n_unnorm_cross_diff.x + crossProdMatVec.z);
	ptr_m_J_01[Row1Idx] = -m_delta *  negN_mod3.y * n_unnorm_cross_diff.y - m_alpha;
	ptr_m_J_02[Row1Idx] = -m_delta * (negN_mod3.y * n_unnorm_cross_diff.z - crossProdMatVec.x);

	// row3
	ptr_m_J_00[Row2Idx] = -m_delta * (negN_mod3.z * n_unnorm_cross_diff.x - crossProdMatVec.y);
	ptr_m_J_01[Row2Idx] = -m_delta * (negN_mod3.z * n_unnorm_cross_diff.y + crossProdMatVec.x);
	ptr_m_J_02[Row2Idx] = -m_delta *  negN_mod3.z * n_unnorm_cross_diff.z - m_alpha;


	// J_p1 [ starts from 9 ] = -( beta*I3 + delta*([C_02] - [O_02]) )
	crossProdMatVec = -v_p2p0 / mod;
	n_unnorm_cross_diff = n_unnorm.cross(-v_p2p0);

	// row1
	ptr_m_J_03[Row0Idx] = -m_delta *  negN_mod3.x * n_unnorm_cross_diff.x - m_beta;
	ptr_m_J_04[Row0Idx] = -m_delta * (negN_mod3.x * n_unnorm_cross_diff.y - crossProdMatVec.z);
	ptr_m_J_05[Row0Idx] = -m_delta * (negN_mod3.x * n_unnorm_cross_diff.z + crossProdMatVec.y);

	// row2
	ptr_m_J_03[Row1Idx] = -m_delta * (negN_mod3.y * n_unnorm_cross_diff.x + crossProdMatVec.z);
	ptr_m_J_04[Row1Idx] = -m_delta *  negN_mod3.y * n_unnorm_cross_diff.y - m_beta;
	ptr_m_J_05[Row1Idx] = -m_delta * (negN_mod3.y * n_unnorm_cross_diff.z - crossProdMatVec.x);

	// row3
	ptr_m_J_03[Row2Idx] = -m_delta * (negN_mod3.z * n_unnorm_cross_diff.x - crossProdMatVec.y);
	ptr_m_J_04[Row2Idx] = -m_delta * (negN_mod3.z * n_unnorm_cross_diff.y + crossProdMatVec.x);
	ptr_m_J_05[Row2Idx] = -m_delta *  negN_mod3.z * n_unnorm_cross_diff.z - m_beta;

	// J_p2 [ starts from 12 ] = -( gamma*I3 + delta*([C_21] - [O_21]) )
	crossProdMatVec = v_p1p0 / mod;
	n_unnorm_cross_diff = n_unnorm.cross(v_p1p0);

	// row1
	ptr_m_J_06[Row0Idx] = -m_delta *  negN_mod3.x * n_unnorm_cross_diff.x - m_gamma;
	ptr_m_J_07[Row0Idx] = -m_delta * (negN_mod3.x * n_unnorm_cross_diff.y - crossProdMatVec.z);
	ptr_m_J_08[Row0Idx] = -m_delta * (negN_mod3.x * n_unnorm_cross_diff.z + crossProdMatVec.y);

	// row2
	ptr_m_J_06[Row1Idx] = -m_delta * (negN_mod3.y * n_unnorm_cross_diff.x + crossProdMatVec.z);
	ptr_m_J_07[Row1Idx] = -m_delta *  negN_mod3.y * n_unnorm_cross_diff.y - m_gamma;
	ptr_m_J_08[Row1Idx] = -m_delta * (negN_mod3.y * n_unnorm_cross_diff.z - crossProdMatVec.x);

	// row3
	ptr_m_J_06[Row2Idx] = -m_delta * (negN_mod3.z * n_unnorm_cross_diff.x - crossProdMatVec.y);
	ptr_m_J_07[Row2Idx] = -m_delta * (negN_mod3.z * n_unnorm_cross_diff.y + crossProdMatVec.x);
	ptr_m_J_08[Row2Idx] = -m_delta *  negN_mod3.z * n_unnorm_cross_diff.z - m_gamma;

	// Filling Jacobians for rigid body (if World-attachment not used)
	Vec3 wsAnchorPoint;

	if (m_bodyL != nullptr)
	{
		// Linear Body1
		ptr_m_J_09[Row0Idx] =  1.0f;
		ptr_m_J_10[Row0Idx] =  0.0f;
		ptr_m_J_11[Row0Idx] =  0.0f;

		ptr_m_J_09[Row1Idx] =  0.0f;
		ptr_m_J_10[Row1Idx] =  1.0f;
		ptr_m_J_11[Row1Idx] =  0.0f;

		ptr_m_J_09[Row2Idx] =  0.0f;
		ptr_m_J_10[Row2Idx] =  0.0f;
		ptr_m_J_11[Row2Idx] =  1.0f;

		// Rotational Body1
		wsAnchorPoint = physics::transformOriginToWorld(m_bodyL, m_bodyR, m_bodyAnchorPoint);

		Vec3 bodyOriginPos = physics::transformOriginToWorld(m_bodyL, m_bodyR, Vec3C());
		Vec3 wsAnchorLever = wsAnchorPoint - bodyOriginPos;

		// -CrossProdMatrix(b1rot * b1anchor_ls)
		ptr_m_J_12[Row0Idx] = 0.0f;
		ptr_m_J_13[Row0Idx] =  wsAnchorLever.z;
		ptr_m_J_14[Row0Idx] = -wsAnchorLever.y;
		ptr_m_J_12[Row1Idx] = -wsAnchorLever.z;
		ptr_m_J_13[Row1Idx] = 0.0f;
		ptr_m_J_14[Row1Idx] =  wsAnchorLever.x;
		ptr_m_J_12[Row2Idx] =  wsAnchorLever.y;
		ptr_m_J_13[Row2Idx] = -wsAnchorLever.x;
		ptr_m_J_14[Row2Idx] = 0.0f;
	}
	else
	{
		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_09[Row1Idx] = 0.0f;
		ptr_m_J_09[Row2Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_10[Row2Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = 0.0f;
		ptr_m_J_11[Row2Idx] = 0.0f;
		ptr_m_J_12[Row0Idx] = 0.0f;
		ptr_m_J_12[Row1Idx] = 0.0f;
		ptr_m_J_12[Row2Idx] = 0.0f;
		ptr_m_J_13[Row0Idx] = 0.0f;
		ptr_m_J_13[Row1Idx] = 0.0f;
		ptr_m_J_13[Row2Idx] = 0.0f;
		ptr_m_J_14[Row0Idx] = 0.0f;
		ptr_m_J_14[Row1Idx] = 0.0f;
		ptr_m_J_14[Row2Idx] = 0.0f;

		wsAnchorPoint = m_bodyAnchorPoint;
	}


	// Clear the rest of the Jacobi
	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_15[Row1Idx] = 0.0f;
	ptr_m_J_15[Row2Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_16[Row1Idx] = 0.0f;
	ptr_m_J_16[Row2Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;
	ptr_m_J_17[Row1Idx] = 0.0f;
	ptr_m_J_17[Row2Idx] = 0.0f;

	// World-space coordinates of socket, attached to FE (where rigid body's ball should be)
	Vec3 FE_Socket = m_alpha * n0pos + m_beta * n1pos + m_gamma * n2pos + (m_delta / mod) * n_unnorm;

	// Baumgarte Stabilization
	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	*(ptr_m_J_rhs++) = idt * m_ERP * (FE_Socket.x - wsAnchorPoint.x);
	*(ptr_m_J_rhs++) = idt * m_ERP * (FE_Socket.y - wsAnchorPoint.y);
	*(ptr_m_J_rhs++) = idt * m_ERP * (FE_Socket.z - wsAnchorPoint.z);

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
		*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;


	// Calculate Jointed Nodes Indices
	unsigned int
		*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
		*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
		*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
		*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx],
		*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx];

	uint32_t n0idx = m_node0->m_idx;
	uint32_t n1idx = m_node1->m_idx;
	uint32_t n2idx = m_node2->m_idx;

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;

	// Third Triple Index
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;

	if (m_bodyL != nullptr)
	{
		// Fourth Triple Index
		*(ptr_m_JtdNodes_03++) = m_bodyL->m_idx;
		*(ptr_m_JtdNodes_03++) = m_bodyL->m_idx;
		*(ptr_m_JtdNodes_03++) = m_bodyL->m_idx;

		// Fifth Triple Index
		*(ptr_m_JtdNodes_04++) = m_bodyR->m_idx;
		*(ptr_m_JtdNodes_04++) = m_bodyR->m_idx;
		*(ptr_m_JtdNodes_04++) = m_bodyR->m_idx;
	} else
	{
		// No triple -- "world" anchor

		// Fourth Triple Index
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;

		// Fifth Triple Index
		*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	}

	unsigned int *ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;

	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	for (unsigned int i = 0; i < 3; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = m_lambda0[i];

		*(ptr_m_invMassScale_00++) = 1.0f;
		*(ptr_m_invMassScale_01++) = 1.0f;
		*(ptr_m_invMassScale_02++) = 1.0f;
		*(ptr_m_invMassScale_03++) = 1.0f;
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void BallSocket_FEM::fetchLambdas(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
	m_lambda0[1] = solver.m_lambda[m_startIdx+1];
	m_lambda0[2] = solver.m_lambda[m_startIdx+2];
}


//////////////////////////////////////////////////////////////////////////
// BallSocket_FEM Simple (no shift along normal)
//////////////////////////////////////////////////////////////////////////

void BallSocket_FEM_Simple::init(SolverBase & solver, float ERP, float CFM, const math::Vec3 & anchorPoint, NodeTranslational * bodyL, NodeRotational * bodyR, NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2)
{
	using namespace math;

	memset(m_lambda0, 0, c_numRows * sizeof(float));

	m_bodyL = bodyL;
	m_bodyR = bodyR;
	m_node0 = node0;
	m_node1 = node1;
	m_node2 = node2;

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP = ERP;

	// FE setup

	// Calculating Barycentric for
	Vec3 v_p1p0, v_p2p0, p0;
	p0 = m_node0->m_pos;

	v_p1p0 = m_node1->m_pos - p0;
	v_p2p0 = m_node2->m_pos - p0;

	Vec3 normal = v_p1p0.cross(v_p2p0);
	normal.normalize();

	float delta = (anchorPoint - p0).dot(normal);
	// Project the anchor point onto the triangle, in case it was accidentally set off triangle
	Vec3 anchorPointProj = anchorPoint - delta * normal;

	computeBarycentric(m_node0->m_pos, m_node1->m_pos, m_node2->m_pos, anchorPointProj, &m_alpha, &m_beta, &m_gamma);

	if (m_bodyL != nullptr)
	{
		// Calculating Anchor Points in Local Space
		m_bodyAnchorPoint = physics::transformWorldToOrigin(m_bodyL, m_bodyR, anchorPointProj);
	}
	else
	{
		m_bodyAnchorPoint = anchorPointProj;
	}
}

void BallSocket_FEM_Simple::updateCopy(float dt, SolverBase & solver)
{
	using namespace math;

	// Constraint equation:
	//	c = p_B + [R_B]a_BL - (alpha * p_0 + beta * p_1 + gamma * p_2) = 0
	// See the original paper, "Real-time FEM continuum mechanics coupled with articulated rigid body dynamics"
	//	by F. Benevolensky and A. Voroshilov
	// for derivation details

	// Receive Number for this joint && Update Solver information about number of joints
	m_startIdx = solver.m_numJoints;
	solver.m_numJoints += c_numRows;

	float idt = 1.0f / dt;

	// Transfer CFM into solver
	float *ptr_m_CFM = &solver.m_CFM[m_startIdx];

	for (int i = 0; i < c_numRows; ++i)
		*(ptr_m_CFM++) = m_CFM[i];

	// Jacobian pointers
	float
		*ptr_m_J_00 = &solver.m_J_00[0], *ptr_m_J_01 = &solver.m_J_01[0], *ptr_m_J_02 = &solver.m_J_02[0],
		*ptr_m_J_03 = &solver.m_J_03[0], *ptr_m_J_04 = &solver.m_J_04[0], *ptr_m_J_05 = &solver.m_J_05[0],
		*ptr_m_J_06 = &solver.m_J_06[0], *ptr_m_J_07 = &solver.m_J_07[0], *ptr_m_J_08 = &solver.m_J_08[0],
		*ptr_m_J_09 = &solver.m_J_09[0], *ptr_m_J_10 = &solver.m_J_10[0], *ptr_m_J_11 = &solver.m_J_11[0],
		*ptr_m_J_12 = &solver.m_J_12[0], *ptr_m_J_13 = &solver.m_J_13[0], *ptr_m_J_14 = &solver.m_J_14[0],
		*ptr_m_J_15 = &solver.m_J_15[0], *ptr_m_J_16 = &solver.m_J_16[0], *ptr_m_J_17 = &solver.m_J_17[0];

	unsigned int Row0Idx, Row1Idx, Row2Idx;

	Row0Idx = m_startIdx + 0;
	Row1Idx = m_startIdx + 1;
	Row2Idx = m_startIdx + 2;

	// Filling Jacobians for FE-nodes
	const Vec3 & n0pos = m_node0->m_pos;
	const Vec3 & n1pos = m_node1->m_pos;
	const Vec3 & n2pos = m_node2->m_pos;

	// J_p0 [ starts from 6 ] = -( alpha*I3 )
	//////////////////////////////////////////////////////////////////////////////////////

	// row1
	ptr_m_J_00[Row0Idx] = -m_alpha;
	ptr_m_J_01[Row0Idx] = 0.0f;
	ptr_m_J_02[Row0Idx] = 0.0f;

	// row2
	ptr_m_J_00[Row1Idx] = 0.0f;
	ptr_m_J_01[Row1Idx] = -m_alpha;
	ptr_m_J_02[Row1Idx] = 0.0f;

	// row3
	ptr_m_J_00[Row2Idx] = 0.0f;
	ptr_m_J_01[Row2Idx] = 0.0f;
	ptr_m_J_02[Row2Idx] = -m_alpha;


	// J_p1 [ starts from 9 ] = -( beta*I3 )
	//////////////////////////////////////////////////////////////////////////////////////

	// row1
	ptr_m_J_03[Row0Idx] = -m_beta;
	ptr_m_J_04[Row0Idx] = 0.0f;
	ptr_m_J_05[Row0Idx] = 0.0f;

	// row2
	ptr_m_J_03[Row1Idx] = 0.0f;
	ptr_m_J_04[Row1Idx] = -m_beta;
	ptr_m_J_05[Row1Idx] = 0.0f;

	// row3
	ptr_m_J_03[Row2Idx] = 0.0f;
	ptr_m_J_04[Row2Idx] = 0.0f;
	ptr_m_J_05[Row2Idx] = -m_beta;

	// J_p2 [ starts from 12 ] = -( gamma*I3 )
	//////////////////////////////////////////////////////////////////////////////////////

	// row1
	ptr_m_J_06[Row0Idx] = -m_gamma;
	ptr_m_J_07[Row0Idx] = 0.0f;
	ptr_m_J_08[Row0Idx] = 0.0f;

	// row2
	ptr_m_J_06[Row1Idx] = 0.0f;
	ptr_m_J_07[Row1Idx] = -m_gamma;
	ptr_m_J_08[Row1Idx] = 0.0f;

	// row3
	ptr_m_J_06[Row2Idx] = 0.0f;
	ptr_m_J_07[Row2Idx] = 0.0f;
	ptr_m_J_08[Row2Idx] = -m_gamma;

	// Filling Jacobians for rigid body (if World-attachment not used)
	Vec3 wsAnchorPoint;

	if (m_bodyL != nullptr)
	{
		// Linear Body1
		ptr_m_J_09[Row0Idx] =  1.0f;
		ptr_m_J_10[Row0Idx] =  0.0f;
		ptr_m_J_11[Row0Idx] =  0.0f;

		ptr_m_J_09[Row1Idx] =  0.0f;
		ptr_m_J_10[Row1Idx] =  1.0f;
		ptr_m_J_11[Row1Idx] =  0.0f;

		ptr_m_J_09[Row2Idx] =  0.0f;
		ptr_m_J_10[Row2Idx] =  0.0f;
		ptr_m_J_11[Row2Idx] =  1.0f;

		// Rotational Body1
		wsAnchorPoint = physics::transformOriginToWorld(m_bodyL, m_bodyR, m_bodyAnchorPoint);

		Vec3 bodyOriginPos = physics::transformOriginToWorld(m_bodyL, m_bodyR, Vec3C());
		Vec3 wsAnchorLever = wsAnchorPoint - bodyOriginPos;

		// -CrossProdMatrix(b1rot * b1anchor_ls)
		ptr_m_J_12[Row0Idx] = 0.0f;
		ptr_m_J_13[Row0Idx] =  wsAnchorLever.z;
		ptr_m_J_14[Row0Idx] = -wsAnchorLever.y;
		ptr_m_J_12[Row1Idx] = -wsAnchorLever.z;
		ptr_m_J_13[Row1Idx] = 0.0f;
		ptr_m_J_14[Row1Idx] =  wsAnchorLever.x;
		ptr_m_J_12[Row2Idx] =  wsAnchorLever.y;
		ptr_m_J_13[Row2Idx] = -wsAnchorLever.x;
		ptr_m_J_14[Row2Idx] = 0.0f;
	}
	else
	{
		ptr_m_J_09[Row0Idx] = 0.0f;
		ptr_m_J_09[Row1Idx] = 0.0f;
		ptr_m_J_09[Row2Idx] = 0.0f;
		ptr_m_J_10[Row0Idx] = 0.0f;
		ptr_m_J_10[Row1Idx] = 0.0f;
		ptr_m_J_10[Row2Idx] = 0.0f;
		ptr_m_J_11[Row0Idx] = 0.0f;
		ptr_m_J_11[Row1Idx] = 0.0f;
		ptr_m_J_11[Row2Idx] = 0.0f;
		ptr_m_J_12[Row0Idx] = 0.0f;
		ptr_m_J_12[Row1Idx] = 0.0f;
		ptr_m_J_12[Row2Idx] = 0.0f;
		ptr_m_J_13[Row0Idx] = 0.0f;
		ptr_m_J_13[Row1Idx] = 0.0f;
		ptr_m_J_13[Row2Idx] = 0.0f;
		ptr_m_J_14[Row0Idx] = 0.0f;
		ptr_m_J_14[Row1Idx] = 0.0f;
		ptr_m_J_14[Row2Idx] = 0.0f;

		wsAnchorPoint = m_bodyAnchorPoint;
	}


	// Clear the rest of the Jacobi
	ptr_m_J_15[Row0Idx] = 0.0f;
	ptr_m_J_15[Row1Idx] = 0.0f;
	ptr_m_J_15[Row2Idx] = 0.0f;
	ptr_m_J_16[Row0Idx] = 0.0f;
	ptr_m_J_16[Row1Idx] = 0.0f;
	ptr_m_J_16[Row2Idx] = 0.0f;
	ptr_m_J_17[Row0Idx] = 0.0f;
	ptr_m_J_17[Row1Idx] = 0.0f;
	ptr_m_J_17[Row2Idx] = 0.0f;


	// World-space coordinates of socket, attached to FE (where rigid body's ball should be)
	Vec3 FE_Socket = m_alpha * n0pos + m_beta * n1pos + m_gamma * n2pos;

	// Baumgarte Stabilization
	float *ptr_m_J_rhs = &solver.m_J_rhs[m_startIdx];

	*(ptr_m_J_rhs++) = idt * m_ERP * (FE_Socket.x - wsAnchorPoint.x);
	*(ptr_m_J_rhs++) = idt * m_ERP * (FE_Socket.y - wsAnchorPoint.y);
	*(ptr_m_J_rhs++) = idt * m_ERP * (FE_Socket.z - wsAnchorPoint.z);

	float	*ptr_m_Lo = &solver.m_Lo[m_startIdx],
		*ptr_m_Hi = &solver.m_Hi[m_startIdx];

	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;
	*(ptr_m_Lo++) = -FLT_MAX;
	*(ptr_m_Hi++) =  FLT_MAX;


	// Calculate Jointed Nodes Indices
	unsigned int
		*ptr_m_JtdNodes_00 = &solver.m_JtdNodes_00[Row0Idx],
		*ptr_m_JtdNodes_01 = &solver.m_JtdNodes_01[Row0Idx],
		*ptr_m_JtdNodes_02 = &solver.m_JtdNodes_02[Row0Idx],
		*ptr_m_JtdNodes_03 = &solver.m_JtdNodes_03[Row0Idx],
		*ptr_m_JtdNodes_04 = &solver.m_JtdNodes_04[Row0Idx];

	uint32_t n0idx = m_node0->m_idx;
	uint32_t n1idx = m_node1->m_idx;
	uint32_t n2idx = m_node2->m_idx;

	// First Triple Index
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;
	*(ptr_m_JtdNodes_00++) = n0idx;

	// Second Triple Index
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;
	*(ptr_m_JtdNodes_01++) = n1idx;

	// Third Triple Index
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;
	*(ptr_m_JtdNodes_02++) = n2idx;

	if (m_bodyL != nullptr)
	{
		// Fourth Triple Index
		*(ptr_m_JtdNodes_03++) = m_bodyL->m_idx;
		*(ptr_m_JtdNodes_03++) = m_bodyL->m_idx;
		*(ptr_m_JtdNodes_03++) = m_bodyL->m_idx;

		// Fifth Triple Index
		*(ptr_m_JtdNodes_04++) = m_bodyR->m_idx;
		*(ptr_m_JtdNodes_04++) = m_bodyR->m_idx;
		*(ptr_m_JtdNodes_04++) = m_bodyR->m_idx;
	} else
	{
		// No triple -- "world" anchor

		// Fourth Triple Index
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_03++) = SOLVER_NO_BODY;

		// Fifth Triple Index
		*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
		*(ptr_m_JtdNodes_04++) = SOLVER_NO_BODY;
	}

	unsigned int *ptr_m_JtdNodes_05 = &solver.m_JtdNodes_05[Row0Idx];

	// Sixth Triple Index (should be empty)
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;
	*(ptr_m_JtdNodes_05++) = SOLVER_NO_BODY;

	float *ptr_m_lambda = &solver.m_lambda[m_startIdx];

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	for (unsigned int i = 0; i < 3; ++i)
	{
		// Initial Guess
		*(ptr_m_lambda++) = m_lambda0[i];

		*(ptr_m_invMassScale_00++) = 1.0f;
		*(ptr_m_invMassScale_01++) = 1.0f;
		*(ptr_m_invMassScale_02++) = 1.0f;
		*(ptr_m_invMassScale_03++) = 1.0f;
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}
}

void BallSocket_FEM_Simple::fetchLambdas(SolverBase & solver)
{
	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
	m_lambda0[1] = solver.m_lambda[m_startIdx+1];
	m_lambda0[2] = solver.m_lambda[m_startIdx+2];
}

//////////////////////////////////////////////////////////////////////////
// TriNodeContact
//////////////////////////////////////////////////////////////////////////
void TriNodeContact::init(float ERP, float CFM, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2)
{
	m_cache = nullptr;

	memset(m_lambda0, 0, c_numRowsMax * sizeof(float));

	m_body0L = body0L;
	m_body0R = body0R;
	m_node0 = node0;
	m_node1 = node1;
	m_node2 = node2;

	m_skinWidth = m_min(m_min(m_min(m_body0L->m_skinWidth, node0->m_skinWidth), node1->m_skinWidth), node2->m_skinWidth);
	// TODO: take into account all thee nodes
	m_restitutionCoeff = 0.5f * (m_body0L->m_restitutionCoeff + m_node0->m_restitutionCoeff);
	m_frictionCoeff = sqrtf(m_body0L->m_frictionCoeff * m_node0->m_frictionCoeff);

	m_CFM[0] = CFM;
	m_CFM[1] = CFM;
	m_CFM[2] = CFM;

	m_ERP[0] = ERP;
	m_ERP[1] = 0.001f * ERP;
	m_ERP[2] = 0.001f * ERP;
}

void TriNodeContact::setContactInfo(const math::Vec3 & cp0, const math::Vec3 & cp1, const math::Vec3 & cn)
{
	using namespace math;

	m_contactPoint0W = cp0;
	m_contactPoint1W = cp1;
	m_contactNormal = cn;

	m_cp0_b0o = physics::transformWorldToOrigin(m_body0L, m_body0R, m_contactPoint0W);
	m_cp1_b0o = physics::transformWorldToOrigin(m_body0L, m_body0R, m_contactPoint1W);

	m_cp0_b1o = m_contactPoint0W;
	m_cp1_b1o = m_contactPoint1W;

	// Calculating Barycentric for
	Vec3 v_p1p0, v_p2p0, p0;
	p0 = m_node0->m_pos;

	v_p1p0 = m_node1->m_pos - p0;
	v_p2p0 = m_node2->m_pos - p0;

	Vec3 normal = v_p1p0.cross(v_p2p0);
	normal.normalize();

	float delta = (cp1 - p0).dot(normal);
	// Project the anchor point onto the triangle, in case it was accidentally set off triangle
	Vec3 contactPointProj = cp1 - delta * normal;

	computeBarycentric(m_node0->m_pos, m_node1->m_pos, m_node2->m_pos, contactPointProj, &m_alpha, &m_beta, &m_gamma);
}

void TriNodeContact::resetLambdas()
{
	m_lambda0[0] = 0.0f;
	m_lambda0[1] = 0.0f;
	m_lambda0[2] = 0.0f;
}

uint32_t TriNodeContact::getNumRowsVel() const
{
	return c_numRowsNormal;
}
uint32_t TriNodeContact::getNumRowsVelFriction() const
{
	return c_numRowsFriction;
}

void TriNodeContact::updateCopyVelInternal(float dt, SolverBase & solver, float initialGuessMul)
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

	vRel -= m_alpha * m_node0->m_vel + m_beta * m_node1->m_vel + m_gamma * m_node2->m_vel;

	*ptr_m_J_06++ = m_contactNormal.x;
	*ptr_m_J_07++ = m_contactNormal.y;
	*ptr_m_J_08++ = m_contactNormal.z;
	*ptr_m_J_09++ = m_contactNormal.x;
	*ptr_m_J_10++ = m_contactNormal.y;
	*ptr_m_J_11++ = m_contactNormal.z;
	*ptr_m_J_12++ = m_contactNormal.x;
	*ptr_m_J_13++ = m_contactNormal.y;
	*ptr_m_J_14++ = m_contactNormal.z;

	for (int i = 0; i < 1; ++i)
	{
		// Null remaining Jacobian
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

	float	*ptr_m_invMassScale_00 = &solver.m_invMassScale_00[m_startIdx],
			*ptr_m_invMassScale_01 = &solver.m_invMassScale_01[m_startIdx],
			*ptr_m_invMassScale_02 = &solver.m_invMassScale_02[m_startIdx],
			*ptr_m_invMassScale_03 = &solver.m_invMassScale_03[m_startIdx],
			*ptr_m_invMassScale_04 = &solver.m_invMassScale_04[m_startIdx],
			*ptr_m_invMassScale_05 = &solver.m_invMassScale_05[m_startIdx];

	for (int i = 0; i < c_numRowsNormal; ++i)
	{
		*ptr_m_JtdNodes_00++ = m_body0L->m_idx;
		*ptr_m_JtdNodes_01++ = m_body0R ? m_body0R->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_02++ = m_node0 ? m_node0->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_03++ = m_node1 ? m_node1->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_04++ = m_node2 ? m_node2->m_idx : SOLVER_NO_BODY;

		// Null out remaining indices
		*ptr_m_JtdNodes_05++ = SOLVER_NO_BODY;

		*(ptr_m_invMassScale_00++) = 1.0f;
		*(ptr_m_invMassScale_01++) = 1.0f;
		*(ptr_m_invMassScale_02++) = 1.0f;
		*(ptr_m_invMassScale_03++) = 1.0f;
		*(ptr_m_invMassScale_04++) = 1.0f;
		*(ptr_m_invMassScale_05++) = 1.0f;
	}

	// Lambda Initial Guess
	solver.m_lambda[m_startIdx] = initialGuessMul * m_lambda0[0];

	m_precalcVRel = vRel;
}
void TriNodeContact::updateCopyVel(float dt, SolverBase & solver)
{
	// We want to avoid overshooting on the first velocity solve, hence
	//	we only accept 50% of the initial guess that is coming from the
	//	previous frame
	const float initialGuessMul = 0.5f;
	updateCopyVelInternal(dt, solver, initialGuessMul);
}
void TriNodeContact::updateCopyVelFriction(float dt, SolverBase & solver)
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
	m_contactNormal.tangentSpace(tan0, tan1);

	Vec3 com0 = m_body0L->m_pos;
	Vec3 relCP0(m_contactPoint0W - com0);
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

	// Friction row 0, body 1
	*ptr_m_J_06++ =  tan0.x;
	*ptr_m_J_07++ =  tan0.y;
	*ptr_m_J_08++ =  tan0.z;
	*ptr_m_J_09++ =  tan0.x;
	*ptr_m_J_10++ =  tan0.y;
	*ptr_m_J_11++ =  tan0.z;
	*ptr_m_J_12++ =  tan0.x;
	*ptr_m_J_13++ =  tan0.y;
	*ptr_m_J_14++ =  tan0.z;

	// Friction row 1, body 1
	*ptr_m_J_06++ =  tan1.x;
	*ptr_m_J_07++ =  tan1.y;
	*ptr_m_J_08++ =  tan1.z;
	*ptr_m_J_09++ =  tan1.x;
	*ptr_m_J_10++ =  tan1.y;
	*ptr_m_J_11++ =  tan1.z;
	*ptr_m_J_12++ =  tan1.x;
	*ptr_m_J_13++ =  tan1.y;
	*ptr_m_J_14++ =  tan1.z;

	for (uint32_t i = 0; i < numRowsFrictionOnly; ++i)
	{
		// Null remaining Jacobian
		*ptr_m_J_15++ = 0.0f;
		*ptr_m_J_16++ = 0.0f;
		*ptr_m_J_17++ = 0.0f;
	}

	bool anchorFriction = false;
#if 0
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

		const float frictionBreakDistEps = 0.025f;
		const float frictionBreakVelEps = 0.05f;
		cpDelta = (worldAnchor00 - worldAnchor10);
		if (cpDelta.sqLen() > frictionBreakDistEps*frictionBreakDistEps || m_precalcVRel.sqLen() > frictionBreakVelEps*frictionBreakVelEps)
		{
			anchorFriction = false;
			getContactInfoOrigin(&m_cache->cp0_b0o, &m_cache->cp1_b0o, &m_cache->cp0_b1o, &m_cache->cp1_b1o);
		}
	}
#else
	Vec3 cpDelta = Vec3C();
#endif
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

	for (uint32_t i = 0; i < numRowsFrictionOnly; ++i)
	{
		*ptr_m_JtdNodes_00++ = m_body0L->m_idx;
		*ptr_m_JtdNodes_01++ = m_body0R ? m_body0R->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_02++ = m_node0 ? m_node0->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_03++ = m_node1 ? m_node1->m_idx : SOLVER_NO_BODY;
		*ptr_m_JtdNodes_04++ = m_node2 ? m_node2->m_idx : SOLVER_NO_BODY;

		// Null out remaining indices
		*ptr_m_JtdNodes_05++ = SOLVER_NO_BODY;
	}

	// Lambda Initial Guess
	const float initialGuessMul = 1.0f;
	solver.m_lambda[frictionStartIdx  ] = initialGuessMul*m_lambda0[1];
	solver.m_lambda[frictionStartIdx+1] = initialGuessMul*m_lambda0[2];
}

void TriNodeContact::fetchLambdasVel(SolverBase & solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
}
void TriNodeContact::fetchLambdasVelFriction(SolverBase & solver)
{
	if (!m_bActive)
		return;

	m_lambda0[0] = solver.m_lambda[m_startIdx  ];
	m_lambda0[1] = solver.m_lambda[m_startIdx+1];
	m_lambda0[2] = solver.m_lambda[m_startIdx+2];
}

}

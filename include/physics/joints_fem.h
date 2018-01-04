#pragma once

#include "math/Mat33.h"
#include "joints.h"

namespace physics
{

class FEMJoint: public JointBase
{
public:

	virtual JointBase::Type getType() const { return JointBase::Type::eFEM; }

	virtual void onStepStart(float dt);

	virtual uint32_t getNumRowsVel() const { return c_numRows; }
	virtual uint32_t getNumRowsVelFriction() const { return c_numRows; }

	/**
	This method is initializes Joint and calculates internal data

	\param solver 				solver that must be used
	\param NodePos0				vector of initial node positions (needed for calculations)
	\param Node1Idx				Index of the 1st node in Tetrahedral-FE
	\param Node2Idx				Index of the 2nd node in Tetrahedral-FE
	\param Node3Idx				Index of the 3rd node in Tetrahedral-FE
	\param Node4Idx				Index of the 4th node in Tetrahedral-FE
	\param Young				Young's modulus of FE (stiffness)
	\param Poisson				Poisson's ratio (transverse strain to axial strain -- perp. contraction to extension)
	\param Yield				Threshold from where strain goes to plastic creep
	\param Creep				Amount of elastic strain, absorbed by plasticity [0; 1]
	\param MaxPlasticStrain		Maximum amount of strain, absorbed by plasticity
	\param Damping				Oscillation reducing coefficient
	\param Regularization 		constraint force mixing parameter
	*/
	virtual void init(
		SolverBase & solver,
		NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2, NodeTranslational * node3, 
		float YoungModulus, float PoissonRatio, float plasticYield, float plasticCreep,
		float maxPlasticStrain, float dampingBeta, float regularization
		);

	virtual void updateCopyVel(float dt, SolverBase &solver) { updateCopy(dt, solver, 0.7f); }
	virtual void updateCopyVelFriction(float dt, SolverBase &solver) { updateCopy(dt, solver, 1.0f); }

	virtual void fetchLambdas(SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver) { fetchLambdas(solver); }
	virtual void fetchLambdasVelFriction(SolverBase &solver) { fetchLambdas(solver); }

	void setYoungPoisson(float Young, float Poisson);

	float getYoungModulus() const
	{
		return m_YoungModulus;
	}

	float getPoissonRatio() const
	{
		return m_PoissonRatio;
	}

	float getPlasticYield() const
	{
		return m_plasticYield;
	}
	void setPlasticYield(float Yield)
	{
		m_plasticYield = Yield;
	}

	float getPlasticCreep() const
	{
		return m_plasticCreep;
	}
	void setPlasticCreep(float Creep)
	{
		m_plasticCreep = Creep;
	}

	float getMaxPlasticStrain() const
	{
		return m_maxPlasticStrain;
	}
	void setMaxPlasticStrain(float MaxPlasticStrain)
	{
		m_maxPlasticStrain = MaxPlasticStrain;
	}

	float getDamping() const
	{
		return m_dampingBeta;
	}
	void setDamping(float Damping)
	{
		m_dampingBeta = Damping;
	}

	float getRegularization() const
	{
		return m_regularization;
	}
	void setRegularization(float Regularization)
	{
		m_regularization = Regularization;
	}

	NodeTranslational * getNode0() { return m_node0; }
	NodeTranslational * getNode1() { return m_node1; }
	NodeTranslational * getNode2() { return m_node2; }
	NodeTranslational * getNode3() { return m_node3; }

	unsigned int m_fetchIndices[6];

	// MEMBERS PUBLIC FOR DEBUG ONLY
	// !!!!!!!!!!DO NOT USE!!!!!!!!!!!!
	float m_lambda0[6];

	// Norm of Elastic Strain for debug purposes [Should be Getter for this?]
	float m_strainNorm;

private:

	struct JointPrecompute
	{
		float J[12][6];
		float e_elastic[6];
	} m_precompute;

	static const uint32_t c_numRows = 6;

	virtual void updateCopy(float dt, SolverBase &solver, float initialGuessCoeff);

	unsigned int m_startIdx;
	float m_regularization;
	float m_CFM[c_numRows];				// Store precomputed CFM (before multiplication)
	float m_B_loc[6 * 12];

	float m_dampingBeta;				// "beta" parameter

	// !!!! This variable is used ONLY FOR setYoungPoisson method
	float m_initialFEVolume;

	// Elasticity parameters
	float m_YoungModulus;
	float m_PoissonRatio;

	// Plasticity parameters
	float m_plasticYield;				// Threshold from where strain goes to plastic creep
	float m_plasticCreep;				// Amount of elastic strain, absorbed by plasticity [0; 1]
	float m_maxPlasticStrain;			// Maximum amount of "saved" plastic strain for FE

#if (WARPING_POLAR_DECOMPOSITION == 1)
	math::Mat33 m_mp0_inv;				// Matrix of tetrahedron positions;
#else
	math::Mat33 m_N;					// Matrix of original FE basis
#endif

	float m_Jp0[6];						// J * pos0;

	NodeTranslational * m_node0, * m_node1, * m_node2, * m_node3;

	float m_E_plastic[6];
};

class BallSocket_FEM: public JointBase
{
public:

	virtual JointBase::Type getType() const { return JointBase::Type::eFEM_BallSocket; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const { return c_numRows; }
	virtual uint32_t getNumRowsVelFriction() const { return c_numRows; }

	/**
	This method is initializes Joint and calculates internal data

	\param solver 			solver that must be used
	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param anchorPoint	 	coordinates of the pin-point in world space
	\param bodyL			rigid body linear node (should be nullptr if attachment to world)
	\param bodyR			rigid body rotational node
	\param node0			FE-point0 linear node
	\param node1			FE-point1 linear node
	\param node2			FE-point2 linear node
	*/
	virtual void init(SolverBase & solver, float ERP, float CFM, const math::Vec3 & anchorPoint, NodeTranslational * bodyL, NodeRotational * bodyR, NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2);
	virtual void updateCopy(float dt, SolverBase & solver);

	virtual void updateCopyVel(float dt, SolverBase & solver) { updateCopy(dt, solver); }
	virtual void updateCopyVelFriction(float dt, SolverBase & solver) { updateCopy(dt, solver); }

	virtual void fetchLambdas(SolverBase & solver);

	virtual void fetchLambdasVel(SolverBase & solver) { fetchLambdas(solver); }
	virtual void fetchLambdasVelFriction(SolverBase & solver) { fetchLambdas(solver); }

	NodeTranslational * getBodyL() { return m_bodyL; }
	NodeRotational *	getBodyR() { return m_bodyR; }
	NodeTranslational * getNode0() { return m_node0; }
	NodeTranslational * getNode1() { return m_node1; }
	NodeTranslational * getNode2() { return m_node2; }

	const math::Vec3 & getBodyAnchor()
	{
		return m_bodyAnchorPoint;
	}

	math::Vec3 getFEMAnchor()
	{
		using namespace math;
		const Vec3 & n0pos = m_node0->m_pos;
		const Vec3 & n1pos = m_node1->m_pos;
		const Vec3 & n2pos = m_node2->m_pos;
		Vec3 normal = (n1pos - n0pos).cross(n2pos - n0pos);
		normal.normalize();
		return m_alpha * n0pos + m_beta * n1pos + m_gamma * n2pos + m_delta * normal;
	}

private:

	static const uint32_t c_numRows = 3;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				// Store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_numRows];

	// Barycentric coordinates of anchor projection onto triangle
	float m_alpha, m_beta, m_gamma;
	// Projection along the normal of anchor point related to triangle
	float m_delta;

	// Anchor point relative to the rigid body, in origin space
	math::Vec3 m_bodyAnchorPoint;

	// TRIPLES INDICES
	NodeTranslational * m_bodyL;
	NodeRotational * m_bodyR;
	
	NodeTranslational * m_node0;
	NodeTranslational * m_node1;
	NodeTranslational * m_node2;
};


class BallSocket_FEM_Simple: public JointBase
{
public:

	virtual JointBase::Type getType() const { return JointBase::Type::eFEM_BallSocketSimple; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const { return c_numRows; }
	virtual uint32_t getNumRowsVelFriction() const { return c_numRows; }

	/**
	This method is initializes Joint and calculates internal data

	\param solver 			solver that must be used
	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param anchorPoint	 	coordinates of the pin-point in world space
	\param bodyL			rigid body linear node (should be nullptr if attachment to world)
	\param bodyR			rigid body rotational node
	\param node0			FE-point0 linear node
	\param node1			FE-point1 linear node
	\param node2			FE-point2 linear node
	*/
	virtual void init(SolverBase & solver, float ERP, float CFM, const math::Vec3 & anchorPoint, NodeTranslational * bodyL, NodeRotational * bodyR, NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2);
	virtual void updateCopy(float dt, SolverBase & solver);

	virtual void updateCopyVel(float dt, SolverBase & solver) { updateCopy(dt, solver); }
	virtual void updateCopyVelFriction(float dt, SolverBase & solver) { updateCopy(dt, solver); }

	virtual void fetchLambdas(SolverBase & solver);

	virtual void fetchLambdasVel(SolverBase & solver) { fetchLambdas(solver); }
	virtual void fetchLambdasVelFriction(SolverBase & solver) { fetchLambdas(solver); }

	NodeTranslational * getBodyL() { return m_bodyL; }
	NodeRotational *	getBodyR() { return m_bodyR; }
	NodeTranslational * getNode0() { return m_node0; }
	NodeTranslational * getNode1() { return m_node1; }
	NodeTranslational * getNode2() { return m_node2; }

	const math::Vec3 & getBodyAnchor()
	{
		return m_bodyAnchorPoint;
	}

	math::Vec3 getFEMAnchor()
	{
		using namespace math;
		const Vec3 & n0pos = m_node0->m_pos;
		const Vec3 & n1pos = m_node1->m_pos;
		const Vec3 & n2pos = m_node2->m_pos;
		return m_alpha * n0pos + m_beta * n1pos + m_gamma * n2pos;
	}

private:

	static const uint32_t c_numRows = 3;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				// Store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_numRows];

	// Barycentric coordinates of anchor projection onto triangle
	float m_alpha, m_beta, m_gamma;

	// Anchor point relative to the rigid body, in origin space
	math::Vec3 m_bodyAnchorPoint;

	// TRIPLES INDICES
	NodeTranslational * m_bodyL;
	NodeRotational * m_bodyR;

	NodeTranslational * m_node0;
	NodeTranslational * m_node1;
	NodeTranslational * m_node2;
};

class TriNodeContact: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param PlanePoint	 	coordinates of any point on the plane
	\param PlaneNormal	 	coordinates of plane's normal vector
	\param Body1L			body1 linear node
	\param Body1R			body1 rotational node
	\param Body2L			body2 linear node (should be nullptr if attachment to world)
	\param Body2R			body2 rotational node
	\param Tolerance		acceptable amount of penetration
	*/
	virtual void init(float ERP, float CFM, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * node0, NodeTranslational * node1, NodeTranslational * node2);
	void setContactInfo(const math::Vec3 & cp0, const math::Vec3 & cp1, const math::Vec3 & cn);

	void resetLambdas();

	virtual JointBase::Type getType() const { return JointBase::Type::eTriNodeContact; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;

	virtual void updateCopyVel(float dt, SolverBase &solver);
	virtual void updateCopyVelFriction(float dt, SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver);
	virtual void fetchLambdasVelFriction(SolverBase &solver);

	NodeTranslational *	getBody0L() const { return m_body0L; }
	NodeRotational *	getBody0R() const { return m_body0R; }
	NodeTranslational *	getNode0() const { return m_node0; }
	NodeTranslational *	getNode1() const { return m_node1; }
	NodeTranslational *	getNode2() const { return m_node2; }

	ContactJointCache * getCache() const { return m_cache; }
	void setCache(ContactJointCache * cache) { m_cache = cache; }
	math::Vec3 getContactPoint0World() const { return m_contactPoint0W; }
	math::Vec3 getContactPoint1World() const { return m_contactPoint1W; }
	math::Vec3 getContactNormalWorld() const { return m_contactNormal; }
	void getContactInfoOrigin(math::Vec3 * cp0_b0o, math::Vec3 * cp1_b0o, math::Vec3 * cp0_b1o, math::Vec3 * cp1_b1o) const
	{
		*cp0_b0o = m_cp0_b0o;
		*cp1_b0o = m_cp1_b0o;
		*cp0_b1o = m_cp0_b1o;
		*cp1_b1o = m_cp1_b1o;
	}

	// MEMBERS PUBLIC FOR DEBUG ONLY
	// !!!!!!!!!!DO NOT USE!!!!!!!!!!!!
	unsigned int m_startIdx;
	bool m_bActive;

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational *	m_body0L;
	NodeRotational *	m_body0R;
	NodeTranslational *	m_node0;
	NodeTranslational *	m_node1;
	NodeTranslational *	m_node2;

	float getLambda(int idx) const
	{
		assert(idx >= 0 && idx < c_numRowsMax);
		return m_lambda0[idx];
	}
	void setLambda(int idx, float lambda)
	{
		assert(idx >= 0 && idx < c_numRowsMax);
		m_lambda0[idx] = lambda;
	}

private:

	static const uint32_t c_numRowsNormal = 1;
	static const uint32_t c_numRowsFriction = 3;
	static const uint32_t c_numRowsMax = c_numRowsFriction;

	void updateCopyVelInternal(float dt, SolverBase &solver, float initialGuessMul);

	float m_alpha, m_beta, m_gamma;

	math::Vec3 m_precalcVRel;

	math::Vec3 m_contactPoint0W, m_contactPoint1W;
	math::Vec3 m_contactNormal;

	ContactJointCache * m_cache;
	math::Vec3 m_cp0_b0o, m_cp1_b0o;
	math::Vec3 m_cp0_b1o, m_cp1_b1o;

	float m_skinWidth;
	float m_restitutionCoeff;
	float m_frictionCoeff;

	float m_lambda0[c_numRowsMax];

	float m_CFM[c_numRowsMax];				// Store precomputed CFM (before multiplication)
	float m_ERP[c_numRowsMax];
	float m_Tolerance;
};

}
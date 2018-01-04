#pragma once

#include <cstdint>

#include "solver_base.h"

namespace physics
{

#define PLANE_FRICTION 1

class JointBase
{
public:

	enum class Type
	{
		eBallSocket,
		eSlider,

		eFixedRotation,
		eAxisRotation,

		eAxisRotationLimit,
		eAxisLinearLimit,

		eMotorRotation,

		ePlane,
		eContact,

		eFEM,
		eFEM_BallSocket,
		eFEM_BallSocketSimple,

		eFEM_FixedRotation,
		eFEM_AxisRotation,

		eTriNodeContact,

		eNUM_ENTRIES
	};

	virtual ~JointBase()
	{
	}

	// Returns joint type for safe casting (rendering/debug)
	virtual Type getType() const = 0;

	// This function should be called for each joint on the beginning of the step
	//	to allow precompute certain quantities, etc.
	virtual void onStepStart(float dt) = 0;

	// Report number of constraint rows on frictionless velocity calculation step
	//	the frictionless velocity calculation step is special step to calculate friction force limits
	virtual uint32_t getNumRowsVel() const = 0;
	// Report number of constraint rows on full velocity solve w/ friction
	virtual uint32_t getNumRowsVelFriction() const = 0;

	/**
	These methods are called each frame before the joint is used for solving in a particular solver.
	The method must prepare the Jacobians, desired velocities, CFMs, ERPs, low and high bounds, and lambda
	initial guesses for each mathematical constraint of this joint and fill this data in teh designated
	slots of the solver.
	*/
	virtual void updateCopyVel(float dt, SolverBase &solver) = 0;
	virtual void updateCopyVelFriction(float dt, SolverBase &solver) = 0;

	/**
	These methods are called each step after solving the system. It must store computed lambdas into internal
	storage, so they could be used as initial guesses later.
	Solver that stores computed lambdas comes in as parameter.
	*/
	virtual void fetchLambdasVel(SolverBase &solver) = 0;
	virtual void fetchLambdasVelFriction(SolverBase &solver) = 0;
};



class BallSocket: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param AnchorPoint	 	coordinates of the pin-point in world space
	\param Body1L			body1 linear node
	\param Body1R			body1 rotational node
	\param Body2L			body2 linear node (should be nullptr if attachment to world)
	\param Body2R			body2 rotational node
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 & anchorPoint, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual Type getType() const { return JointBase::Type::eBallSocket; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase &solver);
	virtual void updateCopyVelFriction(float dt, SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver);
	virtual void fetchLambdasVelFriction(SolverBase &solver);

	// Temporary function for rendering
	void getAnchorPoints_BS(math::Vec3 & bsAnchor0, math::Vec3 & bsAnchor1);

	void setAnchorPoint0(const math::Vec3 & anchorPoint0);
	void setAnchorPoint1(const math::Vec3 & anchorPoint1);

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	void updateCopy(float dt, SolverBase & solver, float initialGuessCoeff);

	static const uint32_t c_numRows = 3;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				//store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_numRows];

	// BallSocket Anchors, body-space
	// IMPORTANT: these coordinates are stored versus user-defined ORIGIN, and not CoM;
	//	this needs to be accounted for in the joint init and update. The reason to do
	//	that is because linear nodes do not have any info about rotation and hence CoMs,
	//	since they could be point masses just as well. And with full rigid bodies they
	//	are pushed to be exactly at CoM, and not where the user originally placed the
	//	body.
	// As user would want to set joints in the original space they've set the body and
	//	not some internal engine space, and also because the rigid body mass could
	//	actually change (e.g. shapes added/removed), the joint attachment shouldn't be
	//	moved with the body CoM => we would want to store the anchor in the design-space.
	math::Vec3 m_bodyOriginAnchorPoints[2];

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

class Slider: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param AnchorPoint	 	coordinates of the pin-point in world space
	\param Body1L			body1 linear node
	\param Body1R			body1 rotational node
	\param Body2L			body2 linear node (should be nullptr if attachment to world)
	\param Body2R			body2 rotational node
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 & axis, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual Type getType() const { return JointBase::Type::eSlider; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase &solver);
	virtual void updateCopyVelFriction(float dt, SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver);
	virtual void fetchLambdasVelFriction(SolverBase &solver);

	// Temporary function for rendering
	math::Vec3 getWorldAxis0() const;
	math::Vec3 getWorldAxis1() const;

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	void updateCopy(float dt, SolverBase & solver, float initialGuessCoeff);

	static const uint32_t c_numRows = 2;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				//store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_numRows];

	math::Vec3 m_localOffsets[2];
	math::Vec3 m_localAxes[2];

	// Anchors, body-space, see BallSocket joint for comments
	math::Vec3 m_bodyOriginAnchorPoints[2];

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

class FixedRotation: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param AnchorPoint	 	coordinates of the pin-point in world space
	\param Body1L			body1 linear node
	\param Body1R			body1 rotational node
	\param Body2L			body2 linear node (should be nullptr if attachment to world)
	\param Body2R			body2 rotational node
	*/
	virtual void init(float ERP, float CFM, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual JointBase::Type getType() const { return JointBase::Type::eFixedRotation; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase & solver);
	virtual void updateCopyVelFriction(float dt, SolverBase & solver);

	virtual void fetchLambdasVel(SolverBase & solver);
	virtual void fetchLambdasVelFriction(SolverBase & solver);

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	void updateCopy(float dt, SolverBase &solver, float initialGuessCoeff);

	static const uint32_t c_numRows = 3;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				//store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_numRows];

	math::Quat m_initialRelRot;

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

class AxisRotation: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param axis			 	allowed rotation axis in world space
	\param body0L			body0 linear node
	\param body0R			body0 rotational node
	\param body1L			body1 linear node (should be nullptr if attachment to world)
	\param body1R			body1 rotational node
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 & axis, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual JointBase::Type getType() const { return JointBase::Type::eAxisRotation; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase & solver);
	virtual void updateCopyVelFriction(float dt, SolverBase & solver);

	virtual void fetchLambdasVel(SolverBase & solver);
	virtual void fetchLambdasVelFriction(SolverBase & solver);

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	void updateCopy(float dt, SolverBase &solver, float initialGuessCoeff);

	static const uint32_t c_numRows = 2;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				//store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_numRows];

	math::Vec3 m_localAxes[2];

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

class AxisRotationLimit: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param axis			 	allowed rotation axis in world space
	\param angLo			minimum allowed rotation angle
	\param angHi			maximum allowed rotation angle
	\param body0L			body0 linear node
	\param body0R			body0 rotational node
	\param body1L			body1 linear node (should be nullptr if attachment to world)
	\param body1R			body1 rotational node
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 & axis, float angLo, float angHi, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual JointBase::Type getType() const { return JointBase::Type::eAxisRotationLimit; }

	virtual void setToleranceAngle(float toleranceAngle)
	{
		m_sinHalfTolerance = sinf(0.5f * toleranceAngle);
	}

	virtual void calcLimitSinHalfAngle(float * limitAngle) const;
	enum class LimitState
	{
		eFree,
		eLoViolation,
		eHiViolation
	};
	virtual LimitState testLimit(float limitSinHalfAngle) const;

	// Calculates bodies relative rotations and world axes to avoid doing so on each limit test,
	//	and in the update function
	virtual void onStepStart(float dt);

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase & solver);
	virtual void updateCopyVelFriction(float dt, SolverBase & solver);

	virtual void fetchLambdasVel(SolverBase & solver);
	virtual void fetchLambdasVelFriction(SolverBase & solver);

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	struct JointPrecompute
	{
		math::Quat relRot;
		math::Quat errRot;
		math::Vec3 worldAxes[2];
	} m_precompute;

	void updateCopy(float dt, SolverBase &solver, float initialGuessCoeff);

	static const uint32_t c_maxNumRows = 1;

	unsigned int m_startIdx;
	float m_CFM[c_maxNumRows];				//store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_maxNumRows];

	LimitState m_activeLimitState;
	float m_sinHalfTolerance;
	float m_sinHalfLo;
	float m_sinHalfHi;

	math::Quat m_initialRelRot;
	math::Vec3 m_localAxes[2];

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

class AxisLinearLimit: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param axis			 	allowed rotation axis in world space
	\param angLo			minimum allowed rotation angle
	\param angHi			maximum allowed rotation angle
	\param body0L			body0 linear node
	\param body0R			body0 rotational node
	\param body1L			body1 linear node (should be nullptr if attachment to world)
	\param body1R			body1 rotational node
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 & axis, float distLo, float distHi, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual JointBase::Type getType() const { return JointBase::Type::eAxisLinearLimit; }

	virtual void setToleranceDistance(float toleranceDistance)
	{
		m_distTolerance = toleranceDistance;
	}

	virtual void calcLimitDist(float * limitAngle) const;
	enum class LimitState
	{
		eFree,
		eLoViolation,
		eHiViolation
	};
	virtual LimitState testLimit(float limitDist) const;

	// Calculates bodies relative rotations and world axes to avoid doing so on each limit test,
	//	and in the update function
	virtual void onStepStart(float dt);

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase & solver);
	virtual void updateCopyVelFriction(float dt, SolverBase & solver);

	virtual void fetchLambdasVel(SolverBase & solver);
	virtual void fetchLambdasVelFriction(SolverBase & solver);

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	struct JointPrecompute
	{
		math::Vec3 worldPos[2];
		math::Vec3 worldAxes[2];
	} m_precompute;

	void updateCopy(float dt, SolverBase &solver, float initialGuessCoeff);

	static const uint32_t c_maxNumRows = 1;

	unsigned int m_startIdx;
	float m_CFM[c_maxNumRows];				//store precomputed CFM (before multiplication)
	float m_ERP;

	float m_lambda0[c_maxNumRows];

	LimitState m_activeLimitState;
	float m_distTolerance;
	float m_distLo;
	float m_distHi;

	math::Vec3 m_localAxes[2];
	// In case body 1 is not specified
	math::Vec3 m_worldPos;

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

class MotorRotation: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param AnchorPoint	 	coordinates of the pin-point in world space
	\param Body1L			body1 linear node
	\param Body1R			body1 rotational node
	\param Body2L			body2 linear node (should be nullptr if attachment to world)
	\param Body2R			body2 rotational node
	*/
	virtual void init(float CFM, const math::Vec3 & axis, float vel, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);

	virtual Type getType() const { return JointBase::Type::eMotorRotation; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;
	virtual void updateCopyVel(float dt, SolverBase &solver);
	virtual void updateCopyVelFriction(float dt, SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver);
	virtual void fetchLambdasVelFriction(SolverBase &solver);

	void setDesiredVelocity(float vel)	{ m_desiredVelocity = vel; }
	float getDesiredVelocity() const	{ return m_desiredVelocity; }
	void setForceMin(float forceMin)	{ m_forceMin = forceMin; }
	float getForceMin() const			{ return m_forceMin; }
	void setForceMax(float forceMax)	{ m_forceMax = forceMax; }
	float getForceMax() const			{ return m_forceMax; }

	void setInvMassScales(float body0InvMassScale, float body1InvMassScale)
	{
		m_invMassScales[0] = body0InvMassScale;
		m_invMassScales[1] = body1InvMassScale;
	}
	void getInvMassScales(float * body0InvMassScale, float * body1InvMassScale) const
	{
		*body0InvMassScale = m_invMassScales[0];
		*body1InvMassScale = m_invMassScales[1];
	}

	NodeTranslational *	getBody0L() { return m_body0L; };
	NodeRotational *	getBody0R() { return m_body0R; };
	NodeTranslational *	getBody1L() { return m_body1L; };
	NodeRotational *	getBody1R() { return m_body1R; };

private:

	void updateCopy(float dt, SolverBase & solver, float initialGuessCoeff);

	static const uint32_t c_numRows = 1;

	unsigned int m_startIdx;
	float m_CFM[c_numRows];				//store precomputed CFM (before multiplication)

	float m_lambda0[c_numRows];

	float m_desiredVelocity;
	float m_forceMin, m_forceMax;

	math::Vec3 m_localAxes[2];

	// InvMass scales, coefficient for the constraint local mass scaling mechanics 
	float m_invMassScales[2];

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;
};

// Contact
//////////////////////////////////////////////////////////////////////////

// POD
struct ContactJointCache
{
	bool isUsed;
	NodeTranslational * body0L;
	NodeRotational * body0R;
	NodeTranslational * body1L;
	NodeRotational * body1R;
	math::Vec3 cp0_b0o;
	math::Vec3 cp1_b0o;
	math::Vec3 cp0_b1o;
	math::Vec3 cp1_b1o;

	float lambdas[3];
};

class Contact: public JointBase
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
	virtual void init(float ERP, float CFM, NodeTranslational * body0L, NodeRotational * body0R, NodeTranslational * body1L, NodeRotational * body1R);
	void setContactInfo(const math::Vec3 & cp0, const math::Vec3 & cp1, const math::Vec3 & cn);

	void resetLambdas();

	virtual JointBase::Type getType() const { return JointBase::Type::eContact; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;

	virtual void updateCopyVel(float dt, SolverBase &solver);
	virtual void updateCopyVelFriction(float dt, SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver);
	virtual void fetchLambdasVelFriction(SolverBase &solver);

	NodeTranslational * getBody0L() const { return m_body0L; }
	NodeRotational * getBody0R() const { return m_body0R; }
	NodeTranslational * getBody1L() const { return m_body1L; }
	NodeRotational * getBody1R() const { return m_body1R; }

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

	// This function is only needed if local mass ration adjustment is enabled
	void setLocalMassScalingParams(bool isEnabled, float maxMassScale, const math::Vec3 & gravity)
	{
		m_localMassScalingParams.isEnabled = isEnabled;
		m_localMassScalingParams.maxMassScale = maxMassScale;
		m_localMassScalingParams.m_gravityDirection = gravity.getNormalized();
	}

	// MEMBERS PUBLIC FOR DEBUG ONLY
	// !!!!!!!!!!DO NOT USE!!!!!!!!!!!!
	unsigned int m_startIdx;
	bool m_bActive;

	// Physics nodes pointer [ nullptr for Body2 linear means attachment to world ]
	NodeTranslational * m_body0L;
	NodeRotational * m_body0R;
	NodeTranslational * m_body1L;
	NodeRotational * m_body1R;

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

	math::Vec3 m_precalcVRel;

	math::Vec3 m_contactPoint0W, m_contactPoint1W;
	math::Vec3 m_contactNormal;

	struct LocalMassScalingParameters
	{
		bool isEnabled;
		float maxMassScale;
		math::Vec3 m_gravityDirection;
	};
	LocalMassScalingParameters m_localMassScalingParams;

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

#if (PLANE_FRICTION == 1)

class PlaneConstraint: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param PlanePoint	 	coordinates of any point on the plane
	\param PlaneNormal	 	coordinates of plane's normal vector
	\param BodyL			body linear node
	\param Tolerance		acceptable amount of penetration
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 &PlanePoint, const math::Vec3 &PlaneNormal, NodeTranslational * bodyL, float Tolerance);

	virtual JointBase::Type getType() const { return JointBase::Type::ePlane; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRowsVel() const;
	virtual uint32_t getNumRowsVelFriction() const;

	virtual void updateCopyVel(float dt, SolverBase &solver);
	virtual void updateCopyVelFriction(float dt, SolverBase &solver);

	virtual void fetchLambdasVel(SolverBase &solver);
	virtual void fetchLambdasVelFriction(SolverBase &solver);


	// MEMBERS PUBLIC FOR DEBUG ONLY
	// !!!!!!!!!!DO NOT USE!!!!!!!!!!!!
	unsigned int m_startIdx;
	bool m_bActive;

	NodeTranslational * m_bodyL;

	// Plane equation vectors
	math::Vec3 m_PlaneNormal;
	float m_PlaneD;

	float getLambda(int idx)
	{
		assert(idx >= 0 && idx < c_numRows);
		return m_lambda0[idx];
	}

private:

	virtual void updateCopy(float dt, SolverBase &solver);

	static const uint32_t c_numRows = 3;

	float m_lambda0[c_numRows];

	float m_CFM[c_numRows];				// Store precomputed CFM (before multiplication)
	float m_ERP[c_numRows];
	float m_Tolerance;
};

//////////////////////////////////////////////////////////////////////////
#else
//////////////////////////////////////////////////////////////////////////

class PlaneConstraint: public JointBase
{
public:

	/**
	This method is initializes Joint and calculates internal data

	\param ERP				error reduction parameter (stabilization)
	\param CFM 				constraint force mixing parameter (regularization)
	\param PlanePoint	 	coordinates of any point on the plane
	\param PlaneNormal	 	coordinates of plane's normal vector
	\param BodyL			body linear node
	\param Tolerance		acceptable amount of penetration
	*/
	virtual void init(float ERP, float CFM, const math::Vec3 &PlanePoint, const math::Vec3 &PlaneNormal, NodeTranslational * bodyL, float Tolerance);

	virtual JointBase::Type getType() const { return JointBase::Type::ePlane; }

	virtual void onStepStart(float dt) { }

	virtual uint32_t getNumRows() const;
	virtual void updateCopy(float dt, SolverBase &solver);

	virtual void fetchLambdas(SolverBase &solver);


	// MEMBERS PUBLIC FOR DEBUG ONLY
	// !!!!!!!!!!DO NOT USE!!!!!!!!!!!!
	unsigned int m_startIdx;
	bool m_bActive;

	NodeTranslational * m_bodyL;

	// Plane equation vectors
	math::Vec3 m_PlaneNormal;
	float m_PlaneD;

	float getLambda(int idx)
	{
		assert(idx >= 0 && idx < c_numRows);
		return m_lambda0[idx];
	}

private:

	static const uint32_t c_numRows = 1;

	float m_lambda0[c_numRows];

	float m_CFM[c_numRows];				// Store precomputed CFM (before multiplication)
	float m_ERP[c_numRows];
	float m_Tolerance;
};

#endif

}

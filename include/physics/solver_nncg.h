#pragma once

#include "solver_base.h"

namespace physics
{

// Implementation of NNCG solver, described in
// "A nonsmooth nonlinear conjugate gradient method for interactive
//	contact force problems"
// by M. Silcowitz-Hansen, S. Niebe, K. Erleben
class SolverNNCG : public SolverBase
{
public:

	SolverNNCG(unsigned int maxIterations, float precision) : SolverBase(maxIterations, precision)
	{
	}

	void solve(float dt);

	virtual void setNumAdditionalRows(uint32_t numRowsTotal)
	{
		m_dLamVec.resize(numRowsTotal);
		m_p.resize(numRowsTotal);
	}

	std::vector<float> m_dLamVec;
	std::vector<float> m_p;
};

}

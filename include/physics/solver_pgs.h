#pragma once

#include "solver_base.h"

namespace physics
{

class SolverPGS : public SolverBase
{
public:

	SolverPGS(unsigned int maxIterations, float precision) : SolverBase(maxIterations, precision)
	{
	}

	void solve(float dt);

	virtual void setNumAdditionalRows(uint32_t numRowsTotal)
	{
	}
};

}

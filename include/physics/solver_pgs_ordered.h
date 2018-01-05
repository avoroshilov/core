#pragma once

#include "solver_base.h"

namespace physics
{

class SolverPGSOrdered : public SolverBase
{
public:

	enum class Order
	{
		eForward,
		eBackward,
		eRandom,
		eLocalShuffle2,
		eSplit2,

		eCustom,

		eNUM_ENTRIES
	};

	SolverPGSOrdered(unsigned int maxIterations, float precision) :
		SolverBase(maxIterations, precision),
		m_order(Order::eRandom)
	{
	}

	void solve(float dt);

	virtual void setNumAdditionalRows(uint32_t numRowsTotal)
	{
		m_permutIdx.resize(numRowsTotal);
	}

	Order m_order;

	// Array of indices to access the constraints
	std::vector<uint32_t> m_permutIdx;
};

}

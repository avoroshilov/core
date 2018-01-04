#pragma once

#include "solver_base.h"

namespace physics
{

class SolverLCPCG : public SolverBase
{
public:

	SolverLCPCG(unsigned int maxIterations, float precision) : SolverBase(maxIterations, precision)
	{
	}

	void solve(float dt);

	virtual void setNumAdditionalRows(uint32_t numRowsTotal)
	{
		m_ad_vec.resize(numRowsTotal);
		m_ap_vec.resize(numRowsTotal);

		m_p.resize(numRowsTotal);
		m_d.resize(numRowsTotal);
		m_g.resize(numRowsTotal);
	}

	std::vector<float> m_ad_vec;								// vector A * d
	std::vector<float> m_ap_vec;								// vector A * p

	std::vector<float> m_p;										// stepping vector
	std::vector<float> m_d;										// stepping vector, also intermediate in Power Iteration
	std::vector<float> m_g;										// gradient vector
};

}

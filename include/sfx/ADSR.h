#pragma once 

#include <assert.h>

namespace sfx
{

#if defined(_DEBUG)
#	define M_ASSERT(expr) assert(expr)
#else
#	define M_ASSERT(expr)
#endif

class ADSRModulator
{
	float m_start[2];
	float m_attack[2];
	float m_decay[2];
	float m_sustain[2];
	float m_release[2];
	float m_endAmp[2];

public:

	ADSRModulator()
	{
		m_start[0] = 0.0f; m_start[1] = 0.0f;
		m_endAmp[0] = 1.0f; m_endAmp[1] = 0.0f;
	}

	float getValue(float t)
	{
		float val;

		if (t > m_endAmp[0])
		{
			val = m_endAmp[1];
		}
		else
		if (t > m_release[0])
		{
			float interp = (t - m_release[0]) / (m_endAmp[0] - m_release[0]);
			M_ASSERT(interp >= 0.0f && interp <= 1.0f);
			val = m_release[1] + interp * (m_endAmp[1] - m_release[1]);
		}
		else
		if (t > m_sustain[0])
		{
			float interp = (t - m_sustain[0]) / (m_release[0] - m_sustain[0]);
			M_ASSERT(interp >= 0.0f && interp <= 1.0f);
			val = m_sustain[1] + interp * (m_release[1] - m_sustain[1]);
		}
		else
		if (t > m_decay[0])
		{
			float interp = (t - m_decay[0]) / (m_sustain[0] - m_decay[0]);
			M_ASSERT(interp >= 0.0f && interp <= 1.0f);
			val = m_decay[1] + interp * (m_sustain[1] - m_decay[1]);
		}
		else
		if (t > m_attack[0])
		{
			float interp = (t - m_attack[0]) / (m_decay[0] - m_attack[0]);
			M_ASSERT(interp >= 0.0f && interp <= 1.0f);
			val = m_attack[1] + interp * (m_decay[1] - m_attack[1]);
		}
		else
		if (t > m_start[0])
		{
			float interp = (t - m_start[0]) / (m_attack[0] - m_start[0]);
			M_ASSERT(interp >= 0.0f && interp <= 1.0f);
			val = m_start[1] + interp * (m_attack[1] - m_start[1]);
		}
		else
		{
			val = m_start[1];
		}

		return val;
	}

	float getValue(int n, int num)
	{
		float t = n / (float)num;
		return getValue(t);		
	}

	void setStart(float x, float y) { m_start[0] = x; m_start[1] = y; }
	void setAttack(float x, float y) { m_attack[0] = x; m_attack[1] = y; }
	void setDecay(float x, float y) { m_decay[0] = x; m_decay[1] = y; }
	void setSustain(float x, float y) { m_sustain[0] = x; m_sustain[1] = y; }
	void setRelease(float x, float y) { m_release[0] = x; m_release[1] = y; }
	void setFinalAmp(float x, float y) { m_endAmp[0] = x; m_endAmp[1] = y; }

	float getSustainTime() const { return m_decay[0]; }
	float getReleaseTime() const { return m_sustain[0]; }

	void sumDurations()
	{
		m_attack[0] += m_start[0];
		m_decay[0] += m_attack[0];
		m_sustain[0] += m_decay[0];
		m_release[0] += m_sustain[0];
		m_endAmp[0] += m_release[0];
	}
};

#undef M_ASSERT

}
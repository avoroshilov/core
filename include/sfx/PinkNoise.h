#pragma once

#include "helpers/Common.h"

namespace sfx
{

class PinkNoise
{
protected:

	scalar * m_coeffs;
	scalar * m_buf;
	uint m_numPoles;

	uint m_curHistoryIdx;

	scalar m_alpha;

public:

	// [0.0, 2.0)
	// alpha == 0.0, output noise is white
	// alpha == 1.0, output noise is pink
	// alpha == 2.0, output noise is red
	void setAlpha(scalar alpha) { m_alpha = alpha; }
	scalar getAlpha() const { return m_alpha; }

	PinkNoise():
		m_alpha(1.0f),
		m_coeffs(0),
		m_buf(0)
	{
		setNumPoles(10);
	}

	void setNumPoles(uint numPoles)
	{
		m_numPoles = numPoles;
		if (m_coeffs)
			delete [] m_coeffs;
		if (m_buf)
			delete [] m_buf;

		m_coeffs = new scalar[numPoles];
		m_buf = new scalar[numPoles];
	}
	uint getNumPoles() const { return m_numPoles; }

	void init()
	{
		m_curHistoryIdx = 0;
		scalar coeff = 1.0f;
		for (uint i = 0; i < m_numPoles; ++i)
		{
			m_buf[i] = 0.0f;
			coeff = m_coeffs[i] = ( (i - m_alpha/2.0f) * coeff ) / (i+1);
		}
	}

	// Generates noise sample, however it is not normalized
	// (more poles, more spread around 0.0)
	scalar generateSample()
	{
		uint rnd30 = rand30() & 65535;
		scalar white_noise = (rnd30 - 32768.0f) / 32768.0f;
		scalar pink_noise = white_noise;

		for (uint j = 0; j < m_numPoles; ++j)
		{
			pink_noise -= m_coeffs[j] * m_buf[(m_curHistoryIdx - j) % m_numPoles];
		}
		m_curHistoryIdx = (m_curHistoryIdx + 1) % m_numPoles;
		m_buf[m_curHistoryIdx] = pink_noise;

		return pink_noise;
	}
};

}
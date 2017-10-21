#pragma once

namespace sfx
{

class FilterDSP
{
protected:

	float * m_contHistoryI, * m_contHistoryO;

	uint m_tapsI, m_tapsO;
	float * m_coeffI, * m_coeffO;

public:

	FilterDSP(uint tapsI, uint tapsO):
		m_tapsI(tapsI), m_tapsO(tapsO),
		m_coeffI(0), m_coeffO(0),
		m_contHistoryI(0), m_contHistoryO(0)
	{
		if (m_tapsI || m_tapsO)
		{
			float * buf = new float[(m_tapsI + m_tapsO) << 1];
			m_coeffI = buf;
			m_coeffO = buf + m_tapsI;
			m_contHistoryI = buf + m_tapsI + m_tapsO;
			m_contHistoryO = buf + m_tapsI + m_tapsO + m_tapsI;
		}

		resetHistory();
	}
	~FilterDSP()
	{
		freeMem();
	}

	void freeMem()
	{
		// Deleting only once
		if (m_coeffI)
		{
			delete [] m_coeffI;
			m_coeffI = 0;
		}
	}

	void setTaps(uint tapsI, uint tapsO)
	{
		freeMem();

		m_tapsI = tapsI;
		m_tapsO = tapsO;

		if (m_tapsI || m_tapsO)
		{
			float * buf = new float[m_tapsI + m_tapsO + m_tapsO];
			m_coeffI = buf;
			m_coeffO = buf + m_tapsI;
			m_contHistoryI = buf + m_tapsI + m_tapsO;
			m_contHistoryO = buf + m_tapsI + m_tapsO + m_tapsI;
		}
		else
		{
			m_coeffI = 0;
			m_coeffO = 0;
			m_contHistoryI = 0;
			m_contHistoryO = 0;
		}

		resetHistory();
	}

	float * getCoefI() { return m_coeffI; }
	float * getCoefO() { return m_coeffO; }

	void apply(float * output, float * input, uint length)
	{
		for (uint i = 0; i < length; ++i)
		{
			float filteredVal = m_coeffI[0] * input[i];
			for (uint j = 1; j < m_tapsI; ++j)
			{
				if (i >= j)
					filteredVal += m_coeffI[j] * input[i - j];
			}
			for (uint j = 0; j < m_tapsO; ++j)
			{
				if (i > j)
					filteredVal += m_coeffO[j] * output[i - j - 1];
			}
			output[i] = filteredVal;
		}
	}

	float * getHistoryI() { return m_contHistoryI; }
	float * getHistoryO() { return m_contHistoryO; }

	void resetHistory()
	{
		for (uint j = 0; j < m_tapsI - 1; ++j)
		{
			m_contHistoryI[j] = 0.0f;
		}
		for (uint j = 0; j < m_tapsO; ++j)
		{
			m_contHistoryO[j] = 0.0f;
		}
	}

	void applyCont(float * output, float input)
	{
		float & filteredVal = *output;
		filteredVal = m_coeffI[0] * input;
		//////////////////////////////////////////////////////////////////////////
		for (uint i = 1; i < m_tapsI; ++i)
		{
			filteredVal += m_coeffI[i] * m_contHistoryI[i - 1];
		}
		for (uint i = m_tapsI - 1; i >= 1; --i)
		{
			m_contHistoryI[i] = m_contHistoryI[i - 1];
		}
		m_contHistoryI[0] = input;
		//////////////////////////////////////////////////////////////////////////
		for (uint i = 0; i < m_tapsO; ++i)
		{
			filteredVal += m_coeffO[i] * m_contHistoryO[i];
		}
		for (uint i = m_tapsO - 1; i >= 1; --i)
		{
			m_contHistoryO[i] = m_contHistoryO[i - 1];
		}
		m_contHistoryO[0] = *output;
		//////////////////////////////////////////////////////////////////////////
	}
};

class LowPass1pIIR : public FilterDSP
{
public:

	LowPass1pIIR():
		FilterDSP(1, 1)
	{
	}

	void setParams(float freqNormalized)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		float phi = 2 * PI * freqNormalized;

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		float alpha = freqNormalized * 2.0f;

		coeffI[0] = alpha;
		coeffO[0] = 1.0f - alpha;
	}

	static LowPass1pIIR & getInstance()
	{
		static LowPass1pIIR lowPassIIR;
		return lowPassIIR;
	}
};


class FilterIIR_2p2z : public FilterDSP
{
protected:

	float m_z1[2], m_z2[2];	// zeros
	float m_p1[2], m_p2[2];	// poles
	float m_baseCoeffI[3];

	float m_normalization;
	float m_cutoffFreq;

public:

	FilterIIR_2p2z():
		FilterDSP(3, 2),
		m_normalization(1.0f)
	{
		m_baseCoeffI[0] = m_baseCoeffI[1] = m_baseCoeffI[2] = 0.0f;
		for (uint i = 0; i < 2; ++i)
		{
			m_z1[i] = m_z2[i] = 0.0f;
			m_p1[i] = m_p2[i] = 0.0f;
		}
	}

	void autoNormalize(float freqNormalized = 0.0f)
	{
		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		float maxAmp = responseM(freqNormalized, false);

		m_normalization = 1.0f / maxAmp;
		coeffI[0] = m_baseCoeffI[0] * m_normalization;
		coeffI[1] = m_baseCoeffI[1] * m_normalization;
		coeffI[2] = m_baseCoeffI[2] * m_normalization;
	}

	void autoRenormalize(float freqNormalized = 0.0f)
	{
		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		float maxAmp = responseM(freqNormalized);

		float newNormalization = 1.0f / maxAmp;

		m_normalization *= newNormalization;
		coeffI[0] = m_baseCoeffI[0] * m_normalization;
		coeffI[1] = m_baseCoeffI[1] * m_normalization;
		coeffI[2] = m_baseCoeffI[2] * m_normalization;
	}

	void autoNormalizeI()
	{
		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		coeffI[0] = m_baseCoeffI[0];
		coeffI[1] = m_baseCoeffI[1];
		coeffI[2] = m_baseCoeffI[2];

		float maxResponse = 0.0f;
		resetHistory();
		for (int i = 0; i < 20; ++i)
		{
			float signal = (i == 0)?1.0f:0.0f;
			applyCont(&signal, signal);
			maxResponse += signal;
		}
		resetHistory();

		m_normalization = 1.0f / maxResponse;
		coeffI[0] = m_baseCoeffI[0] * m_normalization;
		coeffI[1] = m_baseCoeffI[1] * m_normalization;
		coeffI[2] = m_baseCoeffI[2] * m_normalization;
	}

#if defined (FILTERS_AUTONORMALIZE_SINETABLE)
	void autoNormalizeS(float freqNormalized = 0.0f)
	{
		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		if (freqNormalized == 0.0f)
			freqNormalized = 5.0f / 44100.0f;

		const uint maxSampleSize = 10000;
		const float mul = freqNormalized * (2 * PI);

		coeffI[0] = m_baseCoeffI[0];
		coeffI[1] = m_baseCoeffI[1];
		coeffI[2] = m_baseCoeffI[2];

		float maxResponse = 0.0f;
		resetHistory();

		float t = 0.0f;
		float dt = mul;
		for (int i = 0; i < 20; ++i)
		{
			float signal = SineTable::GetVal( t );
			applyCont(&signal, signal);
			t += dt;
		}
		int tt = 0;
		for (int i = 20; i < maxSampleSize; ++i)
		{
			float signal = SineTable::GetVal( t );
			applyCont(&signal, signal);
			t += dt;

			++tt;
			if (maxResponse < signal)
			{
				maxResponse = signal;
				tt = 0;
			}

			if (tt > 500)
			{
				break;
			}
		}
		resetHistory();

		m_normalization = 1.0f / maxResponse;
		coeffI[0] = mBaseCoeffI[0] * m_normalization;
		coeffI[1] = mBaseCoeffI[1] * m_normalization;
		coeffI[2] = mBaseCoeffI[2] * m_normalization;
	}
#endif

	float responseM(float freqNormalized, bool normalize = true) const
	{
		//if (freqNormalized > 0.5f)
		//	freqNormalized = 1.0f - (freqNormalized - 0.5f);

		float phi = 2 * PI * freqNormalized;

		// Frequency on complex plane
		float zx = cosf(phi);
		float zy = sinf(phi);
		// Zeros
		float z1x = m_z1[0], z1y = m_z1[1];
		float z2x = m_z2[0], z2y = m_z2[1];
		// Poles
		float p1x = m_p1[0], p1y = m_p1[1];
		float p2x = m_p2[0], p2y = m_p2[1];

		/*
		Here's some maths behind the response magnitude calculation:

		        a0 * (z - z1) * (z - z2)   a0 * (Re(N) + i*Im(N))   a0 * (Re(N) + i*Im(N)) * (Re(D) - i*Im(D))
		 H(w) = ------------------------ = ---------------------- = ------------------------------------------ =
		          (z - p1) * (z - p2)        (Re(D) + i*Im(D))         (Re(D) + i*Im(D))*(Re(D) - i*Im(D))
			  
			    a0 * (Re(N) + i*Im(N)) * (Re(D) - i*Im(D))
			  = ------------------------------------------
			             Re(D)*Re(D) + Im(D)*Im(D)

		 
		 |H(w)| is the magnitude (use |H(w)| = 1 for normalization)

		 Further simplifications:
		 
		 Numerator:
		  a0 * (Nre + i*Nim)*(Dre - i*Dim) = (a + i*b) * (c - i*d) = a*c + b*d + i*(b*c - a*d) = 
					a0 * (Nre*Dre + Nim*Dim + i*(Nim*Dre - Nre*Dim))
		
		 =>	Re(H(w)) = a0 * (Nre*Dre + Nim*Dim) / (Dre*Dre + Dim*Dim)
			Im(H(w)) = a0 * (Nim*Dre - Nre*Dim) / (Dre*Dre + Dim*Dim)

				   ______________________________________
		 |H(w)| = V Re(H(w))*Re(H(w)) + Im(H(w))*Im(H(w))

		*/

		float Nre = zx*zx - zx*z2x - zy*zy - zy*z2y - z1x*zx + z1x*z2x + z1y*zy + z1y*z2y;
		float Nim = -zy*z2x - z1x*zy - z1x*z2y + zx*z2y - z1y*zx + 2*zx*zy + z1y*z2x;

		float Dre = zx*zx - zx*p2x - zy*zy + zy*p2y - p1x*zx + p1x*p2x + p1y*zy - p1y*p2y;
		float Dim = -zx*p2y - p1y*zx + p1x*p2y + p1y*p2x - zy*p2x + 2*zy*zx - p1x*zy;

		float denom = 1.0f / (Dre*Dre + Dim*Dim);

		float real = (Nre*Dre + Nim*Dim) * denom;
		float imag = (Nim*Dre - Nre*Dim) * denom;

		float magnitude = sqrtf(real*real + imag*imag);

		if (normalize)
		{
			// Multiply by a0 if needed
			magnitude *= m_normalization;
		}

		return magnitude;
	}
};

class LowPassIIR : public FilterIIR_2p2z
{
public:

	LowPassIIR():
		FilterIIR_2p2z()
	{
	}

	void setParams(float freqNormalized)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		m_cutoffFreq = freqNormalized;
		float phi = 2 * PI * freqNormalized;

		float r;
		// For better lowpass functionality, it is better to use elliptic design
		// i.e. change R so that in the [freq 0] it is ~0.9, in [freq pi/2] it is ~0.4
		// and in [freq pi] it is back to ~0.9
		r = 0.9f * (1.0f - sinf(phi) / 2.0f);

		m_z1[0] = -1.0f;
		m_z1[1] =  0.0f;
		m_z2[0] = -1.0f;
		m_z2[1] =  0.0f;

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		float normGain = 1.0f;
		if (freqNormalized > 0.25f)
		{
			normGain += (freqNormalized - 0.25f) * 7.5f;
		}
		normGain *= 1.45f;

		float normalization = 0.25f * normGain * ( (1 - r) * sqrtf(r * (r - 4.0f * cosf(phi) + 2.0f) + 1.0f) );
		// Although this one calculated analytically (gives lower amplitudes on high cut-off freq)
		//float normalization = 0.25f * normGain * ( r*r - 2*r + 1 );

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] = 1.0f;
		m_baseCoeffI[1] = 2.0f;
		m_baseCoeffI[2] = 1.0f;
		coeffI[0] = m_baseCoeffI[0] * normalization;
		coeffI[1] = m_baseCoeffI[1] * normalization;
		coeffI[2] = m_baseCoeffI[2] * normalization;
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	void setParams(float freqNormalized, float r, float normGain = 1.0f)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		m_cutoffFreq = freqNormalized;
		float phi = 2 * PI * freqNormalized;
		float normalization = 0.25f * normGain * ( (1 - r) * sqrtf(r * (r - 4.0f * cosf(phi) + 2.0f) + 1.0f) );
		// Although this one calculated analytically (gives lower amplitudes on high cut-off freq)
		//float normalization = 0.25f * normGain * ( r*r - 2*r + 1 );

		m_z1[0] = -1.0f;
		m_z1[1] =  0.0f;
		m_z2[0] = -1.0f;
		m_z2[1] =  0.0f;

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] = 1.0f;
		m_baseCoeffI[1] = 2.0f;
		m_baseCoeffI[2] = 1.0f;
		coeffI[0] = m_baseCoeffI[0] * normalization;
		coeffI[1] = m_baseCoeffI[1] * normalization;
		coeffI[2] = m_baseCoeffI[2] * normalization;
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	static LowPassIIR & getInstance()
	{
		static LowPassIIR lowPassIIR;
		return lowPassIIR;
	}
};

class HighPassIIR : public FilterIIR_2p2z
{
public:

	HighPassIIR():
		FilterIIR_2p2z()
	{
	}

	void setParams(float freqNormalized)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		float phi = 2 * PI * freqNormalized;

		float r;
		// For better lowpass functionality, it is better to use elliptic design
		// i.e. change R so that in the [freq 0] it is ~0.9, in [freq pi/2] it is ~0.4
		// and in [freq pi] it is back to ~0.9
		r = 0.9f * (1.0f - sinf(phi) / 2.0f);

		m_z1[0] =  1.0f;
		m_z1[1] =  0.0f;
		m_z2[0] =  1.0f;
		m_z2[1] =  0.0f;

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		m_cutoffFreq = freqNormalized;

		// This normalization fits better
		float normGain = 1.0f;
		if (r > 0.9f)
			normGain += (r - 0.9f) * 100.0f;
		if (freqNormalized < 0.25f)
		{
			float gain = (0.25f - freqNormalized) * 10.0f;
			if (freqNormalized < 0.1f)
				gain *= 40.0f * (0.1f - freqNormalized);
			normGain += gain;
		}
		else
		{
			float gain = 1.0f + (freqNormalized - 0.25f) * 7.0f;
			normGain /= gain;
		}

		float normalization = -0.25f * normGain * ( (1 - r) * sqrtf(r * (r + 6.0f) + 1.0f) );

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] =  1.0f;
		m_baseCoeffI[1] = -2.0f;
		m_baseCoeffI[2] =  1.0f;
		coeffI[0] = m_baseCoeffI[0] * normalization;
		coeffI[1] = m_baseCoeffI[1] * normalization;
		coeffI[2] = m_baseCoeffI[2] * normalization;
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	void setParams(float freqNormalized, float r, float normGain = 1.0f)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		float phi = 2 * PI * freqNormalized;

		m_z1[0] =  1.0f;
		m_z1[1] =  0.0f;
		m_z2[0] =  1.0f;
		m_z2[1] =  0.0f;

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		m_cutoffFreq = freqNormalized;

		float normalization = -0.25f * normGain * ( (1 - r) * sqrtf(r * (r + 6.0f) + 1.0f) );

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] =  1.0f;
		m_baseCoeffI[1] = -2.0f;
		m_baseCoeffI[2] =  1.0f;
		coeffI[0] = m_baseCoeffI[0] * normalization;
		coeffI[1] = m_baseCoeffI[1] * normalization;
		coeffI[2] = m_baseCoeffI[2] * normalization;
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	static HighPassIIR & getInstance()
	{
		static HighPassIIR highPassIIR;
		return highPassIIR;
	}
};

class BandPassIIR : public FilterIIR_2p2z
{
public:

	BandPassIIR():
		FilterIIR_2p2z()
	{
	}

	void setParams(float freqNormalized)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		m_cutoffFreq = freqNormalized;
		float phi = 2 * PI * freqNormalized;

		float r;
		// For better lowpass functionality, it is better to use elliptic design
		// i.e. change R so that in the [freq 0] it is ~0.9, in [freq pi/2] it is ~0.4
		// and in [freq pi] it is back to ~0.9
		r = 0.9f * (1.0f - sinf(phi) / 2.0f);

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		m_z1[0] =  m_p1[0];
		m_z1[1] =  0.0f;
		m_z2[0] = -m_p1[0];
		m_z2[1] =  0.0f;

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] = 1.0f;
		m_baseCoeffI[1] = -(m_z1[0] + m_z2[0]);
		m_baseCoeffI[2] = m_z1[0] * m_z2[0];
		coeffI[0] = m_baseCoeffI[0];
		coeffI[1] = m_baseCoeffI[1];
		coeffI[2] = m_baseCoeffI[2];
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	void setParams(float freqNormalized, float r, float normGain = 1.0f)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		m_cutoffFreq = freqNormalized;
		float phi = 2 * PI * freqNormalized;

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		m_z1[0] =  m_p1[0];
		m_z1[1] =  0.0f;
		m_z2[0] = -m_p1[0];
		m_z2[1] =  0.0f;

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] = 1.0f;
		m_baseCoeffI[1] = -(m_z1[0] + m_z2[0]);
		m_baseCoeffI[2] = m_z1[0] * m_z2[0];
		coeffI[0] = m_baseCoeffI[0];
		coeffI[1] = m_baseCoeffI[1];
		coeffI[2] = m_baseCoeffI[2];
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	static BandPassIIR & getInstance()
	{
		static BandPassIIR bandPassIIR;
		return bandPassIIR;
	}
};

class NotchIIR : public FilterIIR_2p2z
{
public:

	NotchIIR():
		FilterIIR_2p2z()
	{
	}

	void setParams(float freqNormalized)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		m_cutoffFreq = freqNormalized;
		float phi = 2 * PI * freqNormalized;

		float r;
		// For better lowpass functionality, it is better to use elliptic design
		// i.e. change R so that in the [freq 0] it is ~0.9, in [freq pi/2] it is ~0.4
		// and in [freq pi] it is back to ~0.9
		r = 0.9f * (1.0f - sinf(phi) / 2.0f);

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		m_z1[0] = (r + 0.2f) * cosf(phi);
		m_z1[1] = (r + 0.2f) * sinf(phi);
		m_z2[0] =  m_z1[0];
		m_z2[1] = -m_z1[1];

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] = 1.0f;
		m_baseCoeffI[1] = -2 * m_z1[0];
		m_baseCoeffI[2] = m_z1[0]*m_z1[0] + m_z1[1]*m_z1[1];
		coeffI[0] = m_baseCoeffI[0];
		coeffI[1] = m_baseCoeffI[1];
		coeffI[2] = m_baseCoeffI[2];
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	void setParams(float freqNormalized, float r, float normGain = 1.0f)
	{
		if (freqNormalized > 0.5f)
			freqNormalized = 0.5f;

		m_cutoffFreq = freqNormalized;
		float phi = 2 * PI * freqNormalized;

		m_p1[0] = r * cosf(phi);
		m_p1[1] = r * sinf(phi);
		m_p2[0] =  m_p1[0];		// conjugate
		m_p2[1] = -m_p1[1];		// pair

		float zeroR = clamp<float>(r + 1.0f, 0.0f, 1.0f);

		m_z1[0] = zeroR*cosf(phi);
		m_z1[1] = zeroR*sinf(phi);
		m_z2[0] =  m_z1[0];
		m_z2[1] = -m_z1[1];

		float * coeffI = getCoefI();
		float * coeffO = getCoefO();

		m_baseCoeffI[0] = 1.0f;
		m_baseCoeffI[1] = -2.0f * m_z1[0];
		m_baseCoeffI[2] = m_z1[0]*m_z1[0] + m_z1[1]*m_z1[1];
		coeffI[0] = m_baseCoeffI[0];
		coeffI[1] = m_baseCoeffI[1];
		coeffI[2] = m_baseCoeffI[2];
		coeffO[0] = 2 * r * cosf(phi);
		coeffO[1] = -(r * r);
	}

	static NotchIIR & getInstance()
	{
		static NotchIIR notchIIR;
		return notchIIR;
	}
};

}
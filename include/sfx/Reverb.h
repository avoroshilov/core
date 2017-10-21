#pragma once

#include "helpers/common.h"
#include "math/AuxMath.h"

namespace sfx
{

// 4-channel Feedback Delay Network 

// The code is based on
//	1. Christian Schueler's article
//	2. Julius Smith and Nelson Lee RealSimple Project materials
//	3. John Stautner and Miller Puckett, "Designing Multi-Channel Reverberator"

struct ReverbMatrixTypes
{
	enum Values
	{
		// Householder reflection matrix permutations
		eHouseholder0 = 0,
		eHouseholder1,
		eHouseholder2,
		eHouseholder3,
		eHouseholder4,
		eHouseholder5,
		eHouseholder6,
		eHouseholder7,

		eIdentity,
		eTinyRotation,
		eLittleRotation,
		eMediumRotation,

		eNegIdentity,
		eNegTinyRotation,
		eNegLittleRotation,
		eNegMediumRotation,
		
		// J. Stautner and M. Puckett feedback matrix permutations
		eStautner0,
		eStautner1,
		eStautner2,
		eStautner3,

		// Hadamard (H4) matrix permutations
		eHadamard0,
		eHadamard1,
		eHadamard2,
		eHadamard3,

		LAST_VAL
	};
};

static const scalar g_reverbMatrices[][4][4] = 
{
	// eHouseholder0
	{
		{  1, -1, -1, -1 },		// 0
		{ -1,  1, -1, -1 },		// 1
		{ -1, -1,  1, -1 },		// 2
		{ -1, -1, -1,  1 },		// 3
	},

	// eHouseholder1
	{
		{ -1,  1, -1, -1, },	// 1
		{ -1, -1,  1, -1, },	// 2
		{ -1, -1, -1,  1, },	// 3
		{  1, -1, -1, -1, },	// 0
	},

	// eHouseholder2
	{
		{ -1, -1,  1, -1, },	// 2
		{ -1, -1, -1,  1, },	// 3
		{  1, -1, -1, -1, },	// 0
		{ -1,  1, -1, -1, },	// 1
	},

	// eHouseholder3
	{
		{ -1, -1, -1,  1, },	// 3
		{  1, -1, -1, -1, },	// 0
		{ -1,  1, -1, -1, },	// 1
		{ -1, -1,  1, -1, },	// 2
	},

	// eHouseholder4
	{
		{ -1,  1, -1, -1, },	// 1
		{  1, -1, -1, -1, },	// 0
		{ -1, -1, -1,  1, },	// 3
		{ -1, -1,  1, -1, },	// 2
	},

	// eHouseholder5
	{
		{  1, -1, -1, -1, },	// 0
		{ -1, -1,  1, -1, },	// 2
		{ -1,  1, -1, -1, },	// 1
		{ -1, -1, -1,  1, },	// 3
	},

	// eHouseholder6
	{
		{ -1, -1,  1, -1, },	// 2
		{  1, -1, -1, -1, },	// 0
		{ -1, -1, -1,  1, },	// 3
		{ -1,  1, -1, -1, },	// 1
	},

	// eHouseholder7
	{
		{  1,  1, -1,  1, },	// inv(2)
		{ -1,  1,  1,  1, },	// inv(0)
		{  1,  1,  1, -1, },	// inv(3)
		{  1, -1,  1,  1, },	// inv(1)
	},

	// eIdentity
	{
		{ 1,  0,  0,  0, },
		{ 0,  1,  0,  0, },
		{ 0,  0,  1,  0, },
		{ 0,  0,  0,  1, },
	},

	// eTinyRotation
	{
		{ 0.980416164f, -0.121205295f, -0.121205295f, -0.096964236f, },
		{ 0.096964236f,  0.983109615f, -0.121205295f, -0.096964236f, },
		{ 0.121205295f,  0.096964236f,  0.980416164f, -0.121205295f, },
		{ 0.121205295f,  0.096964236f,  0.096964236f,  0.983109615f, },
	},

	// eLittleRotation
	{
		{ 0.8271971909f, -0.3741443293f, -0.3741443293f, -0.1891474501f, },
		{ 0.1891474501f,  0.8879540606f, -0.3741443293f, -0.1891474501f, },
		{ 0.3741443293f,  0.1891474501f,  0.8271971909f, -0.3741443293f, },
		{ 0.3741443293f,  0.1891474501f,  0.1891474501f,  0.8879540606f, },
	},

	// eMediumRotation
	{
		{ 0.4027342996f, -0.7482592774f, -0.5147750269f, -0.1136640546f, },
		{ 0.1136640546f,  0.6111487307f, -0.7482592774f, -0.2317012589f, },
		{ 0.5147750269f,  0.1136640546f,  0.4027342996f, -0.7482592774f, },
		{ 0.7482592774f,  0.2317012589f,  0.1136640546f,  0.6111487307f, },
	},

	// eNegIdentity
	{
		{ -1,  0,  0,  0, },
		{  0, -1,  0,  0, },
		{  0,  0, -1,  0, },
		{  0,  0,  0, -1, },
	},

	// eNegTinyRotation
	{
		{ -0.980416164f,  0.121205295f,  0.121205295f,  0.096964236f, },
		{ -0.096964236f, -0.983109615f,  0.121205295f,  0.096964236f, },
		{ -0.121205295f, -0.096964236f, -0.980416164f,  0.121205295f, },
		{ -0.121205295f, -0.096964236f, -0.096964236f, -0.983109615f, },
	},

	// eNegLittleRotation
	{
		{ -0.8271971909f,  0.3741443293f,  0.3741443293f,  0.1891474501f, },
		{ -0.1891474501f, -0.8879540606f,  0.3741443293f,  0.1891474501f, },
		{ -0.3741443293f, -0.1891474501f, -0.8271971909f,  0.3741443293f, },
		{ -0.3741443293f, -0.1891474501f, -0.1891474501f, -0.8879540606f, },
	},

	// eNegMediumRotation
	{
		{ -0.4027342996f,  0.7482592774f,  0.5147750269f,  0.1136640546f, },
		{ -0.1136640546f, -0.6111487307f,  0.7482592774f,  0.2317012589f, },
		{ -0.5147750269f, -0.1136640546f, -0.4027342996f,  0.7482592774f, },
		{ -0.7482592774f, -0.2317012589f, -0.1136640546f, -0.6111487307f, },
	},

	// eStautner0
	{
		{  0,  1,  1,  0, },	// 0
		{ -1,  0,  0, -1, },	// 1
		{  1,  0,  0, -1, },	// 2
		{  0,  1, -1,  0, },	// 3
	},

	// eStautner1
	{
		{  0,  1,  1,  0, },	// 0
		{ -1,  0,  0, -1, },	// 1
		{  0,  1, -1,  0, },	// 3
		{  1,  0,  0, -1, },	// 2
	},

	// eStautner2
	{
		{  1,  0,  1,  0, },
		{  0, -1,  0, -1, },
		{  1,  0, -1,  0, },
		{  0,  1,  0, -1, },
	},

	// eStautner3
	{
		{  1,  0,  1,  0, },
		{  0, -1,  0, -1, },
		{  0,  1,  0, -1, },
		{  1,  0, -1,  0, },
	},

	// eHadamard0
	{
		{  1,  1,  1,  1, },
		{ -1,  1, -1,  1, },
		{ -1, -1,  1,  1, },
		{  1, -1, -1,  1, },
	},

	// eHadamard1
	{
		{  1,  1,  1,  1, },
		{  1, -1,  1, -1, },
		{  1,  1, -1, -1, },
		{  1, -1, -1,  1, },
	},

	// eHadamard2
	{
		{  1, -1, -1,  1, },
		{  1,  1, -1, -1, },
		{  1, -1,  1, -1, },
		{  1,  1,  1,  1, },
	},

	// eHadamard3
	{
		{  1, -1, -1,  1, },
		{ -1, -1,  1,  1, },
		{ -1,  1, -1,  1, },
		{  1,  1,  1,  1, },
	},

};

// Ideally, the normalization could be incorporated pre-compile time
static const scalar g_reverbNormalize[] = 
{
	// 1/2 for Householder reflections
	.5f, .5f, .5f, .5f, .5f, .5f, .5f, .5f,

	// 1 for identities and rotations
	1, 1, 1, 1, 1, 1, 1, 1,

	// 1/sqrt(2) for Stautner matrices
	0.70710678f, 0.70710678f, 0.70710678f, 0.70710678f,

	// 1/2 for Hadamard matrices
	0.5f, 0.5f, 0.5f, 0.5f,
};

// TODO:
//	The code could be unified with the rest of DSP framework by using DelayLineFrac,
//	and LFO-driven BandPass/LowPass filters. But it will require re-tweaking
//	make modulation take varying sample rate into account

#define NUM_FDN_CHANNELS 4

class ReverbFDN4
{
protected:

	struct DelayLine
	{
		scalar * data;								// Memory buffer for delay line
		uint memSize;								// Size of memory buffer
		uint length;								// Length of delay line
		uint bitMask;								// Bit-mask for faster residual mapping
		uint writeIdx;								// Write cursor
		uint readIdx;								// Read cursor
	};

	DelayLine m_delayLines[NUM_FDN_CHANNELS];

	// Feedback
	scalar m_A[NUM_FDN_CHANNELS][NUM_FDN_CHANNELS];	// Feedback matrix
	scalar m_iG[2][NUM_FDN_CHANNELS];				// Input mix/gain matrix
	scalar m_oG[NUM_FDN_CHANNELS][2];				// Output mix/gain matrix
	scalar m_p[NUM_FDN_CHANNELS];					// Partial read cursor
	scalar m_dp[NUM_FDN_CHANNELS];					// Velocity
	scalar m_g[NUM_FDN_CHANNELS];					// Scalar feedback gains
	scalar m_beta[NUM_FDN_CHANNELS];				// Extra feedback gains for high freq
	scalar m_alpha[NUM_FDN_CHANNELS];				// Filter alphas
	scalar m_filter[NUM_FDN_CHANNELS];				// Filter state

	// Mixing
	scalar m_dry;									// Dry coeff
	scalar m_wet;									// Wet coeff

	// Sweeping modulation
	int m_sweepCounter;								// Samples remaining until next modulation update
	int m_sweepInterval;							// Number of samples between modulation updates
	scalar m_sweepAmp;								// Use this fraction of the delay lengths for modulation
	scalar m_sweepPhase;							// LFO sweep phase offset

public:

	ReverbFDN4()		
	{
		for (uint i = 0; i < NUM_FDN_CHANNELS; ++i)
		{
			m_delayLines[i].data = 0;
		}
	}

	void init()
	{
		for (uint i = 0; i < NUM_FDN_CHANNELS; ++i)
		{
			m_p[i] = 0.0f;
			m_dp[i] = 0.0f;
			m_g[i] = 0.0f;

			m_beta[i] = 0.0f;
			m_alpha[i] = 0.0f;
			m_filter[i] = 0.0f;
		}

		m_dry = 0.0f;
		m_wet = 0.0f;

		m_sweepCounter = 0;
		m_sweepInterval = 0;
		m_sweepAmp = 0.0f;
		m_sweepPhase = 0.0f;
	}

	void initDLs(uint lengths[NUM_FDN_CHANNELS])
	{
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			m_delayLines[fdnCh].length = lengths[fdnCh];

			// We want to allocate memory enough for delay line and
			// additional half-length for sweeping
			const uint targetMemSize = 3 * m_delayLines[fdnCh].length / 2;

			// In order to speed up delay line wrapping, we want to use power-of-2 memory sizes
			m_delayLines[fdnCh].memSize = 1;
			while (m_delayLines[fdnCh].memSize < targetMemSize) 
				m_delayLines[fdnCh].memSize <<= 1;

			// We're using PO2 memory sizes, hence to wrap around we can use "and"
			m_delayLines[fdnCh].bitMask = m_delayLines[fdnCh].memSize - 1;

			m_delayLines[fdnCh].data = new scalar[m_delayLines[fdnCh].memSize];
		}
		clearDLs();
	}
	void clearDLs()
	{
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			for (uint j = 0; j < m_delayLines[fdnCh].memSize; ++j)
				m_delayLines[fdnCh].data[j] = 0.0f;

			// Set write pointer to 0, and read pointer in a way it will reach 0 in "DL length" samples
			m_delayLines[fdnCh].writeIdx = 0;
			m_delayLines[fdnCh].readIdx = m_delayLines[fdnCh].memSize - m_delayLines[fdnCh].length;
		}
	}
	void deinitDLs()
	{
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			if (m_delayLines[fdnCh].data)
			{
				delete [] m_delayLines[fdnCh].data;
				m_delayLines[fdnCh].data = 0;
			}
		}
	}

	void setHalflives(scalar halflife_LF, scalar halflife_HF)
	{
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			m_g[fdnCh] = powf(2, -(scalar)(m_delayLines[fdnCh].length) / halflife_LF);
			m_beta[fdnCh] = powf(2, -(scalar)(m_delayLines[fdnCh].length) / halflife_HF) / m_g[fdnCh];
			m_alpha[fdnCh] = 2.0f * m_beta[fdnCh] / (1.0f + m_beta[fdnCh]);
		}
	}

	void setFeedbackMatrix(ReverbMatrixTypes::Values matrixType)
	{
		for (uint i = 0; i < NUM_FDN_CHANNELS; ++i)
			for (uint j = 0; j < NUM_FDN_CHANNELS; ++j)
				m_A[i][j] = g_reverbNormalize[matrixType] * g_reverbMatrices[matrixType][i][j];
	}

	void setInOutRotations(scalar inDeg, scalar outDeg)
	{
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			scalar angOffsetDeg = fdnCh * 90.0f;

			m_iG[0][fdnCh] = sinf(Deg2RadMul * (inDeg + angOffsetDeg));
			m_iG[1][fdnCh] = cosf(Deg2RadMul * (inDeg + angOffsetDeg));

			m_oG[fdnCh][0] = 0.5f * sinf(Deg2RadMul * (outDeg + angOffsetDeg));
			m_oG[fdnCh][1] = 0.5f * cosf(Deg2RadMul * (outDeg + angOffsetDeg));
		}
	}

	void setMixCoeffs(scalar dryCoeff, scalar wetCoeff)
	{
		m_dry = dryCoeff;
		m_wet = wetCoeff;
	}

	// period - sweep interval, mapped 2pi -> 1000 samples 
	// depth - percentage of the delay line fraction (less than 50%)
	void setModulation(scalar period, scalar depthPercent)
	{
		m_sweepInterval = (int)(1000.f * (period / _2PI));

		if (m_sweepInterval == 0)
		{
			m_sweepCounter = 0;
			m_sweepAmp = 0.0f;
			m_sweepPhase = 0.0f;
		}
		else
		{
			m_sweepCounter = 0;
			m_sweepAmp = 0.01f * depthPercent;

			if (m_sweepAmp > 0.499f)
				m_sweepAmp = 0.499f;

			m_sweepPhase = 0.0f;
		}

		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			m_p[fdnCh] = 0;
			m_dp[fdnCh] = 0;
		}
	}

	void processCont(scalar inL, scalar inR, scalar * outL, scalar * outR)
	{
		// LFO-driven delay line filter
		if (m_sweepAmp && m_sweepCounter == 0)
		{    
			m_sweepCounter = m_sweepInterval;
			m_sweepPhase += 1.0f;

			const scalar reverbPhaseTable[] = { 0.0f, _PI2, PI, 1.5f * PI };
			for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
			{
				// Calculate actual and new delay length
				scalar actLen = (scalar)(( m_delayLines[fdnCh].memSize + m_delayLines[fdnCh].writeIdx - m_delayLines[fdnCh].readIdx ) & m_delayLines[fdnCh].bitMask);        
				scalar newLen = (scalar)(m_delayLines[fdnCh].length) * (1 + m_sweepAmp * ( sinf(m_sweepPhase + reverbPhaseTable[fdnCh]) ));

				// Calculate sweeping velocity: 
				// reach the new length in sweepCounter number of samples
				m_dp[fdnCh] = -(newLen - actLen) / (scalar)(m_sweepCounter);

				// Boost the high frequency gain dependent of the sweeping velocity
				// saturate at a factor of 1.75, but stay proportional to dp^2 for very small dp
				scalar dp2 = m_dp[fdnCh] * m_dp[fdnCh];
				scalar newBeta = m_beta[fdnCh] * (1.0f + 0.7f * dp2 / (1e-6f + dp2));
				m_alpha[fdnCh] = 2.0f * newBeta / (1.0f + newBeta);
			}
		}
		m_sweepCounter--;

		scalar delayLineSample[NUM_FDN_CHANNELS];
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			// Add velocity, update DL read index if position changes
			m_p[fdnCh] += m_dp[fdnCh];
			float posInt = floorf(m_p[fdnCh]);
			m_delayLines[fdnCh].readIdx = (m_delayLines[fdnCh].readIdx + (int)posInt) & m_delayLines[fdnCh].bitMask;
			m_p[fdnCh] -= posInt;

			// Read DL value, update read index
			float sample = m_delayLines[fdnCh].data[m_delayLines[fdnCh].readIdx];
			m_delayLines[fdnCh].readIdx = (m_delayLines[fdnCh].readIdx + 1) & m_delayLines[fdnCh].bitMask;
			// Linear interpolation in position fraction
			delayLineSample[fdnCh] = sample + m_p[fdnCh] * (m_delayLines[fdnCh].data[m_delayLines[fdnCh].readIdx] - sample);
		}

		// Apply output matrix: output = delayLineSample * D
		//	delay lines convergence/sync point
		scalar outputL = delayLineSample[0] * m_oG[0][0] + delayLineSample[1] * m_oG[1][0] + delayLineSample[2] * m_oG[2][0] + delayLineSample[3] * m_oG[3][0];
		scalar outputR = delayLineSample[0] * m_oG[0][1] + delayLineSample[1] * m_oG[1][1] + delayLineSample[2] * m_oG[2][1] + delayLineSample[3] * m_oG[3][1];

		scalar w[NUM_FDN_CHANNELS];
		for (uint fdnCh = 0; fdnCh < NUM_FDN_CHANNELS; ++fdnCh)
		{
			// Apply feedback and input matrices: w = A * r + C * input
			w[fdnCh] = delayLineSample[0] * m_A[0][fdnCh] + delayLineSample[1] * m_A[1][fdnCh] + delayLineSample[2] * m_A[2][fdnCh] + delayLineSample[3] * m_A[3][fdnCh];
			w[fdnCh] += inL * m_iG[0][fdnCh] + inR * m_iG[1][fdnCh];

			// Apply filter and gain: w = g * filter(w); put the data into the delay line
			w[fdnCh] = m_g[fdnCh] * (m_filter[fdnCh] += m_alpha[fdnCh] * (w[fdnCh] - m_filter[fdnCh]));
			m_delayLines[fdnCh].data[m_delayLines[fdnCh].writeIdx] = w[fdnCh];

			// Update DL write index
			m_delayLines[fdnCh].writeIdx = (m_delayLines[fdnCh].writeIdx + 1) & m_delayLines[fdnCh].bitMask;
		}

		// Mix output samples
		*outL = m_wet * outputL + m_dry * inL;
		*outR = m_wet * outputR + m_dry * inR;
	}

	void processStereoInterleaved(scalar * buffer, uint numSamples)
	{
		scalar * pBuf = buffer;
		for (uint i = 0; i < numSamples; ++i)
		{
			scalar inputL = pBuf[0];
			scalar inputR = pBuf[1];
			processCont(inputL, inputR, pBuf, pBuf+1);
			pBuf += 2;
		}
	}

};

}
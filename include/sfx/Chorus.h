#pragma once

#include "math/AuxMath.h"
#include "sfx/Delay.h"

namespace sfx
{

class Chorus
{
protected:

	scalar m_sampleFreq;

	uint m_numVoices;

	scalar m_feedbackGain;					// Feedback mul [0..1)
	scalar m_depth;							// Phaser depth (filtered signal gain, 0.0 to get dry sound)

	scalar * m_rangeMin, * m_rangeMax;		// Allpass delay range [0..22050) [Hz]
	scalar * m_LFOInc;						// Allpass delay modulation LFO freq [0..1]
	scalar * m_LFOPhase, * m_LFOPhaseInit;	// Allpass delay modulation LFO initial phase [0..1]
	DelayFrac ** m_delayLines;

public:
	Chorus():
		m_feedbackGain(0.7f),
		m_depth(1.0f),
		m_sampleFreq(44100.0f),
		m_numVoices(4),
		m_LFOPhase(0),
		m_LFOPhaseInit(0),
		m_LFOInc(0),
		m_delayLines(0),
		m_rangeMin(0),
		m_rangeMax(0)
	{
	}

	void init()
	{
		// TODO: use single chunk of memory
		if (m_LFOPhase != 0)
		{
			delete [] m_rangeMin;
			delete [] m_rangeMax;
			delete [] m_LFOPhase;
			delete [] m_LFOPhaseInit;
			delete [] m_LFOInc;
			for (uint i = 0; i < m_numVoices; ++i)
			{
				delete m_delayLines[i];
			}
			delete [] m_delayLines;
		}
		m_rangeMin = new scalar [m_numVoices];
		m_rangeMax = new scalar [m_numVoices];
		m_LFOPhase = new scalar [m_numVoices];
		m_LFOPhaseInit = new scalar [m_numVoices];
		m_LFOInc = new scalar [m_numVoices];
		m_delayLines = new DelayFrac*[m_numVoices];
		for (uint i = 0; i < m_numVoices; ++i)
		{
			m_delayLines[i] = new DelayFrac;
			m_delayLines[i]->setParams(4096, 0);
		}
	}
	void initPhases()
	{
		for (uint i = 0; i < m_numVoices; ++i)
		{
			m_LFOPhase[i] = m_LFOPhaseInit[i];
		}
	}

	void setNumVoices(uint numVoices)
	{
		if (m_delayLines != 0)
		{
			// Delete old delay lines
			for (uint i = 0; i < m_numVoices; ++i)
			{
				delete m_delayLines[i];
				m_delayLines[i] = 0;
			}
		}
		m_numVoices = numVoices;
	}
	uint getNumVoices() const { return m_numVoices; }

	void setPhase(uint idxVoice, scalar phase)
	{
		m_LFOPhaseInit[idxVoice] = _2PI * phase;
	}
	scalar getPhase(uint idxVoice) const { return m_LFOPhase[idxVoice] / _2PI; }

	void setRange(uint idxVoice, scalar fMin, scalar fMax)
	{
		m_rangeMin[idxVoice] = fMin;
		m_rangeMax[idxVoice] = fMax;
	}
	scalar getRangeMin(uint idxVoice) const { return m_rangeMin[idxVoice]; }
	scalar getRangeMax(uint idxVoice) const { return m_rangeMax[idxVoice]; }

	void setRate(uint idxVoice, scalar rate)
	{
		m_LFOInc[idxVoice] = _2PI * (rate / m_sampleFreq);
	}
	scalar getRate(uint idxVoice) const { return m_LFOInc[idxVoice] * (m_sampleFreq / _2PI); }

	void setFeedback(scalar fb)
	{
		m_feedbackGain = fb;
	}
	scalar getFeedback() const { return m_feedbackGain; }

	void setDepth(scalar depth)
	{
 		m_depth = depth;
	}
	scalar getDepth() const { return m_depth; }

	float update(scalar inSamp)
	{
#if 0
		const scalar voiceGain = 1.0f / mNumVoices;
		scalar voices = 0.0f;
		for (uint i = 0; i < mNumVoices; ++i)
#else
		const scalar voiceGain = 1.0f;
		scalar voices = 0.0f;
		uint i = 0;
#endif
		{
			// Calculate and update phaser sweep LFO
			scalar d = m_rangeMin[i] + (m_rangeMax[i] - m_rangeMin[i]) * ( (sinf(m_LFOPhase[i]) + 1.0f) / 2.0f );
			m_LFOPhase[i] += m_LFOInc[i];
			if (m_LFOPhase[i] >= _2PI)
    			m_LFOPhase[i] -= _2PI;

			m_delayLines[i]->resize(d);

			scalar delayOutput = m_delayLines[i]->getSample();
			m_delayLines[i]->setSample(inSamp);

			voices += voiceGain * delayOutput;
		}

		//return inSamp + voices * m_depth;
		return voices;
	}
};

}
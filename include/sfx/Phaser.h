#pragma once

#include "math/AuxMath.h"

#include "FiltersDelay.h"

namespace sfx
{

class Phaser
{
protected:

	uint m_allPassNum;
	AllpassDelay1p * m_allPass;

	scalar m_sampleFreq;

	scalar m_rangeMin, m_rangeMax;		// Allpass delay range [0..22050) [Hz]
	scalar m_feedbackGain;				// Feedback mul [0..1)
	scalar m_LFOPhase, m_LFOPhaseInit;	// Allpass delay modulation LFO initial phase [0..1]
	scalar m_LFOInc;					// Allpass delay modulation LFO freq [0..1]
	scalar m_depth;						// Phaser depth (filtered signal gain, 0.0 to get dry sound)

	scalar m_feedback;

public:
	Phaser():
		m_allPassNum(6),
		m_allPass(0),
		m_feedbackGain(0.7f),
		m_LFOPhaseInit(0.0f),
		m_depth(1.0f),
		m_sampleFreq(44100.0f)
	{
		setRange(440.0f, 1600.0f);
		setRate(0.5f);
	}

	void init()
	{
		m_feedback = 0.0f;
		m_LFOPhase = m_LFOPhaseInit;

		// TODO: boolean flag if number has changed, then delete and realloc
		if (m_allPass)
		{
			delete [] m_allPass;
			m_allPass = 0;
		}
		m_allPass = new AllpassDelay1p[m_allPassNum];

		for (uint i = 0; i < m_allPassNum; ++i)
		{
			m_allPass[i].init();
		}
	}

	void setAllpassNum(uint allpassNum)
	{
		m_allPassNum = allpassNum;
	}
	uint getAllpassNum() const { return m_allPassNum; }

	void setPhase(scalar phase)
	{
		m_LFOPhaseInit = _2PI * phase;
	}
	scalar getPhase() const { return m_LFOPhase / _2PI; }

	void setRange(scalar fMin, scalar fMax)
	{
		m_rangeMin = fMin / (m_sampleFreq / 2.0f);
		m_rangeMax = fMax / (m_sampleFreq / 2.0f);
	}
	scalar getRangeMin() const { return m_rangeMin * (m_sampleFreq / 2.0f); }
	scalar getRangeMax() const { return m_rangeMax * (m_sampleFreq / 2.0f); }

	void setRate(scalar rate)
	{
		m_LFOInc = _2PI * (rate / m_sampleFreq);
	}
	scalar getRate() const { return m_LFOInc * (m_sampleFreq / _2PI); }

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
		// Typical feedback-phaser. To turn it into the simple phaser, set FeedbackGain to 0.0
		// Uses series of all-pass filters (currently 1st order) on a signal
		// And then adds the signal to its all-pass filtered version
		// Producing in some sense comb-filter with irregularly spaced teeth 

		// Calculate and update phaser sweep LFO
		scalar d  = m_rangeMin + (m_rangeMax - m_rangeMin) * ( (sinf(m_LFOPhase) + 1.0f) / 2.0f );
		m_LFOPhase += m_LFOInc;
		if (m_LFOPhase >= _2PI)
    		m_LFOPhase -= _2PI;

		// Update filter coeffs and calculate output
		scalar y = inSamp + m_feedback * m_feedbackGain;
		for (uint i = 0; i < m_allPassNum; ++i)
		{
    		m_allPass[i].delay(d);
			y = m_allPass[i].update(y);
		}
		m_feedback = y;

		return inSamp + y * m_depth;
	}
};

}
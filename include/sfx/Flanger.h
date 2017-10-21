#pragma once

#include "Delay.h"

namespace sfx
{

class Flanger
{
protected:

	DelayFrac m_delayLine;

	scalar m_sampleFreq;

	scalar m_rangeMin, m_rangeMax;		// Allpass delay range [0..22050) [Hz]
	scalar m_feedbackGain;				// Feedback mul [0..1)
	scalar m_LFOPhase, m_LFOPhaseInit;	// Allpass delay modulation LFO initial phase [0..1]
	scalar m_LFOInc;					// Allpass delay modulation LFO freq [0..1]
	scalar m_depth;						// Phaser depth (filtered signal gain, 0.0 to get dry sound)

public:
	Flanger():
		m_feedbackGain(0.7f),
		m_LFOPhaseInit(0.0f),
		m_depth(1.0f),
		m_sampleFreq(44100.0f)
	{
		m_delayLine.setParams(4096, 0);
		setRange(440.0f, 1600.0f);
		setRate(0.5f);
	}

	void init()
	{
		m_LFOPhase = m_LFOPhaseInit;
	}

	void setPhase(scalar phase)
	{
		m_LFOPhaseInit = _2PI * phase;
	}
	scalar getPhase() const { return m_LFOPhase / _2PI; }

	void setRange(scalar fMin, scalar fMax)
	{
		m_rangeMin = fMin;
		m_rangeMax = fMax;
	}
	scalar getRangeMin() const { return m_rangeMin; }
	scalar getRangeMax() const { return m_rangeMax; }

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
		// Calculate and update phaser sweep LFO
		scalar d = m_rangeMin + (m_rangeMax - m_rangeMin) * ( (sinf(m_LFOPhase) + 1.0f) / 2.0f );
		m_LFOPhase += m_LFOInc;
		if (m_LFOPhase >= _2PI)
    		m_LFOPhase -= _2PI;

		m_delayLine.resize(d);

		scalar delayOutput = m_delayLine.getSample();
		m_delayLine.setSample(inSamp);

		return inSamp + delayOutput * m_depth;
	}
};

}
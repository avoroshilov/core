#pragma once

#include "Delay.h"

namespace sfx
{

class AllpassDelay1p
{
protected:
	scalar m_gain;
	scalar m_delayedSample;

public:
	AllpassDelay1p():
		m_gain(0.0f)
	{
	}

	void init()
	{
		m_delayedSample = 0.0f;
	}

	void delay(scalar delay) // Sample delay time [for all-pass fractional delay lines]
	{
		m_gain = (1.0f - delay) / (1.0f + delay);
	}
	void setGainRaw(scalar gain)
	{
		m_gain = gain;
	}

	scalar update(scalar inSamp)
	{
		// In comments - several different implementations of the all-pass feedback/feedforward

		/*

		// Supplemental information, how to calculate the phase and group delay of this kind of all-pass filter:

		// phi(w) = arg{ H(e^jw) }
		// 1st orded allpass has pole at p
		// phase_delay = -phi(w)/w;
		// group_delay = -dphi(w)/dw;

		// WARNING: this is not for the canonical form of the filter, hence aM is
		// actually a negative from the "canonical" aM (same as the FDN-thesis allpass)

		// freq - is the frequency at which we want to calculate the phase delay

		scalar omegaBase = freq * 2*PI / m_SamplesPerSec;
		scalar aM2 = aM*aM;
		scalar phase_delay = 1/omegaBase * atan2f( (1-aM2) * sinf(M*omegaBase), (1+aM2) * cosf(M*omegaBase) - 2*aM2 );
		scalar group_delay = (1-tuneGain2) / ( 1 - 2*tuneGain*cosf(omegaBase) + tuneGain2 );

		*/
		//////////////////////////////////////////////////////////////////////////

		/*
		// Canonical version, suggested by J. Smith in
		// "Computational Acoustic Modeling with Digital delay"
		
		// M - order of the filter (delay line length)
		// aM - filter gain coefficient
		// b0 = aM

		// Transfer function:
		//		    b0 + z^-M
		// H(z) = -------------
		//		  1 + aM * z^-M
		
		scalar delayOutput = m_delayedSample;

		scalar delayInput = inSamp + delayOutput * -m_gain;
		scalar feedForward = delayInput * m_gain;

		m_delayedSample = delayInput;

		return delayOutput + feedForward;
		*/
		//////////////////////////////////////////////////////////////////////////

		// This version is the negative from the above
		// it uses m_gain = -aM, since this way all-pass filter is active when m_gain = [0.0, 1.0)
		// in canonical implementation, aM should be negative for filter to work like desired

		scalar delayOutput = m_delayedSample;

		scalar delayInput = inSamp + delayOutput * m_gain;
		scalar feedForward = delayInput * -m_gain;

		m_delayedSample = delayInput;

		return delayOutput + feedForward;

		//////////////////////////////////////////////////////////////////////////
		/*
		// Implementation from random (a bit different from canonical) flowchart from the internet
		// Works as the one with m_gain = -aM

		scalar delayOutput = m_delayedSample;

		scalar feedForward = inSamp * -m_gain;
		scalar out = delayOutput + feedForward;
		scalar delayInput = inSamp + m_gain * out;

		m_delayedSample = delayInput;

		return out;

		// Well-known Phaser implementation on MusicDSP seems to use same approach:
		scalar y = inSamp * -mA1 + mZM1;
		mZM1 = y * mA1 + inSamp;
		return y;
		*/
	}
};

// Feed-forward comb filter, features richer frequency response
// J.O. Smith "Computational Acoustic Modeling with Digital delay"
class CombDelayFF
{
protected:

	// b0, bM:
	//	Gain is max (b0 + bM) when a whole number of periods fits in M samples
	//	Gain is min |b0 - bM| when an odd number of half-periods fits in M samples
	scalar m_b0, m_bM;

	// M is the order of the filter
	scalar m_M;

	DelayFrac m_delayLine;

public:

	CombDelayFF():
		m_b0(0.5f),
		m_bM(0.5f),
		m_M(5.0f)
	{
		m_delayLine.setParams(4096, 0);
	}

	void setGain(scalar gainMin, scalar gainMax)
	{
		m_b0 = (gainMin + gainMax) * 0.5f;
		m_bM = gainMax - m_b0;
	}
	void setGainRaw(scalar b0, scalar bM)
	{
		m_b0 = b0;
		m_bM = bM;
	}

	void init(scalar delaySamples)
	{
		m_M = delaySamples;
		m_delayLine.init(m_M);
	}

	scalar update(scalar inSamp)
	{
		scalar feedForward = inSamp * m_b0;
		scalar delaySample = m_delayLine.getSample();
		m_delayLine.setSample(inSamp);

		return delaySample * m_bM + feedForward;
	}
};

// Feed-back comb filter, features spiky frequency response
class CombDelayFB
{
protected:

	// b0, aM:
	// For simplified special case:
	//	Gain is max 1/(1-aM) when a whole number of periods fits in M samples
	//	Gain is min 1/(1+aM) when an odd number of half-periods fits in M samples
	// So, to normalize the filter, we want maxGain to be 1.0
	// => we need to set b0 to 1/maxGain;
	// => 1/(1-aM) = 1/b0
	// => b0 = 1 - aM;
	scalar m_b0, m_aM;

	// M is the order of the filter
	scalar m_M;

	DelayFrac m_delayLine;

public:

	CombDelayFB():
		m_aM(0.5f),
		m_b0(0.5f),
		m_M(5.0f)
	{
		m_delayLine.setParams(4096, 0);
	}

	void setGain(scalar gain)
	{
		m_aM = gain;
		m_b0 = 1.0f;
	}
	void setGain(scalar gain, scalar volume)
	{
		m_aM = gain;
		m_b0 = volume * (1.0f - m_aM);
	}

	void init(scalar delaySamples)
	{
		m_M = delaySamples;
		m_delayLine.init(m_M);
	}

	scalar update(scalar inSamp)
	{
		scalar delaySample = m_delayLine.getSample();
		scalar feedBack = delaySample * -m_aM;
		scalar delayInput = feedBack + inSamp;
		scalar feedForward = delayInput * m_b0;
		m_delayLine.setSample(delayInput);

		return feedForward;
	}
};

}
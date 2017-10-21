#pragma once

#include "helpers/Common.h"
#include "math/AuxMath.h"

namespace sfx
{

// Stereo storage interleaved

inline scalar oscSine(scalar freq, scalar amp, scalar initPhase, scalar time, scalar duty = 0.5f)
{
	float shift = (duty - 0.5f) * 2.0f;
	scalar val = amp * sinf(_2PI * freq * time + initPhase) + shift;

	if (val > 0.0f)
	{
		val /= 1.0f + shift;
	}
	else
	{
		val /= 1.0f - shift;
	}

	return val;
}

inline scalar oscSine(scalar phase, scalar duty = 0.5f)
{
	float shift = (duty - 0.5f) * 2.0f;
	scalar val = sinf(phase) + shift;

	if (val > 0.0f)
	{
		val /= 1.0f + shift;
	}
	else
	{
		val /= 1.0f - shift;
	}

	return val;
}

inline scalar oscTriangle(scalar phase, scalar duty = 0.5f)
{
	scalar clampedPhase = phase - (int)phase;
	if (clampedPhase > duty)
	{
		return 1.0f - 2.0f * (clampedPhase - duty) / (1.0f - duty);
	}
	else
	{
		return 2.0f * clampedPhase / duty - 1.0f;
	}
}
inline scalar oscTriangle(scalar freq, scalar amp, scalar initPhase, scalar time, scalar duty = 0.5f)
{
	initPhase += 0.5f * duty;

	scalar phase = freq * time + initPhase;
	scalar clampedPhase = phase - (int)phase;

	scalar val = amp;

	if (clampedPhase > duty)
	{
		val *= 1.0f - 2.0f * (clampedPhase - duty) / (1.0f - duty);
	}
	else
	{
		val *= 2.0f * clampedPhase / duty - 1.0f;
	}

	return val;
}

inline scalar oscSquare(scalar phase, scalar duty = 0.5f)
{
	scalar clampedPhase = phase - (int)phase;
	if (clampedPhase > duty)
	{
		return -1.0f;
	}
	else
	{
		return  1.0f;
	}
}
inline scalar oscSquare(scalar freq, scalar amp, scalar initPhase, scalar time, scalar duty = 0.5f)
{
	scalar phase = freq * time + initPhase;
	scalar clampedPhase = phase - (int)phase;

	scalar val = amp;
	if (clampedPhase > duty)
	{
		val *= -1.0f;
	}

	return val;
}

struct OscillatorMode
{
	enum Values
	{
		eSine = 0,
		eTriangle,
		eSawtooth,
		eSquare,

		eLAST_VAL
	};
};

template <OscillatorMode::Values oscMode>
void fillOsc(scalar freq, scalar amp, scalar initPhase, scalar * buf, uint len, uint sampleFreq)
{
	for (uint i = 0; i < len; ++i)
	{
		scalar time = i / (scalar)sampleFreq;
		scalar val;
		if		(oscMode == OscillatorMode::eSine)
		{
			val = oscSine(freq, amp, initPhase, time);
		}
		else if (oscMode == OscillatorMode::eTriangle)
		{
			val = oscTriangle(freq, amp, initPhase, time);
		}
		*buf++ = val;
	}
}
template <OscillatorMode::Values oscMode>
void fillOscStereo(scalar freq, scalar amp, scalar initPhase, scalar * buf, uint len, uint sampleFreq)
{
	for (uint i = 0; i < len; ++i)
	{
		scalar time = i / (scalar)sampleFreq;
		scalar val;
		if		(oscMode == OscillatorMode::eSine)
		{
			val = oscSine(freq, amp, initPhase, time);
		}
		else if (oscMode == OscillatorMode::eTriangle)
		{
			val = oscTriangle(freq, amp, initPhase, time);
		}
		*buf++ = val;
		*buf++ = val;
	}
}

}
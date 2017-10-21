#pragma once

#include <Windows.h>
#include <mmsystem.h>
#include <vector>

#include "math/AuxMath.h"
#include "helpers/Common.h"
#include "windows/WaveOut_Base.h"

#include "sfx/Filters.h"

#define FILTER_RECONFIG_FRAMES			20

// WARNING: Stereo only!

namespace windows
{

class WaveOutSfx : public WaveOutBase
{
protected:

	HANDLE m_mutex;

	uint m_totalBufLen, m_subBufLen;
	short * m_wholeBuf, ** m_subBufs;
	bool * m_isSubBufferValid;
	uint m_curSubBuf, m_totalSubBufs;
	double m_sampleIdxL, m_sampleIdxR;


	bool m_isPrevStateValid;
	scalar m_prevTimeScale;
	scalar m_prevIntensityL, m_prevIntensityR;
	scalar m_prevDelayL, m_prevDelayR;
	scalar m_prevLowPassFreqL, m_prevLowPassFreqR;

	// Changeable variables
	volatile scalar m_azimuth;
	volatile scalar m_timeScale;
	volatile scalar m_distanceFade;
	volatile scalar m_intensity;

	sfx::LowPassIIR m_lowPassL, m_lowPassR;

public:

	friend static DWORD WINAPI AudioSfxOutThreadProc(LPVOID lpParameter);

	WaveOutSfx::WaveOutSfx();
	WaveOutSfx::~WaveOutSfx();

	virtual bool WaveOutSfx::startThread();

	uint WaveOutSfx::advanceSubBuffer();

	bool WaveOutSfx::prepareSubBuffer();

	bool WaveOutSfx::playSubBuffer();

	bool WaveOutSfx::playSoundStereo(short * buf, uint uSize);

	void WaveOutSfx::setTimeScale(scalar ts);
	scalar WaveOutSfx::getTimeScale();

	void WaveOutSfx::setAzimuth(scalar a);
	scalar WaveOutSfx::getAzimuth();

	void WaveOutSfx::setDistanceFade(scalar df);
	scalar WaveOutSfx::getDistanceFade();

	void WaveOutSfx::setIntensity(scalar intensity);
	scalar WaveOutSfx::getIntensity();

	void WaveOutSfx::bufferAdd();
	void WaveOutSfx::bufferSub();

	int WaveOutSfx::getBufferNum();

	bool m_recordSamples = false;
	void setSampleRecordingState(bool recordSamples)
	{
		EnterCriticalSection(&m_criticalSection);
		m_recordSamples = recordSamples;
		m_samplesInterleaved.reserve(10000);
		LeaveCriticalSection(&m_criticalSection);
	}
	bool getSampleRecordingState()
	{
		bool recordSamples;
		EnterCriticalSection(&m_criticalSection);
		recordSamples = m_recordSamples;
		LeaveCriticalSection(&m_criticalSection);
		return recordSamples;
	}

	std::vector<short> m_samplesInterleaved;
	// 2 channels only
	void recordSamples(short * samples, uint numSamples)
	{
		EnterCriticalSection(&m_criticalSection);
		uint currentSamplesNum = (uint)m_samplesInterleaved.size();
		m_samplesInterleaved.resize(currentSamplesNum + numSamples);
		for (uint i = 0; i < numSamples; ++i)
		{
			m_samplesInterleaved[currentSamplesNum + i] = samples[i];
		}
		LeaveCriticalSection(&m_criticalSection);
	}
	void resetSamplesRecording()
	{
		EnterCriticalSection(&m_criticalSection);
		m_samplesInterleaved.resize(0);
		LeaveCriticalSection(&m_criticalSection);
	}
};

}

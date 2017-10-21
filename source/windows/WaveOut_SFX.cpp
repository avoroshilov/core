#pragma once

#include <Windows.h>
#include <mmsystem.h>
#include "math/AuxMath.h"
#include "helpers/Common.h"
#include "windows/WaveOut_Base.h"
#include "windows/WaveOut_SFX.h"

#include "sfx/Filters.h"

#define FILTER_RECONFIG_FRAMES			20

// WARNING: Stereo only!

namespace windows
{

WaveOutSfx::WaveOutSfx()
{
	m_azimuth = 0.0f;
	m_timeScale = 1.0f;
	m_distanceFade = 0.0f;
	m_intensity = 1.0f;

	m_prevIntensityL = 1.0f;
	m_prevIntensityR = 1.0f;

	m_prevDelayL = 0.0f;
	m_prevDelayR = 0.0f;

	m_prevLowPassFreqL = 0.5f;
	m_prevLowPassFreqR = 0.5f;

	m_prevTimeScale = 1.0f;

	m_subBufLen = SamplesPerSec / 50;	// 20ms
	m_wholeBuf = 0;

	m_curSubBuf = 0;
	m_totalSubBufs = 5;
	m_subBufs = (short **)malloc(m_totalSubBufs * sizeof(short *));
	m_isSubBufferValid = (bool *)malloc(m_totalSubBufs * sizeof(bool));
	for (uint i = 0; i < m_totalSubBufs; ++i)
	{
		m_subBufs[i] = (short *)malloc(2 * m_subBufLen * sizeof(short));
		m_isSubBufferValid[i] = false;
	}

	m_sampleIdxL = 0.0f;
	m_sampleIdxR = 0.0f;

	m_mutex = CreateMutex(0, false, 0);
}

WaveOutSfx::~WaveOutSfx()
{
	if (m_wholeBuf)
	{
		free(m_wholeBuf);
		m_wholeBuf = 0;
	}

	if (m_subBufs[0])
	{
		free(m_subBufs[0]);
		m_subBufs[0] = 0;
	}
	if (m_subBufs[1])
	{
		free(m_subBufs[1]);
		m_subBufs[1] = 0;
	}

	free(m_subBufs);
	free(m_isSubBufferValid);
}

bool WaveOutSfx::startThread()
{
	if (m_bThreadStart)
	{
		return false;
	}

	m_hAudioOut = CreateThread(0, 0, AudioSfxOutThreadProc, this, 0, &m_dwAudioOutId);
	if (!m_hAudioOut)
	{
		return false;
	}

	m_bThreadStart = true;
	return true;
}

uint WaveOutSfx::advanceSubBuffer()
{
	uint t_csb;

	EnterCriticalSection(&m_criticalSection);
	m_curSubBuf = (m_curSubBuf + 1)%m_totalSubBufs;
	t_csb = m_curSubBuf;
	LeaveCriticalSection(&m_criticalSection);

	return t_csb;
}

bool WaveOutSfx::prepareSubBuffer()
{
	// Settings
	const scalar ratioLR = 2.0f;								// ILD ratio: when the source is strictly to the left, L is X times stronger than R
	const scalar maxDelay = 0.00063f;							// ITD ratio: 0.63ms
	const scalar minFreqNormalized = 1600.0f / SamplesPerSec;	// Head shadow: 1500Hz during full-shadow

	bool isCurSubBufferValid = false;

	if ((m_sampleIdxL < m_totalBufLen - 1) || (m_sampleIdxR < m_totalBufLen - 1))
	{
		isCurSubBufferValid = true;
	}

	m_isSubBufferValid[m_curSubBuf] = isCurSubBufferValid;
	if (!isCurSubBufferValid)
	{
		return isCurSubBufferValid;
	}

	scalar timeScale;
	scalar azimuth;
	scalar distanceFade;
	scalar sndIntensity;

	EnterCriticalSection(&m_criticalSection);

	timeScale = m_timeScale;
	azimuth = m_azimuth;
	distanceFade = m_distanceFade;
	sndIntensity = m_intensity;

	LeaveCriticalSection(&m_criticalSection);

	scalar sineCoeffL = sinf( (0.5f - azimuth / PI) * 0.5f * PI );	// 1.0 when strictly L
	scalar sineCoeffR = sinf( (0.5f + azimuth / PI) * 0.5f * PI );	// 1.0 when strictly R
		
	// Determine IID here: max L/R ratio is 2.5
	scalar intensityL = 1.0f, intensityR = 1.0f;
	intensityL = distanceFade * sndIntensity * ( (1.0f - 1.0f / ratioLR) * sineCoeffL + (1.0f / ratioLR) );
	intensityR = distanceFade * sndIntensity * ( (1.0f - 1.0f / ratioLR) * sineCoeffR + (1.0f / ratioLR) );

	// Normalize intensity, otherwise localization perceived wrong
	const scalar thresholdIntensity = 2.0f;
	scalar maxIntensity = intensityL > intensityR ? intensityL : intensityR;
	if (maxIntensity > thresholdIntensity)
	{
		intensityL /= (maxIntensity / thresholdIntensity);
		intensityR /= (maxIntensity / thresholdIntensity);
	}

	// Determine ITD here
	scalar delayL, delayR;
	delayL = (1.0f - sineCoeffL) * maxDelay * (scalar)SamplesPerSec;
	delayR = (1.0f - sineCoeffR) * maxDelay * (scalar)SamplesPerSec;

	// TODO: determine low-pass param here 
	scalar lowPassFreqL, lowPassFreqR;
	lowPassFreqL = 0.5f - (0.5f - minFreqNormalized) * clamp( azimuth / PI * 2.0f, 0.0f, 1.0f);
	lowPassFreqR = 0.5f - (0.5f - minFreqNormalized) * clamp(-azimuth / PI * 2.0f, 0.0f, 1.0f);

	if (!m_isPrevStateValid)
	{
		m_prevDelayL = delayL;
		m_prevDelayR = delayR;
		m_prevIntensityL = intensityL;
		m_prevIntensityR = intensityR;
		m_prevLowPassFreqL = lowPassFreqL;
		m_prevLowPassFreqR = lowPassFreqR;
		m_prevTimeScale = timeScale;

		m_isPrevStateValid = true;
	}

	for (uint i = 0, iend = m_subBufLen; i < iend; ++i)
	{
		scalar interpParam = i / (scalar)m_subBufLen;
		scalar locIntensityL = clamp( m_prevIntensityL * (1.0f - interpParam) + intensityL * interpParam, 0.0f, 1.1f );
		scalar locIntensityR = clamp( m_prevIntensityR * (1.0f - interpParam) + intensityR * interpParam, 0.0f, 1.1f );
		scalar locDelayL = m_prevDelayL * (1.0f - interpParam) + delayL * interpParam;
		scalar locDelayR = m_prevDelayR * (1.0f - interpParam) + delayR * interpParam;
		scalar locTimeScale = m_prevTimeScale * (1.0f - interpParam) + timeScale * interpParam;

		if ((i % FILTER_RECONFIG_FRAMES) == 0)
		{
			scalar locLowPassFreqL = m_prevLowPassFreqL * (1.0f - interpParam) + lowPassFreqL * interpParam;
			scalar locLowPassFreqR = m_prevLowPassFreqR * (1.0f - interpParam) + lowPassFreqR * interpParam;

			m_lowPassL.setParams(locLowPassFreqL);
			m_lowPassL.autoNormalize();
			m_lowPassR.setParams(locLowPassFreqR);
			m_lowPassR.autoNormalize();
		}

		double delayedSampleIdxL = m_sampleIdxL - locDelayL;
		if (delayedSampleIdxL > 0.0 && delayedSampleIdxL < m_totalBufLen - 1)
		{
			int iSampleIdxL = (int)delayedSampleIdxL;
			scalar alpha = (scalar)(delayedSampleIdxL - iSampleIdxL);

			scalar sample;

#define INTERP_TYPE		1

#if (INTERP_TYPE == 0)
			// Linear (2 point, 1st order)
			short sample0 = m_wholeBuf[iSampleIdxL*2+0], sample1 = m_wholeBuf[(iSampleIdxL+1)*2+0];
			mLowPassL.ApplyCont(&sample, locIntensityL * (sample0 * (1.0f - alpha) + sample1 * alpha));
#elif (INTERP_TYPE == 1)
			short sample1 = m_wholeBuf[iSampleIdxL*2+0], sample2 = m_wholeBuf[(iSampleIdxL+1)*2+0];
			short sample0 = (iSampleIdxL > 0) ? m_wholeBuf[(iSampleIdxL-1)*2+0] : 0,
					sample3 = (iSampleIdxL < (int)(m_totalBufLen - 2)) ? m_wholeBuf[(iSampleIdxL+2)*2+0] : 0;
				
			// B-Spline (4 point, 3rd order)
			scalar ym1py1 = (scalar)(sample0 + sample2);
			scalar c0 = 1/6.0f*ym1py1 + 2/3.0f*sample1;
			scalar c1 = 1/2.0f*(sample2-sample0);
			scalar c2 = 1/2.0f*ym1py1 - sample1;
			scalar c3 = 1/2.0f*(sample1-sample2) + 1/6.0f*(sample3-sample0);

			m_lowPassL.applyCont(&sample, locIntensityL * ( ((c3*alpha+c2)*alpha+c1)*alpha+c0 ));
#elif (INTERP_TYPE == 2)
			short sample1 = m_wholeBuf[iSampleIdxL*2+0], sample2 = m_wholeBuf[(iSampleIdxL+1)*2+0];
			short sample0 = (iSampleIdxL > 0) ? m_wholeBuf[(iSampleIdxL-1)*2+0] : 0,
					sample3 = (iSampleIdxL < (int)(m_totalBufLen - 2)) ? m_wholeBuf[(iSampleIdxL+2)*2+0] : 0;

			// Hermit (4 point, 3rd order)
			scalar c0 = sample1;
			scalar c1 = 1/2.0f*(sample2-sample0);
			scalar c2 = sample0 - 5/2.0f*sample1 + 2*sample2 - 1/2.0f*sample3;
			scalar c3 = 1/2.0f*(sample3-sample0) + 3/2.0f*(sample1-sample2);

			mLowPassL.ApplyCont(&sample, locIntensityL * ( ((c3*alpha+c2)*alpha+c1)*alpha+c0 ));
#endif

			m_subBufs[m_curSubBuf][i*2+0] = (short)sample;
			m_sampleIdxL += locTimeScale;
		}
		else
		{
			// Advance if we're delayed
			if (delayedSampleIdxL < 0.0)
				m_sampleIdxL += locTimeScale;

			m_subBufs[m_curSubBuf][i*2+0] = 0;
		}

		double delayedSampleIdxR = m_sampleIdxR - locDelayR;
		if (delayedSampleIdxR > 0.0 && delayedSampleIdxR < m_totalBufLen - 1)
		{
			int iSampleIdxR = (int)delayedSampleIdxR;
			scalar alpha = (scalar)(delayedSampleIdxR - iSampleIdxR);

			scalar sample;
#if (INTERP_TYPE == 0)
			// Linear (2 point, 1st order)
			short sample0 = m_wholeBuf[iSampleIdxR*2+1], sample1 = m_wholeBuf[(iSampleIdxR+1)*2+1];
			mLowPassR.ApplyCont(&sample, locIntensityR * (sample0 * (1.0f - alpha) + sample1 * alpha));
#elif (INTERP_TYPE == 1)
			short sample1 = m_wholeBuf[iSampleIdxR*2+1], sample2 = m_wholeBuf[(iSampleIdxR+1)*2+1];
			short sample0 = (iSampleIdxR > 0) ? m_wholeBuf[(iSampleIdxR-1)*2+1] : 0,
					sample3 = (iSampleIdxR < (int)(m_totalBufLen - 2)) ? m_wholeBuf[(iSampleIdxR+2)*2+1] : 0;

			// B-Spline (4 point, 3rd order)
			scalar ym1py1 = (scalar)(sample0 + sample2);
			scalar c0 = 1/6.0f*ym1py1 + 2/3.0f*sample1;
			scalar c1 = 1/2.0f*(sample2-sample0);
			scalar c2 = 1/2.0f*ym1py1 - sample1;
			scalar c3 = 1/2.0f*(sample1-sample2) + 1/6.0f*(sample3-sample0);

			m_lowPassR.applyCont(&sample, locIntensityR * ( ((c3*alpha+c2)*alpha+c1)*alpha+c0 ));
#elif (INTERP_TYPE == 2)
			short sample1 = m_wholeBuf[iSampleIdxR*2+1], sample2 = m_wholeBuf[(iSampleIdxR+1)*2+1];
			short sample0 = (iSampleIdxR > 0) ? m_wholeBuf[(iSampleIdxR-1)*2+1] : 0,
					sample3 = (iSampleIdxR < (int)(m_totalBufLen - 2)) ? m_wholeBuf[(iSampleIdxR+2)*2+1] : 0;

			// Hermit (4 point, 3rd order)
			scalar c0 = sample1;
			scalar c1 = 1/2.0f*(sample2-sample0);
			scalar c2 = sample0 - 5/2.0f*sample1 + 2*sample2 - 1/2.0f*sample3;
			scalar c3 = 1/2.0f*(sample3-sample0) + 3/2.0f*(sample1-sample2);

			mLowPassR.ApplyCont(&sample, locIntensityR * ( ((c3*alpha+c2)*alpha+c1)*alpha+c0 ));
#endif

			m_subBufs[m_curSubBuf][i*2+1] = (short)sample;
			m_sampleIdxR += locTimeScale;
		}
		else
		{
			// Advance if we're delayed
			if (delayedSampleIdxR < 0.0)
				m_sampleIdxR += locTimeScale;

			m_subBufs[m_curSubBuf][i*2+1] = 0;
		}
	}

	if (m_recordSamples)
	{
		recordSamples(m_subBufs[m_curSubBuf], m_subBufLen*2);
	}

	m_prevIntensityL = intensityL;
	m_prevIntensityR = intensityR;

	m_prevDelayL = delayL;
	m_prevDelayR = delayR;

	m_prevLowPassFreqL = lowPassFreqL;
	m_prevLowPassFreqR = lowPassFreqR;

	m_prevTimeScale = timeScale;

	return isCurSubBufferValid;
}

bool WaveOutSfx::playSubBuffer()
{
	if (!m_bDevOpen)
	{
		return false;
	}
	if (!m_isSubBufferValid[m_curSubBuf])
	{
		return false;
	}

	LPWAVEHDR pwh = new WAVEHDR;
	if (!pwh)
	{
		return false;
	}

	ZeroMemory(pwh, sizeof(WAVEHDR));
	pwh->dwBufferLength = 2 * m_subBufLen * sizeof(short);
	pwh->lpData = (char *)(m_subBufs[m_curSubBuf]);

	m_mmr = waveOutPrepareHeader(m_hOut, pwh, sizeof(WAVEHDR));
	if (m_mmr)
	{
		return false;
	}

	m_mmr = waveOutWrite(m_hOut, pwh, sizeof(WAVEHDR));
	if (m_mmr)
	{
		return false;
	}

	return true;
}

bool WaveOutSfx::playSoundStereo(short * buf, uint uSize)
{
	if (!m_bDevOpen)
	{
		return false;
	}
		
	if (m_wholeBuf)
	{
		free(m_wholeBuf);
		m_wholeBuf = 0;
	}

	m_wholeBuf = (short *)malloc(2 * uSize * sizeof(short));
	memcpy(m_wholeBuf, buf, 2 * uSize * sizeof(short));

	m_sampleIdxL = 0.0f;
	m_sampleIdxR = 0.0f;

	m_totalBufLen = uSize;	// Number of samples per channel

	m_lowPassL.resetHistory();
	m_lowPassR.resetHistory();

	m_isPrevStateValid = false;

	WaitForSingleObject(m_mutex, INFINITE);

	// Prepare&play first sub buffer
	prepareSubBuffer();
	playSubBuffer();
	bufferAdd();

	// Prepare other sub buffers
	for (uint i = 0; i < m_totalSubBufs - 1; ++i)
	{
		advanceSubBuffer();

		prepareSubBuffer();
		playSubBuffer();

		bufferAdd();
	}

	ReleaseMutex(m_mutex);

	if (m_recordSamples)
	{
		resetSamplesRecording();
	}

	return true;
}

void WaveOutSfx::setTimeScale(scalar ts)
{
	EnterCriticalSection(&m_criticalSection);
	m_timeScale = ts;
	LeaveCriticalSection(&m_criticalSection);
}
scalar WaveOutSfx::getTimeScale()
{
	scalar t_ts;

	EnterCriticalSection(&m_criticalSection);
	t_ts = m_timeScale;
	LeaveCriticalSection(&m_criticalSection);

	return t_ts;
}

void WaveOutSfx::setAzimuth(scalar a)
{
	EnterCriticalSection(&m_criticalSection);
	m_azimuth = a;
	LeaveCriticalSection(&m_criticalSection);
}
scalar WaveOutSfx::getAzimuth()
{
	scalar t_a;

	EnterCriticalSection(&m_criticalSection);
	t_a = m_azimuth;
	LeaveCriticalSection(&m_criticalSection);

	return t_a;
}

void WaveOutSfx::setDistanceFade(scalar df)
{
	EnterCriticalSection(&m_criticalSection);
	m_distanceFade = df;
	LeaveCriticalSection(&m_criticalSection);
}
scalar WaveOutSfx::getDistanceFade()
{
	scalar t_df;

	EnterCriticalSection(&m_criticalSection);
	t_df = m_distanceFade;
	LeaveCriticalSection(&m_criticalSection);

	return t_df;
}

void WaveOutSfx::setIntensity(scalar intensity)
{
	EnterCriticalSection(&m_criticalSection);
	m_intensity = intensity;
	LeaveCriticalSection(&m_criticalSection);
}
scalar WaveOutSfx::getIntensity()
{
	scalar t_intensity;

	EnterCriticalSection(&m_criticalSection);
	t_intensity = m_intensity;
	LeaveCriticalSection(&m_criticalSection);

	return t_intensity;
}

void WaveOutSfx::bufferAdd()
{
	EnterCriticalSection(&m_criticalSection);
	++m_iBufferNum;
	LeaveCriticalSection(&m_criticalSection);
}

void WaveOutSfx::bufferSub()
{
	EnterCriticalSection(&m_criticalSection);
	--m_iBufferNum;
	LeaveCriticalSection(&m_criticalSection);
}

int WaveOutSfx::getBufferNum()
{
	int iTemp;
		
	EnterCriticalSection(&m_criticalSection);
	iTemp = m_iBufferNum;
	LeaveCriticalSection(&m_criticalSection);

	return iTemp;
}

static DWORD WINAPI AudioSfxOutThreadProc(LPVOID lpParameter)
{
	WaveOutSfx *pWaveIn;
	pWaveIn = (WaveOutSfx *)lpParameter;

	MSG msg;
	while (GetMessage(&msg,0,0,0))
	{
		switch( msg.message )
		{
		case WOM_OPEN:
			break;
		case WOM_CLOSE:
			break;
		case WOM_DONE:

			// TODO: reuse PWH pointers too
			WAVEHDR * pwh = (WAVEHDR *)msg.lParam;
			waveOutUnprepareHeader((HWAVEOUT)msg.wParam, pwh, sizeof(WAVEHDR));
			delete pwh;

			// We need this wait to make sure that this thread
			// does not interfere with the parent thread in ::PrepareSubBuffer() function
			WaitForSingleObject(pWaveIn->m_mutex, INFINITE);
			ReleaseMutex(pWaveIn->m_mutex);

			// TODO: add spin loop checking if the buffer is ready
			pWaveIn->advanceSubBuffer();
			bool isActive = pWaveIn->prepareSubBuffer();
			if (isActive)
			{
				pWaveIn->playSubBuffer();
			}

			if (!isActive)
			{
				// Playback ended - finalize sample
				pWaveIn->bufferSub();
			}
			break;
		}
	}

	return (DWORD)msg.wParam;
}

}

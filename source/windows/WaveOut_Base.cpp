#pragma once

#include <Windows.h>
#include <mmsystem.h>
#include "windows/WaveOut_Base.h"

namespace windows
{
WaveOutBase::WaveOutBase():
	Channels(2),
	SamplesPerSec(44100),
	Bitness(16),
	m_isInitiated(false)
{
	InitializeCriticalSection(&m_criticalSection);

	m_hOut = 0;

	m_hAudioOut = 0;
	m_dwAudioOutId = 0;
	m_iBufferNum = 0;

	m_bThreadStart = false;
	m_bDevOpen = false;

	++s_dwInstance;
}

WaveOutBase::~WaveOutBase()
{
	DeleteCriticalSection(&m_criticalSection);
}

bool WaveOutBase::stopThread()
{
	if (!m_bThreadStart)
	{
		return false;
	}

	if (m_hAudioOut)
	{
		int t = 50;
		DWORD ExitCode;
		bool bEnd = false;
		PostThreadMessage(m_dwAudioOutId, WM_QUIT, 0, 0);

		while (t)
		{
			GetExitCodeThread(m_hAudioOut, &ExitCode);
			if (ExitCode != STILL_ACTIVE)
			{
				bEnd = true;
				break;
			}
			else
			{
				Sleep(10);
			}
			--t;
		}

		if (!bEnd)
		{
			TerminateThread(m_hAudioOut,0);
		}

		m_hAudioOut = 0;
	}
	m_bThreadStart = false;

	return true;
}

bool WaveOutBase::openDev()
{
	if (m_bDevOpen)
	{
		return FALSE;
	}

	WAVEFORMATEX wfx;
	wfx.wFormatTag = WAVE_FORMAT_PCM;
	wfx.nChannels = Channels;
	wfx.nSamplesPerSec = SamplesPerSec;
	wfx.nAvgBytesPerSec = (Channels * SamplesPerSec * Bitness) >> 3;
	wfx.nBlockAlign = (Bitness * Channels) >> 3;
	wfx.wBitsPerSample = Bitness;
	wfx.cbSize = 0;

	uint devID = WAVE_MAPPER;
	m_mmr = waveOutOpen(0, devID, &wfx, 0, 0, WAVE_FORMAT_QUERY);
	if (m_mmr)
	{
		return false;
	}

	m_mmr = waveOutOpen(&m_hOut, devID, &wfx, m_dwAudioOutId, s_dwInstance, CALLBACK_THREAD);
	if (m_mmr)
	{
		return false;
	}

	m_bDevOpen = true;
	m_iBufferNum = 0;

	return true;
}

bool WaveOutBase::closeDev()
{
	if (!m_bDevOpen)
	{
		return false;
	}

	if (!m_hOut)
	{
		return false;
	}

	m_mmr = waveOutClose(m_hOut);
	if (m_mmr)
	{
		return false;
	}

	m_hOut = 0;
	m_bDevOpen = false;

	return true;
}

bool WaveOutBase::startPlay()
{
	static uint numOpenedDevices = 0;

	if (!startThread())           
	{
		return false;
	}
	if (!openDev())
	{
		stopThread();
		return false;
	}

	m_isInitiated = true;
	return true;
}

bool WaveOutBase::stopPlay()
{
	closeDev();
	stopThread();

	m_isInitiated = false;
	return true;
}

bool WaveOutBase::reset()
{
	if (!m_hOut)
	{
		return false;
	}

	waveOutReset(m_hOut);

	return true;
}

DWORD WaveOutBase::s_dwInstance = 0;

}

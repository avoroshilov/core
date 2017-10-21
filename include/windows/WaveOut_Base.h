#pragma once

#include <Windows.h>
#include <mmsystem.h>
#include "helpers/common.h"

#define PLAY_DELAY 10

namespace windows
{

class WaveOutBase
{
protected:

	MMRESULT m_mmr;
	HWAVEOUT m_hOut;
	HANDLE m_hAudioOut;
	DWORD m_dwAudioOutId;

	int m_iBufferNum;
	CRITICAL_SECTION m_criticalSection;

	BOOL m_bThreadStart;
	BOOL m_bDevOpen;

	bool m_isInitiated;

public:

	WORD Channels;
	DWORD SamplesPerSec;
	WORD Bitness;

	static DWORD s_dwInstance;
	DWORD WaveOutBase::GetInstance()
	{
		return s_dwInstance;
	}

	friend static DWORD WINAPI AudioOutThreadProc(LPVOID lpParameter);

	WaveOutBase::WaveOutBase();
	WaveOutBase::~WaveOutBase();

	virtual bool WaveOutBase::startThread() = 0;
	bool WaveOutBase::stopThread();

	bool WaveOutBase::openDev();
	bool WaveOutBase::closeDev();

	bool WaveOutBase::startPlay();
	bool WaveOutBase::stopPlay();

	bool WaveOutBase::reset();
};

}

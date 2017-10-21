#pragma once

#include <Windows.h>
#include <mmsystem.h>
#include "helpers/common.h"
#include "windows/WaveOut_Base.h"
#include "windows/WaveOut_Simple.h"

namespace windows
{

bool WaveOutSimple::startThread()
{
	if (m_bThreadStart)
	{
		return false;
	}

	m_hAudioOut = CreateThread(0, 0, AudioOutThreadProc, this, 0, &m_dwAudioOutId);
	if (!m_hAudioOut)
	{
		return false;
	}

	m_bThreadStart = true;
	return true;
}

bool WaveOutSimple::play(char* buf, UINT uSize)
{
	if (!m_bDevOpen)
	{
		return false;
	}
	if (getBufferNum () > PLAY_DELAY)
	{
		return true;
	}
	char * p;
	LPWAVEHDR pwh = new WAVEHDR;
	if (!pwh)
	{
		return false;
	}
		
	p = new char[uSize];
	if (!p)
	{
		return false;
	}

	CopyMemory(p, buf, uSize);
	ZeroMemory(pwh, sizeof(WAVEHDR));
	pwh->dwBufferLength = uSize;
	pwh->lpData = p;

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
	bufferAdd();

	return true;
}

void WaveOutSimple::bufferAdd()
{
	EnterCriticalSection(&m_criticalSection);
	++m_iBufferNum;
	LeaveCriticalSection(&m_criticalSection);
}

void WaveOutSimple::bufferSub()
{
	EnterCriticalSection(&m_criticalSection);
	--m_iBufferNum;
	LeaveCriticalSection(&m_criticalSection);
}

int WaveOutSimple::getBufferNum()
{
	int iTemp;
		
	EnterCriticalSection(&m_criticalSection);
	iTemp = m_iBufferNum;
	LeaveCriticalSection(&m_criticalSection);

	return iTemp;
}

static DWORD WINAPI AudioOutThreadProc(LPVOID lpParameter)
{
	WaveOutSimple *pWaveIn;
	pWaveIn = (WaveOutSimple *)lpParameter;

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
			WAVEHDR * pwh = (WAVEHDR *)msg.lParam;
			waveOutUnprepareHeader((HWAVEOUT)msg.wParam, pwh, sizeof(WAVEHDR));
			delete [] pwh->lpData;
			delete pwh;
			pWaveIn->bufferSub();
			break;
		}
	}

	return (DWORD)msg.wParam;
}

}
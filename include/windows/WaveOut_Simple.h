#pragma once

#include <Windows.h>
#include <mmsystem.h>
#include "helpers/common.h"
#include "windows/WaveOut_Base.h"

#define PLAY_DELAY 10

namespace windows
{

class WaveOutSimple : public WaveOutBase
{
public:

	friend static DWORD WINAPI AudioOutThreadProc(LPVOID lpParameter);

	WaveOutSimple::WaveOutSimple()
	{
	}

	WaveOutSimple::~WaveOutSimple()
	{
	}

	virtual bool WaveOutSimple::startThread();

	bool WaveOutSimple::play(char* buf, UINT uSize);

	void WaveOutSimple::bufferAdd();
	void WaveOutSimple::bufferSub();
	int WaveOutSimple::getBufferNum();
};

}
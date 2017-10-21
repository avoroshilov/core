#include <Windows.h>
#include "windows\timer.h"

#define TIMER_TYPE __int64

namespace windows
{

	TIMER_TYPE Timer::calculateFrequency()
	{
		LARGE_INTEGER tFreq;
		QueryPerformanceFrequency(&tFreq);
		return tFreq.QuadPart;
	}

	void Timer::start(void)
	{
		LARGE_INTEGER s;
		QueryPerformanceCounter(&s);
		m_startTime = s.QuadPart;
	}

	double Timer::time(void)
	{
		m_elapsedTime = ((count() - m_startTime) * 1000 / (double)m_timerFrequency);
		return m_elapsedTime;
	}
	double Timer::timeFromDeltaCount(const TIMER_TYPE & deltaCount)
	{
		m_elapsedTime = (deltaCount * 1000 / (double)m_timerFrequency);
		return m_elapsedTime;
	}

	TIMER_TYPE Timer::deltaCount(void)
	{
		return count() - m_startTime;
	}

	//////////////////////////////////////////////////////////////////////////
	// Private functions
	//////////////////////////////////////////////////////////////////////////
	TIMER_TYPE Timer::count(void)
	{
		LARGE_INTEGER s;
		QueryPerformanceCounter(&s);
		return s.QuadPart;
	}

	TIMER_TYPE Timer::getFrequency()
	{
		static TIMER_TYPE Freq = calculateFrequency();
		return Freq;
	}

}
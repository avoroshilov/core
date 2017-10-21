#pragma once

#define TIMER_TYPE __int64

namespace windows
{

class Timer
{
public:

	Timer()
	{
		m_timerFrequency = getFrequency();
	}

	void start();
	double time();
	double timeFromDeltaCount(const TIMER_TYPE & deltaCount);
	TIMER_TYPE deltaCount();

private:

	TIMER_TYPE count();
	static inline TIMER_TYPE calculateFrequency();
	static TIMER_TYPE getFrequency();

	TIMER_TYPE m_startTime;
	double m_elapsedTime;
	TIMER_TYPE m_timerFrequency;
};

}

#undef TIMER_TYPE
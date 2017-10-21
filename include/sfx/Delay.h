#pragma once

#include "helpers/Common.h"

namespace sfx
{

class Delay
{
protected: 

	uint m_bufferSize, m_bufferPos;
	float * m_buffer;
	uint m_delaySize;

public:

	Delay():
		m_buffer(0),
		m_bufferPos(0),
		m_bufferSize(0),
		m_delaySize(1)
	{
	}

	~Delay()
	{
		if (m_buffer)
			delete [] m_buffer;
	}

	void setParams(uint bufSize, uint delay)
	{
		m_bufferSize = bufSize;
		m_delaySize = delay;

		m_buffer = new float [bufSize];
		for (uint i = 0; i < m_bufferSize; ++i)
		{
			m_buffer[i] = 0.0f;
		}
		m_bufferPos = 0;
	}

	void init(uint delay)
	{
		m_delaySize = delay;
		m_bufferPos = 0;
		for (uint i = 0; i < m_bufferSize; ++i)
		{
			m_buffer[i] = 0.0f;
		}
	}

	void resize(uint newDelay)
	{
		if (newDelay < m_delaySize)
		{
			// Just rollback the write pointer
			m_delaySize = newDelay;
		}
		else if (newDelay > m_delaySize)
		{
#if 1
			m_delaySize = newDelay;
#else

#if 0
			// Adds weird whistling to the KS synth
			int newBufPos = (int)BufferPos - ((int)newDelay - DelaySize);
			newBufPos = newBufPos % BufferSize;
			BufferPos = (uint)newBufPos;
			DelaySize = newDelay;
#else
			// Sounds just like the very simple variant (DelaySize = newDelay)
			uint sizeDelta = newDelay - DelaySize;

			int idxReadShift = (int)DelaySize - (sizeDelta % DelaySize);
			// Try to fill in the gap by repeating last samples
			for (uint i = 0, iend = sizeDelta; i < iend; ++i)
			{
				uint idxWrite = (BufferPos + DelaySize + i) % BufferSize;
				uint idxRead = (BufferPos + idxReadShift) % BufferSize;

				Buffer[idxWrite] = Buffer[idxRead];

				idxReadShift = (idxReadShift + 1) % DelaySize;
			}
			DelaySize = newDelay;
#endif

#endif
		}
	}

	float getSample()
	{
		float sample = m_buffer[m_bufferPos];
		m_buffer[m_bufferPos] = 0.0f;
		m_bufferPos = (m_bufferPos + 1) % m_bufferSize;
		return sample;
	}
	void setSample(float sample)
	{
		uint idx = (m_bufferPos + m_delaySize) % m_bufferSize;
		m_buffer[idx] = sample;
	}
};

class DelayFrac
{
protected:

	uint m_bufferSize, m_bufferPos;
	float * m_buffer;
	uint m_delaySize;
	float m_fraction;

public:

	DelayFrac():
		m_buffer(0),
		m_bufferPos(0),
		m_bufferSize(0),
		m_delaySize(1)
	{
	}

	~DelayFrac()
	{
		if (m_buffer)
			delete [] m_buffer;
	}

	void setParams(uint bufSize, float delay)
	{
		m_bufferSize = bufSize;
		m_delaySize = (int)delay;
		m_fraction = delay - m_delaySize;
		// We need +1 storage for interpolation
		++m_delaySize;

		m_buffer = new float [bufSize];
		for (uint i = 0; i < m_bufferSize; ++i)
		{
			m_buffer[i] = 0.0f;
		}
		m_bufferPos = 0;
	}

	void init(float delay)
	{
		m_delaySize = (int)delay;
		m_fraction = delay - m_delaySize;
		// We need increased storage for this one
		++m_delaySize;

		m_bufferPos = 0;
		for (uint i = 0; i < m_bufferSize; ++i)
		{
			m_buffer[i] = 0.0f;
		}
	}

	void resize(float newDelay)
	{
		if (newDelay < m_delaySize)
		{
			// Just rollback the write pointer
			m_delaySize = (int)newDelay;
			m_fraction = newDelay - m_delaySize;
			++m_delaySize;
		}
		else if (newDelay > m_delaySize)
		{
			int prevDelay = m_delaySize;
			m_delaySize = (int)newDelay;
			m_fraction = newDelay - m_delaySize;
			++m_delaySize;

			// Sounds just like the very simple variant (DelaySize = newDelay)
			uint sizeDelta = m_delaySize - prevDelay;

			int idxReadShift = (int)m_delaySize - (sizeDelta % m_delaySize);
			// Try to fill in the gap by repeating last samples
			for (uint i = 0, iend = sizeDelta; i < iend; ++i)
			{
				uint idxWrite = (m_bufferPos + m_delaySize + i) % m_bufferSize;
				uint idxRead = (m_bufferPos + idxReadShift) % m_bufferSize;

				m_buffer[idxWrite] = m_buffer[idxRead];

				idxReadShift = (idxReadShift + 1) % m_delaySize;
			}
		}
	}

	float getSample()
	{
		float sample = m_buffer[m_bufferPos];
		m_buffer[m_bufferPos] = 0.0f;
		m_bufferPos = (m_bufferPos + 1) % m_bufferSize;
		float sampleNext = m_buffer[m_bufferPos];
		return sample * m_fraction + sampleNext * (1.0f - m_fraction);
	}
	void setSample(float sample)
	{
		uint idx = (m_bufferPos + m_delaySize) % m_bufferSize;
		m_buffer[idx] = sample;
	}
};

}
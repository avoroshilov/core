#pragma once

#include <cstdint>
#include <vector>

class SortRadix
{
protected:

	static const uint32_t c_maxRadicesToSort = 4;
	static const uint32_t c_maxNumberPerRadix = 256;

	uint32_t m_counters[c_maxNumberPerRadix*c_maxRadicesToSort];
	uint32_t * m_offsets[c_maxNumberPerRadix];

	std::vector<uint32_t> m_tempIndices0;
	std::vector<uint32_t> m_tempIndices1;

	size_t m_lastResultSize = 0;
	uint32_t * m_lastResultIndices = nullptr;

public:

	size_t getIndexElementSize() const { return sizeof(uint32_t); }
	size_t getLastResultSize() const { return m_lastResultSize; }
	const uint32_t * getLastResultIndices() const { return m_lastResultIndices; }

	void resizeTempIndices(size_t size)
	{
		m_tempIndices0.resize(size);
		m_tempIndices1.resize(size);
	}

	void sort(const uint32_t * input, size_t size, uint32_t maxVal = 0xFFFFFFFF)
	{
		m_lastResultSize = 0;
		m_lastResultIndices = nullptr;

		if (size == 0)
			return;

		resizeTempIndices(size);

		uint32_t * pRanks0 = m_tempIndices0.data();
		uint32_t * pRanks1 = m_tempIndices1.data();

		uint32_t maxRadix = c_maxRadicesToSort;
		if (maxVal != 0xFFFFFFFF)
		{
			uint mask = 0xFF000000;
			ubyte radix = maxVal & mask;
			while ((radix == 0) && (mask != 0))
			{
				--maxRadix;
				mask >>= 8;
			};
		}

		uint32_t * h0 = m_counters + c_maxNumberPerRadix*0;
		uint32_t * h1 = m_counters + c_maxNumberPerRadix*1;
		uint32_t * h2 = m_counters + c_maxNumberPerRadix*2;
		uint32_t * h3 = m_counters + c_maxNumberPerRadix*3;

		const uint8_t * bytes = reinterpret_cast<const uint8_t *>(input);
		const uint8_t * bytesEnd = bytes + size * c_maxRadicesToSort;
		if (maxRadix > 0)
		{
			// Create histograms (counters) for all 4 bytes at once
			// Ignore maxRadix here as it is easier to just increment
			memset(m_counters, 0, c_maxRadicesToSort * c_maxNumberPerRadix * sizeof(uint));
			while (bytes < bytesEnd)
			{
				h0[*bytes]++; ++bytes;
				h1[*bytes]++; ++bytes;
				h2[*bytes]++; ++bytes;
				h3[*bytes]++; ++bytes;
			}
		}

		// Radix sort [unsigned-only at the moment]
		for (uint32_t radixNum = 0; radixNum < maxRadix; ++radixNum)
		{
			uint32_t * histogram = m_counters + (radixNum << 8);

			// Check pass validity
			bool performPass = true;

			// Get pointer to first radix to sort
			bytes = reinterpret_cast<const uint8_t *>(input);
			bytes += radixNum;

			uint8_t firstByte = *bytes;
			if (histogram[firstByte] == size)
			{
				performPass = false;
			}

			if (performPass)
			{
				m_offsets[0] = pRanks1;
				for (uint32_t i = 1; i < c_maxNumberPerRadix; ++i)
				{
					m_offsets[i] = m_offsets[i - 1] + histogram[i - 1];
				}

				if (radixNum == 0)
				{
					// Ranks are sequential on the first sorting step
					for (uint32_t i = 0; i < size; ++i)
					{
						*m_offsets[bytes[i << 2]]++ = i;
					}
				}
				else
				{
					uint32_t * indices = pRanks0;
					uint32_t * indicesEnd = pRanks0 + size;
					while (indices < indicesEnd)
					{
						uint32_t idx = *indices++;
						*m_offsets[bytes[idx << 2]]++ = idx;
					}
				}

				uint32_t * tmpRanks = pRanks0;
				pRanks0 = pRanks1;
				pRanks1 = tmpRanks;

				m_lastResultIndices = pRanks0;
			}
			else if (radixNum == 0)
			{
				// This is required since otherwise temporary indices are not initialized
				m_lastResultIndices = pRanks0;
				for (uint32_t i = 0; i < size; ++i)
				{
					m_lastResultIndices[i] = i;
				}
			}
		}

		// No sorting required
		if (m_lastResultIndices == nullptr)
		{
			m_lastResultIndices = pRanks0;
			for (uint i = 0; i < size; ++i)
			{
				m_lastResultIndices[i] = i;
			}
		}

		m_lastResultSize = size;

#if 1//(DBG_RADIXSORT_CHECK_SORT_ORDER == 1)
		// DBG check radix sort
		uint32_t prevVal = input[ m_lastResultIndices[0] ];
		for (uint i = 1; i < size; ++i)
		{
			uint32_t curVal = input[ m_lastResultIndices[i] ];
			if (curVal < prevVal)
			{
				assert(false);
			}
			prevVal = curVal;
		}
#endif

	}

	static SortRadix & getInstance()
	{
		static SortRadix s_sortRadix;
		return s_sortRadix;
	}
};


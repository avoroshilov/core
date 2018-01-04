#pragma once

template <class DataType>
class PoolBasic
{
protected:

	std::vector<void *> m_allocatedMemChunks;
	std::vector<DataType *> m_freeSlots;
	size_t m_numPreallocatedElements;

public:

	PoolBasic():
		m_numPreallocatedElements(1000)
	{
		m_freeSlots.reserve(1000);
	}

	~PoolBasic()
	{
		for (size_t i = 0, iEnd = m_allocatedMemChunks.size(); i < iEnd; ++i)
		{
			free(m_allocatedMemChunks[i]);
		}
		m_allocatedMemChunks.resize(0);
	}

	DataType * getElementMemory()
	{
		if (m_freeSlots.size() == 0)
		{
			DataType * data = (DataType *)malloc(m_numPreallocatedElements * sizeof(DataType));
			m_allocatedMemChunks.push_back((void *)data);

			size_t prevSize = m_freeSlots.size();
			size_t newSize = prevSize + m_numPreallocatedElements;
			m_freeSlots.resize(newSize);
			
			DataType ** freeSlots = m_freeSlots.data();
			for (size_t i = 0; i < m_numPreallocatedElements; ++i)
			{
				freeSlots[prevSize+i] = data + i;
			}
		}

		DataType * element = 0;
		
		size_t numSlots = m_freeSlots.size();
		if (numSlots > 0)
		{
			element = m_freeSlots.back();
			m_freeSlots.pop_back();
		}

		return element;
	}

	void putElementMemory(DataType * element)
	{
		m_freeSlots.push_back(element);
	}

	void setPreallocationNum(size_t preallocationNum) { m_numPreallocatedElements = preallocationNum; }	
	size_t getPreallocationNum(void) const { return m_numPreallocatedElements; }

	size_t getFreeSlotsNum() const { return m_freeSlots.size(); }
};
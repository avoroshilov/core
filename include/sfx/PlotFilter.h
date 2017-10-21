#pragma once 

#include <cstring>

#include "helpers/Common.h"
#include "formats/tga.h"
#include "sfx/Filters.h"

namespace sfx
{

void plotFreqResponse(char * filename, const sfx::FilterIIR_2p2z & filter, scalar nyFreq = 22050.0f, uint xTS = 512, uint yTS = 128)
{
	uint imgResolution = xTS * yTS * 4;
	ubyte *tgaFreqData = new ubyte [imgResolution];
	memset(tgaFreqData, 0, imgResolution * sizeof(ubyte));

	uint half_yTS = (yTS >> 1);

	int thousandHz = 0;
	for (uint i = 0; i < xTS; ++i)
	{
		float freq = ((i+1) / (float)xTS) * nyFreq;
		float freqResponse = 0.0f;

		bool drawHz = (freq  > thousandHz * 1000);
		if (drawHz)
		{
			for (uint j = 0; j < yTS; ++j)
			{
				tgaFreqData[((i + j * xTS) << 2) + 0] = 255;
			}
			++thousandHz;
		}

		tgaFreqData[((i + half_yTS * xTS) << 2) + 0] = 255;

		freqResponse = filter.responseM(freq / (scalar)(nyFreq * 2.0f));

		uint yCoord = int(freqResponse * half_yTS);
		if (yCoord >= yTS)
			yCoord = yTS - 1;
		tgaFreqData[((i + yCoord * xTS) << 2) + 0] = 255;
		tgaFreqData[((i + yCoord * xTS) << 2) + 1] = 255;
		tgaFreqData[((i + yCoord * xTS) << 2) + 2] = 255;
	}
	formats::writeTGA(filename, tgaFreqData, xTS, yTS, 32);
	delete [] tgaFreqData;
}

void plotWaveformChannel(char * filename, scalar * waveform, uint numSamples, uint xTS = 512, uint yTS = 128)
{
	uint imgResolution = xTS * yTS * 4;
	ubyte *tgaFreqData = new ubyte [imgResolution];
	memset(tgaFreqData, 0, imgResolution * sizeof(ubyte));

	uint half_yTS = (yTS >> 1);

	int thousandHz = 0;
	uint prevCoord = 0;
	for (uint i = 0; i < xTS; ++i)
	{
		uint pos = (uint)( i / (scalar)xTS * numSamples );
		float sample = 0.0f;

		//bool drawHz = (freq  > thousandHz * 1000);
		//if (drawHz)
		//{
		//	for (uint j = 0; j < yTS; ++j)
		//	{
		//		tgaFreqData[((i + j * xTS) << 2) + 0] = 255;
		//	}
		//	++thousandHz;
		//}

		tgaFreqData[((i + half_yTS * xTS) << 2) + 0] = 255;

		sample = waveform[pos*2+0];

		uint yCoord = int(sample * half_yTS + half_yTS);
		yCoord = clamp<uint>(yCoord, 0, yTS - 1);

		tgaFreqData[((i + yCoord * xTS) << 2) + 0] = 255;
		tgaFreqData[((i + yCoord * xTS) << 2) + 1] = 255;
		tgaFreqData[((i + yCoord * xTS) << 2) + 2] = 255;

		if (i > 0 && i < xTS - 1)
		{
			if (prevCoord > yCoord + 1)
			{
				for (uint cnt = 0, cntEnd = prevCoord - yCoord; cnt < cntEnd; ++cnt)
				{
					scalar delta = cnt / (scalar)cntEnd;
					uint i1 = i-1 + int(delta + 0.5f);

					tgaFreqData[((i1 + (prevCoord - cnt) * xTS) << 2) + 0] = 255;
					tgaFreqData[((i1 + (prevCoord - cnt) * xTS) << 2) + 1] = 255;
					tgaFreqData[((i1 + (prevCoord - cnt) * xTS) << 2) + 2] = 255;
				}
			}
			else if (yCoord > prevCoord + 1)
			{
				for (uint cnt = 0, cntEnd = yCoord - prevCoord; cnt < cntEnd; ++cnt)
				{
					scalar delta = cnt / (scalar)cntEnd;
					uint i1 = i-1 + int(delta + 0.5f);

					tgaFreqData[((i1 + (prevCoord + cnt) * xTS) << 2) + 0] = 255;
					tgaFreqData[((i1 + (prevCoord + cnt) * xTS) << 2) + 1] = 255;
					tgaFreqData[((i1 + (prevCoord + cnt) * xTS) << 2) + 2] = 255;
				}
			}
		}

		prevCoord = yCoord;
	}
	formats::writeTGA(filename, tgaFreqData, xTS, yTS, 32);
	delete [] tgaFreqData;
}

}
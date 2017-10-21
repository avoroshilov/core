#pragma once

#include <stdio.h>

#include "helpers/Common.h"
#include "formats/wav.h"

namespace formats
{

template<typename dataType>
void writeData(FILE * fp, dataType data, size_t size = sizeof(dataType))
{
	for (; size; --size, data >>= 8)
		fputc(static_cast<char>(data & 0xFF), fp);
}

void writeWAV(const char * filename, const short * waveData, uint samplesPerChannel, uint samplesPerSec, uint numChannels)
{
	FILE * fp;
	fopen_s(&fp, filename, "wb");
	if (!fp)
		return;

	// ChunkID
	fprintf(fp, "RIFF");

	// placeholder int to debug WAV output
	const int placeholder = 0xf00dbeef;

	// ChunkSize: placeholder where the RIFF chunk size will be stored 
	size_t riffSizeOffset = ftell(fp);
	writeData(fp, placeholder, 4);

	size_t riffOffset = ftell(fp);
	fprintf(fp, "WAVE");
	// Subchunk1ID: format description
	fprintf(fp, "fmt ");
	// Subchunk1Size: 16 for PCM
	writeData(fp, 16, 4);

	// format description
	writeData(fp, 1,								2);  // AudioFormat: 1 - no compression
	writeData(fp, (short)numChannels,				2);  // NumChannels
	writeData(fp, samplesPerSec,					4);  // SampleRate: Hz
	writeData(fp, samplesPerChannel * numChannels,	4);  // ByteRate: == SampleRate * NumChannels * BitsPerSample/8
	writeData(fp, sizeof(short)*numChannels,		2);  // BlockAlign: == NumChannels * BitsPerSample/8 (The number of bytes for one sample including all channels)
	writeData(fp, sizeof(short)*8,					2);  // BitsPerSample

	// Subchunk2ID: data section starts
	fprintf(fp, "data");

	// Subchunk2Size: placeholder where the data size will be stored
	// == NumSamples * NumChannels * BitsPerSample/8
	// but it will be corrected after the actual file write (see below)
	size_t dataSizeOffset = ftell(fp);
	writeData(fp, placeholder, 4);

	size_t dataOffset = ftell(fp);
	for (uint sampleIdx = 0; sampleIdx < samplesPerChannel; ++sampleIdx)
	{
		for (uint channelIdx = 0; channelIdx < numChannels; ++channelIdx)
		{
			writeData(fp, waveData[sampleIdx*numChannels+channelIdx], 2);
		}
	}

	size_t dataChunkEnd = ftell(fp);

	// fill the Subchunk2Size placeholder (data chunk size)
	int dataSize = (int)(dataChunkEnd - dataOffset);
	fseek(fp, (int)dataSizeOffset, SEEK_SET);
	writeData(fp, dataSize, 4);

	// fill the ChunkSize placeholder (RIFF chunk size)
	fseek(fp, (int)riffSizeOffset, SEEK_SET);
	writeData(fp, dataChunkEnd - riffOffset, 4);

	fclose(fp);
}

}
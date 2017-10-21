#pragma once

#include "helpers/Common.h"

namespace formats
{

// filename - self-explanatory
// waveData - actual waveform to be written to a file
// samplesPerChannel - total amount of samples per channel, typically durationSeconds*samplesPerSec
// samplesPerSec & numChannels - self-explanatory
void writeWAV(const char * filename, const short * waveData, uint samplesPerChannel, uint samplesPerSec = 44100, uint numChannels = 2);

}
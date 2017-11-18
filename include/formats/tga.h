#pragma once

namespace formats
{

enum class ImagePixelFormat
{
	eRGBA,
	eABGR,
	eBGRA,
	eNUM_ENTRIES
};

void writeTGA (
	const char			*filename,
	unsigned char		*imageData,
	short int			width, 
	short int			height, 
	unsigned char		bitsPerPixel,
	ImagePixelFormat	pixelFormat = ImagePixelFormat::eRGBA
	);

}
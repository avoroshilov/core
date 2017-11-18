#pragma once

#include <stdio.h>

#include "formats/tga.h"

namespace formats
{

void writeTGA(
	const char			*filename,
	unsigned char		*imageData,
	short int			width, 
	short int			height, 
	unsigned char		bitsPerPixel,
	ImagePixelFormat	pixelFormat
	)
{
	FILE * fp;
	fopen_s(&fp, filename, "wb");
	if (!fp)
		return;

	const unsigned char zeroChar = 0;
	const short int zeroShort = 0;

	// Length is the length of a string located located after the header
	fwrite(&zeroChar, sizeof(unsigned char), 1, fp);
	// Colour map type - whether a color map is included
	fwrite(&zeroChar, sizeof(unsigned char), 1, fp);

	// Image type: 0 - no image data, 1 - uncompressed color-mapped image, 2 - uncompressed true-color image, 3 - uncompressed black-and-white (grayscale) image
	const int typeUncompressedGrayscale = 3;
	const int typeUncompressedTrueColor = 2;
	int type = typeUncompressedGrayscale;
	if ((bitsPerPixel == 24) || (bitsPerPixel == 32))
		type = typeUncompressedTrueColor;
	fwrite(&type, sizeof(unsigned char), 1, fp);

	// Color map specification
	//	origin
	fwrite(&zeroShort, sizeof(short int), 1, fp);
	//	length
	fwrite(&zeroShort, sizeof(short int), 1, fp);
	//	depth
	fwrite(&zeroChar, sizeof(unsigned char), 1, fp);

	// Image spec
	//	x,y origin
	fwrite(&zeroShort, sizeof(short int), 1, fp);
	fwrite(&zeroShort, sizeof(short int), 1, fp);

	//	w,h and bits per pixel
	fwrite(&width, sizeof(short int), 1, fp);
	fwrite(&height, sizeof(short int), 1, fp);
	fwrite(&bitsPerPixel, sizeof(unsigned char), 1, fp);

	// Image descriptor (alpha+direction)
	fwrite(&zeroChar, sizeof(unsigned char), 1, fp);

	int bytesPerPixel = bitsPerPixel / 8;
	if (bytesPerPixel >= 3)
	{
		if (pixelFormat == ImagePixelFormat::eRGBA)
		{
			unsigned char temp_swap;
			for (int i = 0, iend = width * height * bytesPerPixel; i < iend; i += bytesPerPixel)
			{
				temp_swap = imageData[i];
				imageData[i] = imageData[i+2];
				imageData[i+2] = temp_swap;
			}
		}
		else if (pixelFormat == ImagePixelFormat::eABGR)
		{
			unsigned char temp_swap;
			for (int i = 0, iend = width * height * bytesPerPixel; i < iend; i += bytesPerPixel)
			{
				temp_swap = imageData[i];
				imageData[i] = imageData[i+1];
				imageData[i+1] = imageData[i+2];
				imageData[i+2] = imageData[i+3];
				imageData[i+3] = temp_swap;
			}
		}
		// Nothing to do in eBGRA case
	}

	fwrite(imageData, sizeof(unsigned char), width * height * bytesPerPixel, fp);

	fclose(fp);
}

}
#pragma once

namespace formats
{

void writeTGA (
	const char		*filename,
	unsigned char	*imageData,
	short int		width, 
	short int		height, 
	unsigned char	bitsPerPixel
	);

}
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cstring>

#include "formats/tga.h"
#include "textures/GenTextures.h"

namespace textures
{

void TexGen::SaveToTGA(unsigned char layer, unsigned int num)
{
	unsigned char* temp = mTempLayer;

	int bVal = (layer * xTS * yTS)<<2;

	int x;
	for (x = 0; x < (xTS*yTS)<<2; x++)
	{
		temp[x] = Layer[bVal+x];
	}

	const int filenameSize = 256;
	char filename[filenameSize];
	sprintf_s(filename, filenameSize, "xxTex%04d.tga", num);
	formats::writeTGA(filename, temp, xTS, yTS, 32);
}

void TexGen::SaveToTGA(unsigned char layer, const char * name)
{
	unsigned char* temp = mTempLayer;

	int bVal = (layer * xTS * yTS) << 2;

	int x;
	for (x = 0; x < (xTS*yTS) << 2; x++)
	{
		temp[x] = Layer[bVal + x];
	}

	formats::writeTGA(name, temp, xTS, yTS, 32);
}

void TexGen::SaveToTGAS(int cL, const char *fmt, ...)
{
	char text[1024];
	va_list ap;

	if (fmt == NULL)
		return;

	va_start(ap, fmt);
		vsprintf_s(text, fmt, ap);
	va_end(ap);

	unsigned char* temp = mTempLayer;

	int bVal = cL * xTS * yTS << 2;

	int x;
	for (x = 0; x < xTS*yTS<<2; x++)
	{
		temp[x] = Layer[bVal+x];
	}

	const int filenameSize = 512;
	char filename[filenameSize];
	sprintf_s(filename, filenameSize, "%s.tga", text);
	formats::writeTGA(filename, temp, xTS, yTS, 32);
}

void TexGen::MakeAlpha(unsigned char cLayers)
{
	int bVal = (cLayers >>  4) * xTS * yTS << 2;
	int sVal = (cLayers & 0xF) * xTS * yTS << 2;
	int x, y, yAdd = 0;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
			Layer[bVal+((x+yAdd)<<2)+3] = clamp<int>(int(Layer[sVal+((x+yAdd)<<2)]*0.229f + Layer[sVal+((x+yAdd)<<2)+1]*0.587f + Layer[sVal+((x+yAdd)<<2)+2]*0.134f), 0, 255);

		yAdd += xTS;
	}
}

#if (USE_FONT == 1)
void TexGen::InitFont(char *fontname, int size, int flags)
{
	mdc = CreateCompatibleDC(GetDC(NULL));
	bm = CreateCompatibleBitmap(mdc, xTS, yTS);

	bool Bold = (flags>>2)?true:false;
	bool Italic = ((flags - (Bold<<2))>>1)?true:false;
	bool ULine = (flags - (Bold<<2) - (Italic<<1))?true:false;

	hf = CreateFontA(size, 0, 0, 0, Bold?FW_HEAVY:FW_NORMAL, Italic, ULine, false, ANSI_CHARSET,
				    OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
					DEFAULT_PITCH, fontname);

	r.top = 0; r.left = 0;
	r.right = xTS; r.bottom = yTS;

	bmi.bmiHeader.biSize = sizeof(bmi.bmiHeader);
	bmi.bmiHeader.biWidth = xTS;
	bmi.bmiHeader.biHeight = yTS;
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 32;
	bmi.bmiHeader.biCompression = BI_RGB;

	SelectObject(mdc, hf);
	SetBkMode(mdc, TRANSPARENT);
	SetTextColor(mdc, 0xFFFFFF);

	SelectObject(mdc, bm);
}
void TexGen::InitFont(char *fontname, int size, int flags, int *Widths)
{
	HDC hdc = GetDC(NULL);
	mdc = CreateCompatibleDC(hdc);
	bm = CreateCompatibleBitmap(hdc, xTS, yTS);

	bool Bold = (flags>>2)?true:false;
	bool Italic = ((flags - (Bold<<2))>>1)?true:false;
	bool ULine = (flags - (Bold<<2) - (Italic<<1))?true:false;

	hf = CreateFontA(size, 0, 0, 0, Bold?FW_HEAVY:FW_NORMAL, Italic, ULine, false, ANSI_CHARSET,
				    OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
					DEFAULT_PITCH, fontname);

	r.top = 0; r.left = 0;
	r.right = xTS; r.bottom = yTS;

	bmi.bmiHeader.biSize = sizeof(bmi.bmiHeader);
	bmi.bmiHeader.biWidth = xTS;
	bmi.bmiHeader.biHeight = yTS;
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 32;
	bmi.bmiHeader.biCompression = BI_RGB;

	SelectObject(mdc, hf);
	SetBkMode(mdc, TRANSPARENT);
	SetTextColor(mdc, 0xFFFFFF);

	SelectObject(mdc, bm);

	GetCharWidth(mdc, 0, 255, Widths);

	// Fix character widths here
	Widths['/'] = (int)(Widths['/'] * 1.5f);
	Widths[')'] = (int)(Widths[')'] * 1.2f);
	Widths['i'] = (int)(Widths['i'] * 1.25f);
	Widths['j'] = (int)(Widths['j'] * 1.25f);
	Widths['l'] = (int)(Widths['l'] * 1.25f);
	Widths['.'] = (int)(Widths['.'] * 1.25f);
	Widths['I'] = (int)(Widths['I'] * 1.25f);

	FillRect(mdc, &r, (HBRUSH)GetStockObject(BLACK_BRUSH));
}

void TexGen::RenderFont(int gL, int x, int y, char *str)
{
	unsigned char *temp = mTempLayer;
	int bVal = gL * xTS * yTS << 2, k;

	FillRect(mdc, &r, (HBRUSH)GetStockObject(BLACK_BRUSH));
	TextOutA(mdc, x, y, str, strlen(str));

	GetDIBits(mdc, bm, 0, yTS, temp, &bmi, DIB_RGB_COLORS);

	for (k = 0; k < (xTS*yTS<<2); k++)
		if (temp[k] > 0)
			Layer[bVal+k] = clamp<int>(Layer[bVal+k]+((temp[k]*(255-Layer[bVal+k]))>>8), 0, 255);
}

void TexGen::FormFont(int x, int y, char *str)
{
	TextOutA(mdc, x, y, str, strlen(str));
}
void TexGen::GetFontDIB(int gL)
{
	GetDIBits(mdc, bm, 0, yTS, Layer+(gL*xTS*yTS<<2), &bmi, DIB_RGB_COLORS);
}

void TexGen::DeleteFont(void)
{
	DeleteObject(hf);
	DeleteObject(bm);
}
#endif

void TexGen::Copy(int gL, unsigned char *src)
{
	int i, bVal = gL * xTS * yTS << 2;

	unsigned char *lPos = Layer + bVal, *sPos = src;
	for (i = 0; i < xTS*yTS<<2; i++)
		*lPos++ = *sPos++;
}

void TexGen::CopySub(int gL, int origX, int origY, unsigned char * src, int sizeX, int sizeY)
{
	int bVal = gL * xTS * yTS << 2, yAdd, yAddSrc;
	int x, y;

	unsigned char *lPos = Layer + bVal, *sPos = src;
	
	yAdd = origY * xTS;
	yAddSrc = 0;
	for (y = 0; y < sizeY; y++)
	{
		for (x = 0; x < sizeX; x++)
		{
			int origShift = ((x+origX)+yAdd)<<2;
			int srcShift = (x+yAddSrc)<<2;
			Layer[bVal+origShift  ] = *(src+srcShift  );
			Layer[bVal+origShift+1] = *(src+srcShift+1);
			Layer[bVal+origShift+2] = *(src+srcShift+2);
			Layer[bVal+origShift+3] = *(src+srcShift+3);
		}
		yAdd += xTS;
		yAddSrc += sizeX;
	}
}

/* Generating */
void TexGen::DrawLine(int gL, int x1, int y1, int x2, int y2)
{
	int bVal = gL * xTS * yTS * 4;
	float k, b, y;

	k = float(y2 - y1) / float(x2 - x1);
	b = y1 - k * x1;

	if (x2 < x1)
	{
		int t = x1;
		x1 = x2;
		x2 = t;
	}

	for (int x = x1; x < x2; x++)
	{
		y = k*x + b;
		Layer[bVal+(x+(int)y*xTS)*4+0] = 255;
		Layer[bVal+(x+(int)y*xTS)*4+1] = 255;
		Layer[bVal+(x+(int)y*xTS)*4+2] = 255;
	}
}

void TexGen::GradVert(int gL, int y1, int y2, unsigned char bx, unsigned char by, unsigned char bz, unsigned char ex, unsigned char ey, unsigned char ez)
{
	int x, y, yAdd = y1 * xTS;
	int bVal = gL * xTS * yTS << 2;

	for (y = y1; y < y2; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			Layer[bVal+((x+yAdd)<<2)  ] = (int)((ex-bx)*((y-y1)/float(y2-y1-1))+bx);
			Layer[bVal+((x+yAdd)<<2)+1] = (int)((ey-by)*((y-y1)/float(y2-y1-1))+by);
			Layer[bVal+((x+yAdd)<<2)+2] = (int)((ez-bz)*((y-y1)/float(y2-y1-1))+bz);
		}
		yAdd += xTS;
	}
}

void TexGen::Rect(unsigned char lParams, int xCoord, int yCoord, int xSize, int ySize, int Color, int Angle)
{
	int x, y;
	int bVal = (lParams >> 4) * xTS * yTS << 2;

	int ch = lParams & 15;

	int xPos, yPos;

	int mA = int(2048*cosf(Deg2Rad(float((Angle*360)>>8))));
	int mB = int(2048*sinf(Deg2Rad(float((Angle*360)>>8))));

	int xi, yi, yu;
	int xp, yp;

	for (y = 0; y < ySize; y++)
	{
		xp = xCoord + xTS - (((y-(ySize>>1))*mB)>>11);
		yp = yCoord + yTS + (((y-(ySize>>1))*mA)>>11);

		for (x = 0; x < xSize; x++)
		{
			xPos = xp + (((x-(xSize>>1))*mA)>>11);
			yPos = yp + (((x-(xSize>>1))*mB)>>11);

			xi = int(xPos); yi = int(yPos);
			yu = (yi+1)&(yTS-1);
			xi &= xTS - 1; yi &= yTS - 1;

			int pos[4] =
			{
				(xi+yi*xTS)<<2,
				(xi+yu*xTS)<<2,
			};

			Layer[bVal+pos[0]+ch] = Color;
			Layer[bVal+pos[1]+ch] = Color;
		}
	}
}

void TexGen::Bricks(unsigned char pLayer, unsigned char cRow, unsigned char cCol, unsigned char vSize, unsigned char vAng, unsigned char vCol, unsigned char Shift, unsigned int Spacing)
{
	if (cCol == 0 || cRow == 0) return;

	int sX = (Spacing>>16), sY = Spacing&0xFFFF;

	int SizeX = (xTS << 8) / cCol;
	int SizeY = (yTS << 8) / cRow;

	for (int j = 0; j < cRow; j++)
		for (int i = 0; i < cCol; i++)
			Rect(pLayer, j*Shift+(i*SizeX>>8)+(SizeX>>9), (j*SizeY>>8)+(SizeY>>9), (SizeX>>8)-sX+((vSize*((rand()%1024)-512))>>10), (SizeY>>8)-sY+((vSize*((rand()%1024)-512))>>10), 255-((vCol*(rand()%256))>>8), vAng*((rand()%256)-128)>>8);
}

void TexGen::Check(unsigned char lParams)
{
	unsigned char *bVal = Layer + ((lParams >> 4) * xTS * yTS << 2);
	unsigned char *pL = bVal;
	int x, y, c, SHIFT = lParams & 0x0F;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			c = ((int)(pL - bVal) >> SHIFT)^((y << 2) >> SHIFT);
			*pL = c;
			pL += 4;
		}
	}
}

void TexGen::SineWave(int gL, int k)
{
	int x, y;
	int bVal = gL * xTS * yTS * 4;

	float Z = (k * 2 * PI) / (m_max(xTS, yTS) / 2.0f);

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			float D = sqrtf(sqr(x-xTS/2.0f)+sqr(y-yTS/2.0f));
			float c = 127 * sinf(D * Z) + 128;
			Layer[bVal+((x+(y*xTS))*4)+0] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+1] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+2] = (int)c;
		}
	}
}

void TexGen::SineVLin(int gL, int K)
{
	int x, y;
	int bVal = gL * xTS * yTS * 4;

	float Z = (K * 2 * PI) / (float)xTS;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			float c = 127 * sinf(x * Z) + 128;
			Layer[bVal+((x+(y*xTS))*4)+0] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+1] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+2] = (int)c;
		}
	}
}

void TexGen::SineHLin(int gL, int K)
{
	int x, y;
	int bVal = gL * xTS * yTS * 4;

	float Z = (K * 2 * PI) / (float)yTS;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			float c = 127 * sinf(y * Z) + 128;
			Layer[bVal+((x+(y*xTS))*4)+0] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+1] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+2] = (int)c;
		}
	}
}

void TexGen::SineHCLin(int gL, int K)
{
	int x, y;
	int bVal = gL * xTS * yTS * 4;

	float Z = (K * 2 * PI) / (float)yTS;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			float c = 128 * sinf(y * Z);
			Layer[bVal+((x+(y*xTS))*4)+0] = (int)fabsf(c);
			Layer[bVal+((x+(y*xTS))*4)+1] = (int)fabsf(c);
			Layer[bVal+((x+(y*xTS))*4)+2] = (int)fabsf(c);
		}
	}
}

void TexGen::SineLDg(int gL, int K)
{
	int x, y;
	int bVal = gL * xTS * yTS * 4;

	float xZ = (K * 2 * PI) / (float)xTS;
	float yZ = (K * 2 * PI) / (float)yTS;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			float c = 127 * sinf(y*yZ + x*xZ) + 128;
			Layer[bVal+((x+(y*xTS))*4)+0] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+1] = (int)c;
			Layer[bVal+((x+(y*xTS))*4)+2] = (int)c;
		}
	}
}

void TexGen::SineRDg(unsigned char gL, int K, unsigned char tWraps)
{
	int x, y, yAdd = 0, c;
	int bVal = gL * xTS * yTS << 2;

	float xZ = (K * 2 * PI) / float(1 << (tWraps >>  4));
	float yZ = (K * 2 * PI) / float(1 << (tWraps & 0xF));
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			c = int(128 * sinf(y*yZ - x*xZ)) + 127;
			Layer[bVal+((x+yAdd)<<2)  ] = c;
			Layer[bVal+((x+yAdd)<<2)+1] = c;
			Layer[bVal+((x+yAdd)<<2)+2] = c;
		}
		yAdd += xTS;
	}
}

void TexGen::DrawMLine(int gL, int x1, int y1, int x2, int y2)
{
	DrawLine(gL, x1, y1, x2, y2);
}

void TexGen::Cells(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, unsigned char *Array)
{
	int xCoords[256];
	int yCoords[256];

	unsigned char calcMode = Mode >> 4;
	unsigned char drawMode = Mode & 15;

	for (int i = 0; i < numPoints; i++)
	{
		int xCoord = xCoords[i] = rand()&(xTS-1);
		int yCoord = yCoords[i] = rand()&(yTS-1);

		if ((calcMode > 1) || (drawMode > 1))
		{
			for (int j = 0; j < i; j++)
				if (sqr(xCoord-xCoords[j])+sqr(yCoord-yCoords[j]) < 100)
				{
					numPoints--; i--;
					break;
				}
		}
	}

	CellsAdv(gL, Mode, numPoints, Power, Variance, xCoords, yCoords, Array);
}

void TexGen::CellsEven(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, unsigned char *Array)
{
	int xCoords[512];
	int yCoords[512];

	const int numPointsGenerated = numPoints * 2;

	for (int i = 0; i < numPoints; i++)
	{
		xCoords[i] = rand()&(xTS-1);
		yCoords[i] = rand()&(yTS-1);
	}

	// We don't want to end up with less points that we need, so we need to damp requirement a bit
	const float minDistMul = 0.6f;
	int minDistSq = (int)( (float)((xTS+yTS)>>1) / sqrtf((float)numPoints) * minDistMul );
	minDistSq *= minDistSq;

	int numPointsLeft = numPointsGenerated;
	for (int i = 0; i < numPointsLeft; ++i)
	{
		int xi = xCoords[i];
		int yi = yCoords[i];
		for (int j = 0; j < i; ++j)
		{
			int dx = xCoords[j] - xi;
			int dy = yCoords[j] - yi;

			WrapDist(dx, dy);

			int distSq = dx*dx + dy*dy;
			if (distSq < minDistSq)
			{
				--numPointsLeft;
				xCoords[i] = rand()&(xTS-1);
				yCoords[i] = rand()&(yTS-1);
				--i;
				
				break;
			}
		}
	}

	CellsAdv(gL, Mode, (numPointsLeft > numPoints) ? numPoints : numPointsLeft, Power, Variance, xCoords, yCoords, Array);
}

void TexGen::CellsSplits(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, unsigned char splits, unsigned char xSplitPow, unsigned char ySplitPow, unsigned char *Array)
{
	int xCoords[256];
	int yCoords[256];

	for (int i = 0; i < numPoints; i++)
	{
		int xCoord = xCoords[i] = rand()&(xTS-1);
		int yCoord = yCoords[i] = rand()&(yTS-1);
	}

	int xSplits = splits >> 4;
	int ySplits = splits & 0xF;

	int xSplitStep;
	if (xSplits != 0)
		xSplitStep = xTS / xSplits;
	int ySplitStep;
	if (ySplits != 0)
		ySplitStep = yTS / ySplits;

	if (xSplits != 0 || ySplits != 0)
	{
		for (int i = 0; i < numPoints; i++)
		{
			int xCoord = xCoords[i];
			int yCoord = yCoords[i];

			if (xSplits != 0)
			{
				int xSplitCoord = (xCoord + (xSplitStep >> 1)) / xSplitStep * xSplitStep;
				int xDelta = ((xSplitCoord - xCoord) * xSplitPow) >> 8;
				xCoords[i] = (xCoord + xDelta) & (xTS - 1);
			}

			if (ySplits != 0)
			{
				int ySplitCoord = (yCoord + (ySplitStep >> 1)) / ySplitStep * ySplitStep;
				int yDelta = ((ySplitCoord - yCoord) * ySplitPow) >> 8;
				yCoords[i] = (yCoord + yDelta) & (yTS - 1);
			}
		}
	}

	CellsAdv(gL, Mode, numPoints, Power, Variance, xCoords, yCoords, Array);
}
void TexGen::CellsAdv(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, int * xCoords, int * yCoords, unsigned char *Array)
{
	//////////////////////////////////////////////////////////////////////////
	// TODO:
	//   Point gen function - form point set by color gradient. E.g. you have
	// an image (blurred), you can sample the image below the point, determine
	// the gradient and move the point in direction of this gradient. One can
	// create interesting text effects with this approach.
	//   Code cleanup (now distance is calculated 6 times in the code: 3x
	// "fair" and 3x "pregen" - it could be moved to macro for example).
	//////////////////////////////////////////////////////////////////////////

	bool precalcDist = (Array != 0);

	const int cTexSize = xTS*yTS; 

	unsigned char calcMode = Mode >> 4;
	unsigned char drawMode = Mode & 15;

	bool additionalPass = (drawMode == 3) || (drawMode == 4);
	bool skipFirstPass = (drawMode == 3);

	bool isDistance2Needed = (calcMode == 1) || (calcMode == 2) || (calcMode == 3);

#define USE_RAW_CHUNK 1

#if (USE_RAW_CHUNK == 1)
	unsigned char * & rawChunk = GetRawChunk();
	uint & rawChunkSize = GetRawChunkSize();
	unsigned char * pAllocBuf = rawChunk;

#define ALIGN_POINTER(ptr, alignment) (ptr = (unsigned char *)( ((size_t)ptr + (alignment - 1)) & (~(alignment - 1)) ))

	unsigned char * distBuffer = ALIGN_POINTER(pAllocBuf, 8);	pAllocBuf += cTexSize * sizeof(unsigned char);
	unsigned char * indBuffer1 = ALIGN_POINTER(pAllocBuf, 8);	pAllocBuf += cTexSize * sizeof(unsigned char);
	unsigned char * indBuffer2 = ALIGN_POINTER(pAllocBuf, 8);	pAllocBuf += cTexSize * sizeof(unsigned char);
	unsigned char indBuf2_Stub;

#undef ALIGN_POINTER

#else
	unsigned char * distBuffer = (unsigned char *)malloc(cTexSize * sizeof(unsigned char));
	unsigned char * indBuffer1 = (unsigned char *)malloc(cTexSize * sizeof(unsigned char));
	unsigned char * indBuffer2 = (unsigned char *)malloc(cTexSize * sizeof(unsigned char));
#endif

	char shX = 8 - get2Pow(xTS), shY = 8 - get2Pow(yTS);

	int bVal = ((gL>>4) * xTS * yTS) << 2, ch = gL & 0xF;
	int i, x, y, yAdd = 0;
	
	unsigned int DtNP1, DtNP2, Dist, closestIdx;
	unsigned char * dBuffPos = distBuffer;

#if 0
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			DtNP1 = DtNP2 = 0xFFffFFff;
			if (precalcDist)
			{
				for (i = 0; i < numPoints; i++)
				{
					Dist = Array[(((x+*(xCoords+i))<<shX)&255)+((((y+*(yCoords+i))<<shY)&255)<<8)];

					if (Dist <= DtNP1)
					{
						DtNP2 = DtNP1;
						DtNP1 = Dist;
						indBuffer1[x+yAdd] = closestIdx = i;
					} else if (Dist <= DtNP2) DtNP2 = Dist;
				}
			}
			else
			{
				for (i = 0; i < numPoints; i++)
				{
					// WrapDist
					//////////////////////////////////////////////////////////////////////////
					// Probably implement a lookup table here [2 * min(xTS,yTS) or just very huge like 1024 to map negatives too]
					int dx = x - *(xCoords+i);
					int dy = y - *(yCoords+i);

					WrapDist(dx, dy);

					Dist = dx*dx + dy*dy;
					//////////////////////////////////////////////////////////////////////////

					if (Dist <= DtNP1)
					{
						DtNP2 = DtNP1;
						DtNP1 = Dist;
						indBuffer1[x+yAdd] = closestIdx = i;
					} else if (Dist <= DtNP2) DtNP2 = Dist;
				}

				DtNP1 = (unsigned int)sqrtf((float)DtNP1);
				DtNP2 = (unsigned int)sqrtf((float)DtNP2);
			}

			unsigned char dist;
			if		(calcMode == 0) dist = clamp<int>(DtNP1, 0, 255);
			else if (calcMode == 1) dist = clamp<int>((DtNP2 * DtNP1) >> 5, 0, 255);
			else if (calcMode == 2) dist = clamp<int>(DtNP2 - DtNP1, 0, 255);
			else if (calcMode == 3) dist = clamp<int>((DtNP1 << 5) / DtNP2, 0, 255);

			if (!skipFirstPass)
			{
				int addColor = (Variance * (((indBuffer1[x+yAdd]<<2)+342561)&255)) >> 8;
				if		(drawMode == 0 || drawMode == 4)
					Layer[bVal+((x+yAdd)<<2)+ch] = clamp<int>(((dist * Power) >> 5) - addColor, 0, 255);
				else if (drawMode == 1)
					Layer[bVal+((x+yAdd)<<2)+ch] = 255 - clamp<int>(((dist * Power) >> 5) - addColor, 0, 255);
				else if (drawMode == 2)
					Layer[bVal+((x+yAdd)<<2)+ch] = (dist >= Power)?(255-addColor):0;
			}
		}
		yAdd += xTS;
	}
#else

	//////////////////////////////////////////////////////////////////////////
	//   This speed up works as follows: we don't need to calculate distance
	// for each pixel of the image. For example, right in the middle of the
	// Voronoi region, nearest point is the same for each pixel. So the core
	// of this optimization is "downsampling" - we calculate real distances
	// and point indices only for every fourth pixel (even x and y) in the 1st
	// cycle. And then in the second cycle, we're picking nearest point only
	// from 4 neighboring pixels, so most of the time (75%) we're only calc-
	// ulating 4 distances instead of N distances.
	//////////////////////////////////////////////////////////////////////////

	// If we don't need the second distances buffer, we will just put the second
	// distance index into the stub, thus wasting less memory from the pregen buf
	unsigned char * pIndBuf2 = &indBuf2_Stub;
	for (y = 0; y < yTS; y+=2)
	{
		for (x = 0; x < xTS; x+=2)
		{
			if (isDistance2Needed)
			{
				pIndBuf2 = indBuffer2 + (x+yAdd);
			}

			DtNP1 = DtNP2 = 0xFFffFFff;
			if (precalcDist)
			{
				for (i = 0; i < numPoints; i++)
				{
					Dist = Array[(((x+*(xCoords+i))<<shX)&255)+((((y+*(yCoords+i))<<shY)&255)<<8)];

					if (Dist <= DtNP1)
					{
						DtNP2 = DtNP1;
						DtNP1 = Dist;
						*pIndBuf2 = indBuffer1[x+yAdd];
						indBuffer1[x+yAdd] = closestIdx = i;
					} else if (Dist <= DtNP2)
					{
						*pIndBuf2 = i;
						DtNP2 = Dist;
					}
				}
			}
			else
			{
				for (i = 0; i < numPoints; i++)
				{
					// WrapDist
					//////////////////////////////////////////////////////////////////////////
					// Probably implement a lookup table here [2 * min(xTS,yTS) or just very huge like 1024 to map negatives too]
					int dx = x - *(xCoords+i);
					int dy = y - *(yCoords+i);

					WrapDist(dx, dy);

					Dist = dx*dx + dy*dy;
					//////////////////////////////////////////////////////////////////////////

					if (Dist <= DtNP1)
					{
						DtNP2 = DtNP1;
						DtNP1 = Dist;
						*pIndBuf2 = indBuffer1[x+yAdd];
						indBuffer1[x+yAdd] = closestIdx = i;
					} else if (Dist <= DtNP2)
					{
						*pIndBuf2 = i;
						DtNP2 = Dist;
					}
				}

				DtNP1 = (unsigned int)sqrtf((float)DtNP1);
				DtNP2 = (unsigned int)sqrtf((float)DtNP2);
			}

			unsigned char dist;
			if		(calcMode == 0) dist = clamp<int>(DtNP1, 0, 255);
			else if (calcMode == 1) dist = clamp<int>((DtNP2 * DtNP1) >> 5, 0, 255);
			else if (calcMode == 2) dist = clamp<int>(DtNP2 - DtNP1, 0, 255);
			else if (calcMode == 3) dist = clamp<int>((DtNP1 << 5) / DtNP2, 0, 255);

			if (!skipFirstPass)
			{
				int addColor = (Variance * (((indBuffer1[x+yAdd]<<2)+342561)&255)) >> 8;
				if		(drawMode == 0 || drawMode == 4)
					Layer[bVal+((x+yAdd)<<2)+ch] = clamp<int>(((dist * Power) >> 5) - addColor, 0, 255);
				else if (drawMode == 1)
					Layer[bVal+((x+yAdd)<<2)+ch] = 255 - clamp<int>(((dist * Power) >> 5) - addColor, 0, 255);
				else if (drawMode == 2)
					Layer[bVal+((x+yAdd)<<2)+ch] = (dist >= Power)?(255-addColor):0;
			}
		}
		yAdd += 2*xTS;
	}

	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			DtNP1 = DtNP2 = 0xFFffFFff;

			if ((x&1) == 0 && (y&1) == 0)
				continue;

			int x_0 = (x & (~1));
			int y_0 = (y & (~1));
			int x_1 = (x_0+2)&(xTS-1);
			int y_1 = (y_0+2)&(yTS-1);

			y_0 *= xTS;
			y_1 *= xTS;

			int numIndexes = 0;

			int idxes[8];

#define CHECK_IDX(idx) \
			{ \
				int ci; \
				for (ci = 0; ci < numIndexes; ++ci) \
				{ \
					if (idxes[ci] == idx) \
						break; \
				} \
				if (ci == numIndexes) \
				{ \
					idxes[numIndexes] = idx; \
					++numIndexes; \
				} \
			}

			CHECK_IDX(indBuffer1[x_0+y_0]);
			CHECK_IDX(indBuffer1[x_0+y_1]);
			CHECK_IDX(indBuffer1[x_1+y_1]);
			CHECK_IDX(indBuffer1[x_1+y_0]);
			if (isDistance2Needed)
			{
				CHECK_IDX(indBuffer2[x_0+y_0]);
				CHECK_IDX(indBuffer2[x_0+y_1]);
				CHECK_IDX(indBuffer2[x_1+y_1]);
				CHECK_IDX(indBuffer2[x_1+y_0]);
			}

#undef CHECK_IDX

			if (precalcDist)
			{
				for (int idx = 0; idx < numIndexes; idx++)
				{
					i = idxes[idx];

					Dist = Array[(((x+*(xCoords+i))<<shX)&255)+((((y+*(yCoords+i))<<shY)&255)<<8)];

					if (Dist <= DtNP1)
					{
						DtNP2 = DtNP1;
						DtNP1 = Dist;
						indBuffer1[x+yAdd] = closestIdx = i;
					} else if (Dist <= DtNP2) DtNP2 = Dist;
				}
			}
			else
			{
				for (int idx = 0; idx < numIndexes; idx++)
				{
					i = idxes[idx];

					// WrapDist
					//////////////////////////////////////////////////////////////////////////
					// Probably implement a lookup table here [2 * min(xTS,yTS) or just very huge like 1024 to map negatives too]
					int dx = x - *(xCoords+i);
					int dy = y - *(yCoords+i);

					WrapDist(dx, dy);

					Dist = dx*dx + dy*dy;
					//////////////////////////////////////////////////////////////////////////

					if (Dist <= DtNP1)
					{
						DtNP2 = DtNP1;
						DtNP1 = Dist;
						indBuffer1[x+yAdd] = closestIdx = i;
					} else if (Dist <= DtNP2) DtNP2 = Dist;
				}

				DtNP1 = (unsigned int)sqrtf((float)DtNP1);
				DtNP2 = (unsigned int)sqrtf((float)DtNP2);
			}

			unsigned char dist;
			if		(calcMode == 0) dist = clamp<int>(DtNP1, 0, 255);
			else if (calcMode == 1) dist = clamp<int>((DtNP2 * DtNP1) >> 5, 0, 255);
			else if (calcMode == 2) dist = clamp<int>(DtNP2 - DtNP1, 0, 255);
			else if (calcMode == 3) dist = clamp<int>((DtNP1 << 5) / DtNP2, 0, 255);

			if (!skipFirstPass)
			{
				int addColor = (Variance * (((indBuffer1[x+yAdd]<<2)+342561)&255)) >> 8;
				if		(drawMode == 0 || drawMode == 4)
					Layer[bVal+((x+yAdd)<<2)+ch] = clamp<int>(((dist * Power) >> 5) - addColor, 0, 255);
				else if (drawMode == 1)
					Layer[bVal+((x+yAdd)<<2)+ch] = 255 - clamp<int>(((dist * Power) >> 5) - addColor, 0, 255);
				else if (drawMode == 2)
					Layer[bVal+((x+yAdd)<<2)+ch] = (dist >= Power)?(255-addColor):0;
			}
		}
		yAdd += xTS;
	}
#endif

	yAdd = 0;
	if (additionalPass) 
	{
		for (y = 0; y < yTS; y++)
		{
			for (x = 0; x < xTS; x++)
			{
				int addColor = (Variance * (((indBuffer1[x+yAdd]<<2)+342561)&255)) >> 8;

				if (drawMode == 3)
					Layer[bVal+((x+yAdd)<<2)+ch] = ((indBuffer1[((x+Power)&(xTS-1))+yAdd] - indBuffer1[x+yAdd] == 0) && (indBuffer1[x+((y+Power)&(yTS-1))*xTS] - indBuffer1[x+yAdd] == 0))?(255-addColor):0;
				else if (drawMode == 4)
				{
					Layer[bVal+((x+yAdd)<<2)+ch] = ((indBuffer1[((x+1)&(xTS-1))+yAdd] - indBuffer1[x+yAdd] == 0) && (indBuffer1[x+((y+1)&(yTS-1))*xTS] - indBuffer1[x+yAdd] == 0))?Layer[bVal+((x+yAdd)<<2)+ch]:(255-addColor);
				}
			}
			yAdd += xTS;
		}
	}

#if 0
	for (i = 0; i < numPoints; i++)
	{
		int xCoord = xCoords[i];
		int yCoord = yCoords[i];

		Layer[bVal+((xCoord+yCoord*xTS)<<2)+1] = 255;
	}
#endif

#if (USE_RAW_CHUNK == 0)
	free(distBuffer);
	free(indBuffer1);
	free(indBuffer2);
#endif

#undef USE_RAW_CHUNK
}

void TexGen::PointsWhite(int gL, int k)
{
	unsigned char color;
	int bVal = gL * xTS * yTS << 2;

	for (int i = 0; i < xTS * yTS; i++)
	{
		color = 255;
		if (rand()%k < k - 1) color = 0;
		Layer[bVal + (i << 2)    ] = color;
		Layer[bVal + (i << 2) + 1] = color;
		Layer[bVal + (i << 2) + 2] = color;
	}
}

void TexGen::PointsBlack(int gL, int k)
{
	unsigned char color;
	int bVal = gL * xTS * yTS * 4;

	for (int i = 0; i < xTS * yTS; i++)
	{
		color = 0;
		if (rand()%(k + 1) < k) color = 255;
		Layer[bVal + (i << 2)    ] = color;
		Layer[bVal + (i << 2) + 1] = color;
		Layer[bVal + (i << 2) + 2] = color;
	}
}

void TexGen::FillColor(int gL, unsigned char r, unsigned char g, unsigned char b)	// Filling texMatrix with one color
{
	int i, yAdd = 0;
	int bVal = gL * xTS * yTS << 2;

	unsigned char *lVal = Layer+bVal;
	for (i = 0; i < xTS*yTS; i++)
	{
		*(lVal+(i<<2)  ) = r;
		*(lVal+(i<<2)+1) = g;
		*(lVal+(i<<2)+2) = b;
	}
}

void TexGen::Angles(unsigned char lParam, int Power, unsigned char aNum)
{
	int x, y, ch = lParam & 0xF;
	int bVal = (lParam >> 4) * xTS * yTS << 2;

	Power = 255 - Power;
	Power <<= 3;

	unsigned char *lPos = Layer + bVal + ch;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			*lPos = clamp<int>((255 - Power) + int(Power * sinf(aNum * PArcTan(float(x - (xTS >> 1)), float(y - (yTS >> 1))) + PI / 2)), 0, 255);
			lPos += 4;
		}
	}
}

void TexGen::Perlin(unsigned char lParam, char Step, bool absMode)
{
#if 0
	int gradX[8] = {  256, -256,    0,    0,  256, -256,    0,    0 };
	int gradY[8] = {    0,    0,  256, -256,    0,    0,  256, -256 };
#else

#if 0
	int gradX[8] = { 0, 181, 256, 181, 0, -181, -256, -181 };
	int gradY[8] = { 256, 181, 0, -181, -256, -181, 0, 181 };
#else
	// Slightly randomized (and normalized) version of the above gradients
//	int gradX[8] = {  -24,  181,  255,  182,   -9, -192, -255, -192 };
//	int gradY[8] = {  254,  180,   -2, -179, -255, -168,   15,  168 };
	int gradX[8] = {  -44,  181,  255,  182,  -16, -201, -254, -200 };
	int gradY[8] = {  252,  180,   -4, -179, -255, -158,   27,  158 };
#endif

#endif

	PerlinCustom(lParam, Step, gradX, gradY, absMode);

}

void TexGen::PerlinCustom(unsigned char lParam, char Step, int * gradX, int * gradY, bool absMode)
{
	int x, y, stepX, stepY, Step_Int = 1 << Step;
	char ch = lParam & 0xF, xTS_2pow = get2Pow(xTS);
	int bVal = (((lParam & 0xF0)>>4) * yTS) << (2 + xTS_2pow);
	int yAdd = 0;
	unsigned char *lPos;

	unsigned char* temp = mTempLayer;

	lPos = temp + bVal + ch;
	for (y = 0; y < yTS; y += Step_Int)
	{
		for (x = 0; x < xTS; x += Step_Int)
			*(lPos+((x+yAdd)<<2)) = rand()&7;
		yAdd += xTS << Step;
	}

	unsigned char *clPos, *clPos_y0, *clPos_y1;

	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		stepY = y & (Step_Int - 1);
		int cell_y0 = y - stepY;
		int cell_y1 = (cell_y0 + Step_Int) & (yTS-1);

		clPos = Layer+bVal+(yAdd<<2)+ch;
		clPos_y0 = temp+bVal+(cell_y0<<(2+xTS_2pow))+ch;
		clPos_y1 = temp+bVal+(cell_y1<<(2+xTS_2pow))+ch;

		for (x = 0; x < xTS; x++)
		{
			stepX = x & (Step_Int - 1);

			int cell_x0 = x - stepX;
			int cell_x1 = (cell_x0 + Step_Int) & (xTS-1);

			unsigned char
				v_x0y0 = clPos_y0[cell_x0<<2],
				v_x0y1 = clPos_y1[cell_x0<<2],
				v_x1y0 = clPos_y0[cell_x1<<2],
				v_x1y1 = clPos_y1[cell_x1<<2];

			int dx = (stepX << 8) >> Step;
			int dy = (stepY << 8) >> Step;
			
			// dx - 1, dy - 1
			int dx_i = dx - 255;
			int dy_i = dy - 255;

			// |g| = 256
			int g00_dot_dx = (gradX[v_x0y0] * dx   + gradY[v_x0y0] * dy  ) >> 8;
			int g01_dot_dx = (gradX[v_x0y1] * dx   + gradY[v_x0y1] * dy_i) >> 8;
			int g11_dot_dx = (gradX[v_x1y1] * dx_i + gradY[v_x1y1] * dy_i) >> 8;
			int g10_dot_dx = (gradX[v_x1y0] * dx_i + gradY[v_x1y0] * dy  ) >> 8;

#if 1
			// Cubic
			int ipX = ( (3 * stepX*stepX - ((2 * stepX*stepX*stepX) >> (Step))) << 8 ) >> Step;
			int ipY = ( (3 * stepY*stepY - ((2 * stepY*stepY*stepY) >> (Step))) << 8 ) >> Step;
#else
			// Lerp
			int ipX = stepX << 8;
			int ipY = stepY << 8;
#endif

			// Interpolation
			int a = g00_dot_dx + ((ipX * (g10_dot_dx - g00_dot_dx)) >> (8+Step));
			int b = g01_dot_dx + ((ipX * (g11_dot_dx - g01_dot_dx)) >> (8+Step));

			int color = a + ((ipY * (b - a)) >> (8+Step));
			*(clPos+(x<<2)) = clamp(color + 127, 0, 255);
		}

		yAdd += xTS;
	}
}

void TexGen::Sub(unsigned char lParam, char Step, bool absMode)
{
	__int16 x, y, step, Step_Int = 1 << Step;
	char ch = lParam & 0xF, xTS_2pow = get2Pow(xTS);
	int bVal = (((lParam & 0xF0)>>4) * yTS) << (2 + xTS_2pow);
	int yAdd = 0;
	unsigned char *lPos;

	lPos = Layer + bVal + ch;
	for (y = 0; y < yTS; y += Step_Int)
	{
		for (x = 0; x < xTS; x += Step_Int)
			*(lPos+((x+yAdd)<<2)) = rand()&255;
		yAdd += xTS << Step;
	}

	__int32 c0, c1, c2, c3;
	unsigned char *clPos;
			
	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		if ((y & (Step_Int - 1)) == 0)
		{
			clPos = Layer+bVal+(yAdd<<2)+ch;
			for (x = 0; x < xTS; x++)
			{
				step = x & (Step_Int - 1);
				if (step == 0)
				{
					c1 = (x&(255<<Step))&(xTS-1);
					c2 = (c1 + Step_Int)&(xTS-1);
					c3 = (c2 + Step_Int)&(xTS-1);
					c0 = (c1 - Step_Int)&(xTS-1);
				}

				*(clPos+(x<<2)) = clamp((((*(clPos+(c1<<2)))<<1)+((((*(clPos+(c2<<2)))-(*(clPos+(c0<<2))))*step)>>Step)+(((((*(clPos+(c0<<2)))<<1)-((*(clPos+(c1<<2)))*5)+((*(clPos+(c2<<2)))<<2)-(*(clPos+(c3<<2))))*step*step)>>(Step<<1))+(((((*(clPos+(c1<<2)))*3)-(*(clPos+(c0<<2)))-((*(clPos+(c2<<2)))*3)+(*(clPos+(c3<<2))))*step*step*step)>>(Step*3))) >> 1, 0, 255);
			}
		}
		yAdd += xTS;
	}

	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		if ((y & (Step_Int - 1)) != 0)
		{
			step = y & (Step_Int - 1);
			c1 = (y&(255<<Step))&(yTS-1);
			c2 = (c1 + Step_Int)&(yTS-1);
			c3 = (c2 + Step_Int)&(yTS-1);
			c0 = (c1 - Step_Int)&(yTS-1);
			c0 <<= xTS_2pow; c1 <<= xTS_2pow; c2 <<= xTS_2pow; c3 <<= xTS_2pow; 

			if (absMode)
			{
				for (x = 0; x < xTS; x++)
				{
					clPos = Layer+bVal+(x<<2)+ch;
					int val = (((*(clPos+(c1<<2)))<<1)+((((*(clPos+(c2<<2)))-(*(clPos+(c0<<2))))*step)>>Step)+(((((*(clPos+(c0<<2)))<<1)-((*(clPos+(c1<<2)))*5)+((*(clPos+(c2<<2)))<<2)-(*(clPos+(c3<<2))))*step*step)>>(Step<<1))+(((((*(clPos+(c1<<2)))*3)-(*(clPos+(c0<<2)))-((*(clPos+(c2<<2)))*3)+(*(clPos+(c3<<2))))*step*step*step)>>(Step*3))) >> 1;
					*(clPos+(yAdd<<2)) = clamp(abs(val - 127) << 1, 0, 255);
				}
			}
			else
			{
				for (x = 0; x < xTS; x++)
				{
					clPos = Layer+bVal+(x<<2)+ch;
					*(clPos+(yAdd<<2)) = clamp((((*(clPos+(c1<<2)))<<1)+((((*(clPos+(c2<<2)))-(*(clPos+(c0<<2))))*step)>>Step)+(((((*(clPos+(c0<<2)))<<1)-((*(clPos+(c1<<2)))*5)+((*(clPos+(c2<<2)))<<2)-(*(clPos+(c3<<2))))*step*step)>>(Step<<1))+(((((*(clPos+(c1<<2)))*3)-(*(clPos+(c0<<2)))-((*(clPos+(c2<<2)))*3)+(*(clPos+(c3<<2))))*step*step*step)>>(Step*3))) >> 1, 0, 255);
				}
			}
		}
		yAdd += xTS;
	}

	if (absMode)
	{
		yAdd = 0;
		clPos = Layer+bVal+ch;
		for (y = 0; y < yTS; y++)
		{
			if ((y & (Step_Int - 1)) == 0)
			{
				for (x = 0; x < xTS; x++)
				{
					*(clPos+((x+yAdd)<<2)) = clamp<int>(abs((int)*(clPos+((x+yAdd)<<2)) - 127) << 1, 0, 255);
				}
			}
			yAdd += xTS;
		}
	}
}

void TexGen::SubLin(unsigned char lParam, char Step, bool absMode)
{
	int x, y;
	int bVal = (((lParam & 0xF0)>>4) * xTS * yTS) << 2;
	char ch = lParam & 0xF;
	int yAdd = 0;
	int Step_Int = 1 << Step;
	unsigned char *lPos;

	lPos = Layer + bVal + ch;
	for (y = 0; y < yTS; y += Step_Int)
	{
		for (x = 0; x < xTS; x += Step_Int)
			*(lPos+((x+yAdd)<<2)) = rand()&255;
		yAdd += xTS << Step;
	}
	
	int c0, c1;
	unsigned char *clPos;

	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		if ((y & (Step_Int - 1)) == 0)
		{
			clPos = Layer+bVal+(yAdd<<2)+ch;
			for (x = 0; x < xTS; x++)
			{
				c0 = (x & (255 << Step))&(xTS - 1);
				c1 = (c0 + Step_Int)	&(xTS - 1);

				*(clPos+(x<<2)) = clamp<int>((((*(clPos+(c1<<2)) - *(clPos+(c0<<2))) * (x & (Step_Int - 1))) >> Step) + *(clPos+(c0<<2)), 0, 255);
			}
		}
		yAdd += xTS;
	}

	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		if ((y & (Step_Int - 1)) != 0)
		{
			c0 = (y & (255 << Step))&(yTS - 1);
			c1 = (c0 + Step_Int)	&(yTS - 1);
			c0 *= xTS; c1 *= xTS;

			if (absMode)
			{
				for (x = 0; x < xTS; x++)
				{
					clPos = Layer+bVal+(x<<2)+ch;
					int val = (((*(clPos+(c1<<2)) - *(clPos+(c0<<2))) * (y & (Step_Int - 1))) >> Step) + *(clPos+(c0<<2));
					*(clPos+(yAdd<<2)) = clamp(abs(val - 127) << 1, 0, 255);
				}
			}
			else
			{
				for (x = 0; x < xTS; x++)
				{
					clPos = Layer+bVal+(x<<2)+ch;
					*(clPos+(yAdd<<2)) = clamp<int>((((*(clPos+(c1<<2)) - *(clPos+(c0<<2))) * (y & (Step_Int - 1))) >> Step) + *(clPos+(c0<<2)), 0, 255);
				}
			}
		}
		yAdd += xTS;
	}

	if (absMode)
	{
		yAdd = 0;
		clPos = Layer+bVal+ch;
		for (y = 0; y < yTS; y++)
		{
			if ((y & (Step_Int - 1)) == 0)
			{
				for (x = 0; x < xTS; x++)
				{
					*(clPos+((x+yAdd)<<2)) = clamp<int>(abs((int)*(clPos+((x+yAdd)<<2)) - 127) << 1, 0, 255);
				}
			}
			yAdd += xTS;
		}
	}
}

void TexGen::FractalP(unsigned char lParam, unsigned char St_Oc, uint persistence, unsigned char absMode, unsigned char isLinear)
{
	int SizeF = xTS * yTS;
	unsigned char* temp = mTempLayer;
	int bVal = ((lParam & 0xF0) >> 4) * (SizeF<<2);
	int i, iend, v, ch = lParam & 0xF;
	int numOctaves = (St_Oc & 0xF);
	int Step = St_Oc >> 4, Step_c = 128;

	bool globalAbsMode = (absMode >> 4) != 0;
	bool localAbsMode = (absMode & 0xF) != 0;

	memset(temp, 0, SizeF << 2);
	
	// Calculate normalization parameters for persistence
	float k = persistence / 512.0f;
	float maxAmp = 0.0f;
	for (i = 0, iend = numOctaves; i < iend; i++)
	{
		if (i > Step)
			break;

		float mul = 1.0f;
		for (int j = 0; j < i; ++j)
			mul *= k;

		maxAmp += mul;
	}
	
	int amp = (int)((1.0f / maxAmp) * 1024); // baseAmp
	int mulAmp = (int)(k * 1024);

	unsigned char *tPos, *lPos;
	for (i = 0; i < numOctaves - 1; i++)
	{
		if (Step < 0) break;
		// amp8 is equal to * 0.008 - we probably don't care about that low amps
		if (amp <= 8) break;
		
		if (isLinear == 0)
			Sub(lParam, Step, localAbsMode);
		else
			SubLin(lParam, Step, localAbsMode);

		tPos = temp + ch;
		lPos = Layer + bVal + ch;
		for (v = 0; v < SizeF; v++)
		{
			*tPos = (*tPos) + (((*lPos) * amp) >> 10);
			tPos += 4; lPos += 4;
		}
		Step--; amp = ((amp * mulAmp) >> 10);
	}

	tPos = temp + ch;
	lPos = Layer + bVal + ch;
	if (globalAbsMode)
	{
		for (v = 0; v < SizeF; v++)
		{
			*lPos = clamp<int>(abs((int)*tPos - 127) << 1, 0, 255);
			tPos += 4; lPos += 4;
		}
	}
	else
	{
		for (v = 0; v < SizeF; v++)
		{
			*lPos = *tPos;
			tPos += 4; lPos += 4;
		}
	}
}

void TexGen::SinePlasm(char gL, unsigned char params) // (colX | colY)
{
	unsigned char value;

	float xZ = ((params >>  4) << 1) * PI / (float)xTS;
	float yZ = ((params & 0xF) << 1) * PI / (float)yTS;
	
	int x, y;

	char * xArray = (char *)malloc(xTS * sizeof(char));
	char * yArray = (char *)malloc(yTS * sizeof(char));
	
	for (x = 0; x < xTS; x++)
		xArray[x] = int(127 * sinf(x * xZ));
	for (y = 0; y < yTS; y++)
		yArray[y] = int(127 * sinf(y * yZ));

	unsigned char *lPos = Layer + (gL * xTS * yTS<<2);
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			value = 127 + ((xArray[x] + yArray[y])>>1);
			*lPos++ = value;
			*lPos++ = value;
			*lPos++ = value;
			lPos++;
		}	
	}

	free(xArray);
	free(yArray);
}

void TexGen::Noise(int gL, short k)
{
	int color;
	int bVal = gL * xTS * yTS << 2;

	for (int i = 0; i < xTS * yTS; i++)
	{
		color = rand()%(k * 10);
		if (color > 255) color = 0;
		Layer[bVal+(i<<2)  ] = color;
		Layer[bVal+(i<<2)+1] = color;
		Layer[bVal+(i<<2)+2] = color;
	}
}

void TexGen::Random(unsigned char lParam, unsigned char maxVal)
{
	int bVal = ((lParam & 0xF0) >> 4) * ((xTS * yTS) << 2);
	int ch = lParam & 0xF;

	for (int i = 0; i < xTS * yTS; i++)
	{
		Layer[bVal+(i<<2)+ch] = rand()%maxVal;
	}
}

void TexGen::Env(int gL, unsigned char Dyn1, unsigned char Dyn2, unsigned char Inversed)
{
	int x, y, d;
	int bVal = gL * xTS * yTS << 2;

	int yP1, yP2;
	int    h2 = yTS >> 1;
	float ih2 = 1.0f / (float)h2;
	int    w2 = xTS >> 1;
	float iw2 = 1.0f / (float)w2;
	float r1 = sqr(Dyn1 / 32.0f);
	float r2 = sqr(Dyn2 / 32.0f);
	float fMax = 0.0f, fx, fy;

	for (x = 0; x < 999; x++)
	{
		fx = x / 1000.0f;
		fx = expf(r1*logf(fx) + r2*logf(1-fx));
		if (fx > fMax) fMax = fx;
	} fMax = 252.45f / fMax;

	for (y = 0; y < (yTS >> 1); y++)
	{
		fy = sqr((y-h2)*ih2);

		yP1 = y * xTS;
		yP2 = (yTS-1-y) * xTS;

		for (x = 0; x < (xTS >> 1); x++)
		{
			fx = sqr((x-w2)*iw2)+fy;
			if (fx < 1) d = int(fMax * expf(r1*logf(fx) + r2*logf(1-fx)));
			else d = -1;

			if (d < 0) d = 0;
			else if (Inversed) d = 255 - d;
			
			for (int i = 0; i < 3; i++)
			{
				Layer[bVal+(((xTS-1-x)+yP1)<<2)+i] = d;
				Layer[bVal+(((      x)+yP1)<<2)+i] = d;
				Layer[bVal+(((      x)+yP2)<<2)+i] = d;
				Layer[bVal+(((xTS-1-x)+yP2)<<2)+i] = d;
			}
		}
	}
}

void TexGen::EnvAdd(int gL, int ox, int oy, unsigned char rad, int mul, unsigned char Inversed)
{
	int x, y, xe, ye;
	int bVal = gL * xTS * yTS << 2;

	int radSquared = rad * rad;
	unsigned char rad_2 = rad >> 1;

	bool skipOne = rad & 1;

	int yAdd1, yAdd2, yend = oy + rad, xend = ox + rad;
	for (y = 0, ye = rad+1; y < ye; y++)
	{
		yAdd1 = ((y + oy - rad) & (yTS - 1)) * xTS;
		yAdd2 = ((yend - y) & (yTS - 1)) * xTS;

		bool skipY2 = (y == rad);

		for (x = 0, xe = rad+1; x < xe; x++)
		{
			int x1 = (x + ox - rad) & (xTS - 1);
			int x2 = (xend - x) & (xTS - 1);

			int dist = (rad-x)*(rad-x) + (rad-y)*(rad-y);
			if (dist >= radSquared)
				continue;

			bool skipX2 = (x == rad);

			int color = 255 - (int)(255 * (sqrtf((float)dist) / (float)rad));

			if (Inversed) color = 255 - color;

			int premulColor = color;
			color = ((int)color * mul) / 255;

			for (int i = 0; i < 3; i++)
			{
#if 0
				// Linear interpolation based on the color
				if (!skipX2)
					Layer[bVal+((x2+yAdd1)<<2)+i] = (premulColor * color + (255 - premulColor) * Layer[bVal+((x2+yAdd1)<<2)+i]) >> 8;
					Layer[bVal+((x1+yAdd1)<<2)+i] = (premulColor * color + (255 - premulColor) * Layer[bVal+((x1+yAdd1)<<2)+i]) >> 8;
				if (!skipY2)
					Layer[bVal+((x1+yAdd2)<<2)+i] = (premulColor * color + (255 - premulColor) * Layer[bVal+((x1+yAdd2)<<2)+i]) >> 8;
				if (!skipX2 && !skipY2)
					Layer[bVal+((x2+yAdd2)<<2)+i] = (premulColor * color + (255 - premulColor) * Layer[bVal+((x2+yAdd2)<<2)+i]) >> 8;
#else
				// Simply add to the base layer
				if (!skipX2)
					Layer[bVal+((x2+yAdd1)<<2)+i] = clamp<int>(color + Layer[bVal+((x2+yAdd1)<<2)+i], 0, 255);
					Layer[bVal+((x1+yAdd1)<<2)+i] = clamp<int>(color + Layer[bVal+((x1+yAdd1)<<2)+i], 0, 255);
				if (!skipY2)
					Layer[bVal+((x1+yAdd2)<<2)+i] = clamp<int>(color + Layer[bVal+((x1+yAdd2)<<2)+i], 0, 255);
				if (!skipX2 && !skipY2)
					Layer[bVal+((x2+yAdd2)<<2)+i] = clamp<int>(color + Layer[bVal+((x2+yAdd2)<<2)+i], 0, 255);
#endif
			}
		}
	}
}

void TexGen::RadGrad(unsigned char lParam, int cenX, int cenY, int * colors, unsigned int numColors)
{
	int bVal = ((lParam & 0xF0) >> 4) * ((xTS * yTS) << 2);
	int ch = lParam & 0xF;

	int x, y, yAdd = 0;

	for (y = 0; y < yTS; y++)
	{
		int diff_y = y - cenY;

		for (x = 0; x < xTS; x++)
		{
			int diff_x = x - cenX;

			float fpIdxColor = numColors * (atan2f((float)diff_x, (float)diff_y) + PI) / _2PI;
			int idxColor = (int)( fpIdxColor );
			float fracIdxColor = fpIdxColor - idxColor;

			// In case FP-error caused the original index to overflow
			if (idxColor >= (int)numColors)
				idxColor = (int)(numColors-1);

			int idxColor1 = idxColor + 1;
			if (idxColor1 >= (int)numColors)
				idxColor1 = 0;

#if 1
			// LERP
			int color = (int)( colors[idxColor] * (1.0f - fracIdxColor) + colors[idxColor1] * fracIdxColor );
#else
			int color = colors[x];
#endif

			Layer[bVal+((x+yAdd)<<2)+ch] = color;
		}
		yAdd += xTS;
	}
}
void TexGen::RadGradRGB(int gL, int cenX, int cenY, int * colors, unsigned int numColors)
{
	int bVal = (gL >> 4) * ((xTS * yTS) << 2);

	int x, y, yAdd = 0;

	for (y = 0; y < yTS; y++)
	{
		int diff_y = y - cenY;

		for (x = 0; x < xTS; x++)
		{
			int diff_x = x - cenX;

			float fpIdxColor = numColors * (atan2f((float)-diff_x, (float)-diff_y) + PI) / _2PI;
			int idxColor = (int)( fpIdxColor );
			float fracIdxColor = fpIdxColor - idxColor;
			int idxColor1 = idxColor + 1;
			if (idxColor1 >= (int)numColors)
				idxColor1 = 0;

			float invFrac = 1.0f - fracIdxColor;

			// LERP
			int colorR = (int)( colors[idxColor*3+0] * invFrac + colors[idxColor1*3+0] * fracIdxColor );
			int colorG = (int)( colors[idxColor*3+1] * invFrac + colors[idxColor1*3+1] * fracIdxColor );
			int colorB = (int)( colors[idxColor*3+2] * invFrac + colors[idxColor1*3+2] * fracIdxColor );

			Layer[bVal+((x+yAdd)<<2)+0] = colorR;
			Layer[bVal+((x+yAdd)<<2)+1] = colorG;
			Layer[bVal+((x+yAdd)<<2)+2] = colorB;
		}
		yAdd += xTS;
	}
}

void TexGen::Rays(int gL, int RayCol, int D)
{
	int bVal = (gL * xTS * yTS) << 2;
	int mTS = m_min(xTS, yTS);

	int i, j;

	int xPos, yPos, xTS_2Pow = get2Pow(xTS);
	int cR, cG, cB;

	for (j = 0; j < RayCol; j++)
	{
		float rad = PI * float(rand()%360) / 180.0f;
		float xOffset = cosf(rad);
		float yOffset = sinf(rad);

		int dist = rand()%(D / 2);

		for (i = 0; i < dist; i++)
		{
			xPos = (int((xTS >> 1) + i * xOffset + 0.5f)) & (xTS-1);
			yPos = (int((yTS >> 1) + i * yOffset + 0.5f)) & (yTS-1);

			cR = ((dist - i)<<8) / dist + Layer[bVal+((xPos+(yPos<<xTS_2Pow))<<2)  ] * i / dist;
			cG = ((dist - i)<<8) / dist + Layer[bVal+((xPos+(yPos<<xTS_2Pow))<<2)+1] * i / dist;
			cB = ((dist - i)<<8) / dist + Layer[bVal+((xPos+(yPos<<xTS_2Pow))<<2)+2] * i / dist;

			Layer[bVal+((xPos+(yPos<<xTS_2Pow))<<2)  ] = clamp<int>(cR, 0, 255);
			Layer[bVal+((xPos+(yPos<<xTS_2Pow))<<2)+1] = clamp<int>(cG, 0, 255);
			Layer[bVal+((xPos+(yPos<<xTS_2Pow))<<2)+2] = clamp<int>(cB, 0, 255);
		}
	}
}

void TexGen::FillPieces(int gL, unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2)
{
	int bVal = gL * xTS * yTS << 2;
	unsigned char* temp = mTempLayer;
	int i, j;
	int x, y;
	int xTS_2Pow = get2Pow(xTS);

	for (j = y1; j <= y2; j++)
		for (i = x1; i <= x2; i++)
		{
			temp[((i-x1+((j-y1)<<xTS_2Pow))<<2)  ] = Layer[bVal+((i+(j<<xTS_2Pow))<<2)  ];
			temp[((i-x1+((j-y1)<<xTS_2Pow))<<2)+1] = Layer[bVal+((i+(j<<xTS_2Pow))<<2)+1];
			temp[((i-x1+((j-y1)<<xTS_2Pow))<<2)+2] = Layer[bVal+((i+(j<<xTS_2Pow))<<2)+2];
		}

	for (j = 0; j < yTS / (y2-y1) + 1; j++)
	{
		for (i = 0; i < xTS / (x2-x1) + 1; i++)
		{
			for (y = 0; y < y2-y1; y++)
			{
				for (x = 0; x < x2-x1; x++)
				{
					int xPos = i*(x2-x1), yPos = j*(y2-y1);
					if ((xPos+x >= xTS) || (yPos+y >= yTS)) continue;
					Layer[bVal+((xPos+x+((yPos+y)<<xTS_2Pow))<<2)  ] = temp[((x+(y<<xTS_2Pow))<<2)  ];
					Layer[bVal+((xPos+x+((yPos+y)<<xTS_2Pow))<<2)+1] = temp[((x+(y<<xTS_2Pow))<<2)+1];
					Layer[bVal+((xPos+x+((yPos+y)<<xTS_2Pow))<<2)+2] = temp[((x+(y<<xTS_2Pow))<<2)+2];
				}
			}
		}
	}
}

void TexGen::Cracked(int gL, int Num, int maxLength, int Variation)
{
	int bVal = gL * xTS * yTS << 2;
	
	int Count;
	float x, y, an;

	for (int i = 0; i < Num; i++)
	{
		Count = rand()%maxLength;
		x = float(rand()%xTS);
		y = float(rand()%yTS);
		an = rand()%1000 / 1000.0f * 2 * PI;

		while (--Count >= 0)
		{
			int ix = ((int)x)&(xTS-1), iy = ((int)y)&(yTS-1);
			Layer[bVal+((ix+iy*xTS)<<2)  ] = 255;
			Layer[bVal+((ix+iy*xTS)<<2)+1] = 255;
			Layer[bVal+((ix+iy*xTS)<<2)+2] = 255;

			x += cosf(an);
			y += sinf(an);

			an += Variation / 255.0f * (rand()%2000 / 1000.0f - 1.0f);
		}
	}
}

/* Distortion */
void TexGen::RotoZoom(int gL, float AngRad, unsigned char Scale)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y, yAdd = 0;
	float xPos, yPos;
	unsigned char* temp = mTempLayer;

	float rRad = AngRad;

	float preCos = cosf(rRad) / (float)Scale * 64.0f;
	float preSin = sinf(rRad) / (float)Scale * 64.0f;

	float xp, yp;

	xp = (xTS >> 1) + (yTS >> 1) * preSin - (xTS >> 1) * preCos;
	yp = (yTS >> 1) - (yTS >> 1) * preCos - (xTS >> 1) * preSin;

	unsigned char *tPos = temp, *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		xPos = xp;
		yPos = yp;
		for (x = 0; x < xTS; x++)
		{
			GetBerpTexel_RGB(lPos, xPos, yPos, tPos, tPos + 1, tPos + 2);
			tPos += 4;

			xPos += preCos;
			yPos += preSin;
		}
		xp -= preSin;
		yp += preCos;
	}
	for (x = 0; x < xTS*yTS<<2; x++) Layer[bVal+x] = temp[x];
}
void TexGen::RotoZoomExt(int gL, float AngRad, float ScaleX, float ScaleY, float ScrollX, float ScrollY)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y, yAdd = 0;
	float xPos, yPos;
	unsigned char* temp = mTempLayer;

	float rRad = AngRad;

	// We need to start off of the center
	ScrollX += 0.5f;
	ScrollY += 0.5f;

	float preCos, preSin;

	// 2D Transformation matrix:
	// [ax bx tx]
	// [ay by ty]
	float m00, m01, m02;
	float m10, m11, m12;

	preCos = cosf(rRad);
	preSin = sinf(rRad);

	m00 =  preCos / ScaleX;
	m10 =  preSin / ScaleX;
	m01 = -preSin / ScaleY;
	m11 =  preCos / ScaleY;
	m02 = ScrollX * xTS - (xTS * m00 + yTS * m01) * 0.5f;
	m12 = ScrollY * yTS - (xTS * m10 + yTS * m11) * 0.5f;

	unsigned char *tPos = temp, *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		xPos = y * m01 + m02;
		yPos = y * m11 + m12;
		for (x = 0; x < xTS; x++)
		{
			GetBerpTexel_RGB(lPos, xPos, yPos, tPos, tPos + 1, tPos + 2);
			tPos += 4;

			xPos += m00;
			yPos += m10;
		}
	}
	for (x = 0; x < xTS*yTS<<2; x++) Layer[bVal+x] = temp[x];
}
void TexGen::RotoZoomMul(int gL, unsigned char times, float AngRad, float ScaleX, float ScaleY, float ScrollX, float ScrollY, unsigned char FadeR, unsigned char FadeG, unsigned char FadeB)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y, yAdd = 0;
	float xPos, yPos;
	unsigned char* temp = mTempLayer;

	float rRad = AngRad;

	// We need to start off of the center
	ScrollX += 0.5f;
	ScrollY += 0.5f;

	float preCos, preSin;

	// 2D Transformation matrix:
	// [ax bx tx]
	// [ay by ty]
	float m00, m01, m02;
	float m10, m11, m12;

	const float radScale = 2.0f;
	const float scrollScale = 2.1f;

	unsigned char *lPosBase = Layer + bVal;
	for (uint i = 0; i < times; ++i)
	{
		preCos = cosf(rRad);
		preSin = sinf(rRad);

		m00 =  preCos / ScaleX;
		m10 =  preSin / ScaleX;
		m01 = -preSin / ScaleY;
		m11 =  preCos / ScaleY;
		m02 = ScrollX * xTS - (xTS * m00 + yTS * m01) * 0.5f;
		m12 = ScrollY * yTS - (xTS * m10 + yTS * m11) * 0.5f;

		unsigned char *tPos = temp, *lPos = lPosBase;
		for (y = 0; y < yTS; y++)
		{
			xPos = y * m01 + m02;
			yPos = y * m11 + m12;
			for (x = 0; x < xTS; x++)
			{
				GetBerpTexel_RGB(lPos, xPos, yPos, tPos, tPos + 1, tPos + 2);
				tPos += 4;

				xPos += m00;
				yPos += m10;
			}
		}
		tPos = temp;
		for (x = 0; x < xTS*yTS; x++)
		{
			*lPos = clamp<int>(*lPos + ( ((int)FadeR * *tPos++) >> 8 ), 0, 255);
			++lPos;
			*lPos = clamp<int>(*lPos + ( ((int)FadeG * *tPos++) >> 8 ), 0, 255);
			++lPos;
			*lPos = clamp<int>(*lPos + ( ((int)FadeB * *tPos++) >> 8 ), 0, 255);
			++lPos;

			// Skip alpha
			++lPos; ++tPos;
		}

		rRad *= radScale;
		ScrollX = (ScrollX - 0.5f) * scrollScale + 0.5f;
		ScrollY = (ScrollY - 0.5f) * scrollScale + 0.5f;

		FadeR = (FadeR * FadeR) >> 8;
		FadeG = (FadeG * FadeG) >> 8;
		FadeB = (FadeB * FadeB) >> 8;
	}
}

void TexGen::Glass(int gL, unsigned char dParam, int k)
{
	int x, y, yAdd = 0, ch = dParam & 0xF;
	int xm1, ym1, xp1, yp1, xTS_2Pow = get2Pow(xTS);
	
	int bVal = ((gL * xTS * yTS) << 2);
	int dVal = (((dParam>>4) * xTS * yTS) << 2);

	k -= 128;

	unsigned char* temp  = mTempLayer;
	unsigned char* tempD = (unsigned char *)malloc((xTS*yTS)*sizeof(unsigned char));
	for (x = 0; x < (xTS*yTS<<2); x++) temp[x]  = Layer[bVal+x];
	for (x = 0; x <  xTS*yTS	; x++) tempD[x] = Layer[dVal+(x<<2)+ch];

	int xOffset, yOffset;

	for (y = 0; y < yTS; y++)
	{
		ym1 = ((y-1)&(yTS-1))*xTS;
		yp1 = ((y+1)&(yTS-1))*xTS;

		for (x = 0; x < xTS; x++)
		{
			xm1 = (x-1)&(xTS-1);
			xp1 = (x+1)&(xTS-1);

			xOffset = (           x + ((tempD[xm1+yAdd]	- tempD[xp1+yAdd]) * k >> 7))&(xTS-1);
			yOffset = (xOffset+(((y + ((tempD[x+ym1]	- tempD[x+yp1]	 ) * k >> 7))&(yTS-1))<<xTS_2Pow))<<2;
			
			Layer[bVal+((x+yAdd)<<2)  ] = *(temp+yOffset  );
			Layer[bVal+((x+yAdd)<<2)+1] = *(temp+yOffset+1);
			Layer[bVal+((x+yAdd)<<2)+2] = *(temp+yOffset+2);
		}
		yAdd += xTS;
	}

	free(tempD);
}

void TexGen::Kaleid(int gL, char cN)
{
	int xTSh = xTS >> 1, yTSh = yTS >> 1;
	int bVal = gL * xTS * yTS << 2;
	unsigned char pColor;
	int x, y, Corners[4], xTS_2Pow = get2Pow(xTS);

	unsigned char *bPos = Layer + bVal;
	for (y = 0; y < yTSh; y++)
	{
		for (x = 0; x < xTSh; x++)
		{
			Corners[0] = (       x +(       y <<xTS_2Pow))<<2;
			Corners[1] = ((xTS-1-x)+(       y <<xTS_2Pow))<<2;
			Corners[2] = ((xTS-1-x)+((yTS-1-y)<<xTS_2Pow))<<2;
			Corners[3] = (       x +((yTS-1-y)<<xTS_2Pow))<<2;

			pColor = *(bPos+*(Corners+cN)  );
			*(bPos+*(Corners  )  ) = pColor;
			*(bPos+*(Corners+1)  ) = pColor;
			*(bPos+*(Corners+2)  ) = pColor;
			*(bPos+*(Corners+3)  ) = pColor;

			pColor = *(bPos+*(Corners+cN)+1);
			*(bPos+*(Corners  )+1) = pColor;
			*(bPos+*(Corners+1)+1) = pColor;
			*(bPos+*(Corners+2)+1) = pColor;
			*(bPos+*(Corners+3)+1) = pColor;

			pColor = *(bPos+*(Corners+cN)+2);
			*(bPos+*(Corners  )+2) = pColor;
			*(bPos+*(Corners+1)+2) = pColor;
			*(bPos+*(Corners+2)+2) = pColor;
			*(bPos+*(Corners+3)+2) = pColor;
		}
	}
}

void TexGen::LensDist(int gL, short slE)
{
	unsigned char* temp = mTempLayer;

	int bVal = gL * xTS * yTS << 2;
	int x, y;
	float R, an;
	int mTS = m_min(xTS, yTS) >> 1;

	short lX = xTS >> 1, lY = yTS >> 1;

	float lE = slE / 256.0f;
	float sqrY, subY;

	float PI_2 = PI / 2.0f;

	unsigned char *lPos = Layer + bVal, *tPos = temp;
	for (y = 0; y < yTS; y++)
	{
		sqrY = float(sqr(y - lY));
		subY = float((yTS-y) - (lY));
		for (x = 0; x < xTS; x++)
		{
			R = sqrtf(float(sqr(x - lX)) + sqrY);
			an = PArcTan(float(x - (lX)), subY) - PI_2;
			if (R < mTS) R = mTS * expf(logf(R / mTS) * lE);

			GetBerpTexel_RGB(lPos, lX + (R*cosf(an)), lY + (R*sinf(an)), tPos, tPos + 1, tPos + 2);
			tPos += 4;
		}
	}

	memcpy(Layer + bVal, temp, (xTS*yTS<<2)*sizeof(unsigned char));
}

void TexGen::DirBlur(unsigned char gL, unsigned char dParams, int dist)
{
	if (dist == 0) return;

	unsigned char* temp = mTempLayer;

	int bVal = gL * xTS * yTS << 2;
	int dVal = (dParams >> 4) * xTS * yTS << 2;
	int x, y, i;
	int ch = dParams & 0xF;

	int cR, cG, cB;
	
	// Optimize this by using memcpy
	// NOTE: tried that and it was actually slower
	for (x = 0; x < xTS*yTS << 2; x++)
		temp[x] = Layer[bVal+x];

	float Angle, PI_180 = PI / 180.0f;
	int cAngle[256], sAngle[256];

	// Mark angles as not used (to be filled later)
	for (x = 0; x < 256; x++)
		cAngle[x] = -500;

	int xOffset, yOffset;
	int xPos = 0, yPos = 0, gPos;
	unsigned char *dPos = Layer + dVal + ch, *bPos = Layer + bVal, xTS_2Pow = get2Pow(xTS);
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			xOffset = cAngle[*dPos];

			if (xOffset < -128)
			{
				int val = *dPos;
				Angle = (val*1.41176470588f+90) * PI_180;
				xOffset = cAngle[val] = int(127 * cosf(Angle));
				yOffset = sAngle[val] = int(127 * sinf(Angle));
			}
			else
			{
				yOffset = sAngle[*dPos];
			}
			dPos += 4;

			cR = 0;
			cG = 0;
			cB = 0;

			xPos = x << 7, yPos = y << 7;
			for (i = 0; i < dist; i++)
			{
				xPos += xOffset, yPos += yOffset;
				gPos = (((xPos>>7)&(xTS-1))+(((yPos>>7)&(yTS-1))<<xTS_2Pow))<<2;

				cR += *(temp+gPos  );
				cG += *(temp+gPos+1);
				cB += *(temp+gPos+2);
			}

			// Optimize this by doing the (* (1024 * dist) >> 10) trick
			// NOTE: tried that, appeared to be slower
			cR /= dist;
			cG /= dist;
			cB /= dist;

			*bPos++ = cR;
			*bPos++ = cG;
			*bPos++ = cB;
			bPos++;
		}
	}
}

void TexGen::Blur(int gL, int times)
{
	int i, x, y;
	int bVal = (gL * xTS * yTS) << 2, yAdd;

	int y1, y2, x1, x2;
	int xTS_2Pow = get2Pow(xTS);

	for (i = 0; i < times; i++)
	{
		yAdd = 0;
		for (y = 0; y < yTS; y++)
		{
			y1 = ((y - 1)&(yTS-1))<<xTS_2Pow, y2 = ((y + 1)&(yTS-1))<<xTS_2Pow;

			for (x = 0; x < xTS; x++)
			{
				x1 = (x - 1)&(xTS-1), x2 = (x + 1)&(xTS-1);

				Layer[bVal+((x+yAdd)<<2)  ] = (Layer[bVal+((x1+yAdd)<<2)  ] +
											   Layer[bVal+((x2+yAdd)<<2)  ] +
											   Layer[bVal+((x +  y1)<<2)  ] +
											   Layer[bVal+((x +  y2)<<2)  ]) >> 2;
				Layer[bVal+((x+yAdd)<<2)+1] = (Layer[bVal+((x1+yAdd)<<2)+1] +
											   Layer[bVal+((x2+yAdd)<<2)+1] +
											   Layer[bVal+((x +  y1)<<2)+1] +
											   Layer[bVal+((x +  y2)<<2)+1]) >> 2;
				Layer[bVal+((x+yAdd)<<2)+2] = (Layer[bVal+((x1+yAdd)<<2)+2] +
											   Layer[bVal+((x2+yAdd)<<2)+2] +
											   Layer[bVal+((x +  y1)<<2)+2] +
											   Layer[bVal+((x +  y2)<<2)+2]) >> 2;
			}
			yAdd += xTS;
		}
	}	
}

void TexGen::BlurGaussV(unsigned char layer, int halfRad, unsigned char curveSlope, unsigned char brighten, float * extCoeffs)
{
	float * coeffs, intCoeffs[256];
	if (extCoeffs == 0)
	{
		coeffs = intCoeffs;
		CalcGaussianWeights(coeffs, (float)curveSlope, halfRad);
	}
	else
	{
		coeffs = extCoeffs;
	}
	int iCoeffs[256];
	for (int i = 0; i <= halfRad; ++i)
	{
		iCoeffs[i] = (int)(255 * coeffs[i]);
	}

	int bVal = (layer * xTS * yTS) << 2, yAdd;
	unsigned char * curLayer = Layer + bVal;

	unsigned char* temp = mTempLayer;
	for (int x = 0; x < xTS*yTS << 2; x++) temp[x] = curLayer[x];

	int xTS_2Pow = get2Pow(xTS);

	yAdd = 0;
	for (int y = 0; y < yTS; y++)
	{
		for (int x = 0; x < xTS; x++)
		{
			int cR = iCoeffs[0] * temp[((x+yAdd)<<2)  ];
			int cG = iCoeffs[0] * temp[((x+yAdd)<<2)+1];
			int cB = iCoeffs[0] * temp[((x+yAdd)<<2)+2];

			for (int i = 1; i <= halfRad; ++i)
			{
				int ylAdd = (( (y - i)&(yTS-1) ) << xTS_2Pow);
				int yrAdd = (( (y + i)&(yTS-1) ) << xTS_2Pow);

				cR += iCoeffs[i] * temp[((x+ylAdd)<<2)  ];
				cG += iCoeffs[i] * temp[((x+ylAdd)<<2)+1];
				cB += iCoeffs[i] * temp[((x+ylAdd)<<2)+2];

				cR += iCoeffs[i] * temp[((x+yrAdd)<<2)  ];
				cG += iCoeffs[i] * temp[((x+yrAdd)<<2)+1];
				cB += iCoeffs[i] * temp[((x+yrAdd)<<2)+2];
			}
			curLayer[((x+yAdd)<<2)  ] = (unsigned char)clamp<int>(int(cR * brighten) >> (4 + 8), 0, 255);
			curLayer[((x+yAdd)<<2)+1] = (unsigned char)clamp<int>(int(cG * brighten) >> (4 + 8), 0, 255);
			curLayer[((x+yAdd)<<2)+2] = (unsigned char)clamp<int>(int(cB * brighten) >> (4 + 8), 0, 255);
		}
		yAdd += xTS;
	}
}

void TexGen::BlurGaussH(unsigned char layer, int halfRad, unsigned char curveSlope, unsigned char brighten, float * extCoeffs)
{
	float * coeffs, intCoeffs[256];
	if (extCoeffs == 0)
	{
		coeffs = intCoeffs;
		CalcGaussianWeights(coeffs, (float)curveSlope, halfRad);
	}
	else
	{
		coeffs = extCoeffs;
	}

	int iCoeffs[256];
	for (int i = 0; i <= halfRad; ++i)
	{
		iCoeffs[i] = (int)(255 * coeffs[i]);
	}

	int bVal = (layer * xTS * yTS) << 2, yAdd;
	unsigned char * curLayer = Layer + bVal;

	unsigned char* temp = mTempLayer;
	for (int x = 0; x < xTS*yTS << 2; x++) temp[x] = curLayer[x];

	yAdd = 0;
	for (int y = 0; y < yTS; y++)
	{
		for (int x = 0; x < xTS; x++)
		{
			int cR = iCoeffs[0] * temp[((x+yAdd)<<2)  ];
			int cG = iCoeffs[0] * temp[((x+yAdd)<<2)+1];
			int cB = iCoeffs[0] * temp[((x+yAdd)<<2)+2];

			for (int i = 1; i <= halfRad; ++i)
			{
				int xl = ( (x - i)&(xTS-1) );
				int xr = ( (x + i)&(xTS-1) );

				cR += iCoeffs[i] * temp[((xl+yAdd)<<2)  ];
				cG += iCoeffs[i] * temp[((xl+yAdd)<<2)+1];
				cB += iCoeffs[i] * temp[((xl+yAdd)<<2)+2];

				cR += iCoeffs[i] * temp[((xr+yAdd)<<2)  ];
				cG += iCoeffs[i] * temp[((xr+yAdd)<<2)+1];
				cB += iCoeffs[i] * temp[((xr+yAdd)<<2)+2];
			}
			curLayer[((x+yAdd)<<2)  ] = (unsigned char)clamp<int>(int(cR * brighten) >> (4 + 8), 0, 255);
			curLayer[((x+yAdd)<<2)+1] = (unsigned char)clamp<int>(int(cG * brighten) >> (4 + 8), 0, 255);
			curLayer[((x+yAdd)<<2)+2] = (unsigned char)clamp<int>(int(cB * brighten) >> (4 + 8), 0, 255);
		}
		yAdd += xTS;
	}
}

void TexGen::BlurGauss(unsigned char layer, int halfRad, unsigned char curveSlope, unsigned char brighten, float * extCoeffs)
{
	float * coeffs, intCoeffs[256];
	if (extCoeffs == 0)
	{
		coeffs = intCoeffs;
		CalcGaussianWeights(coeffs, (float)curveSlope, halfRad);
	}
	else
	{
		coeffs = extCoeffs;
	}

	BlurGaussH(layer, halfRad, curveSlope, brighten, coeffs);
	BlurGaussV(layer, halfRad, curveSlope, brighten, coeffs);
}

void TexGen::SmartBlur(unsigned char bParams, unsigned char MaxDiff)
{
	int i, x, y, yAdd, xPos, yPos, x1, y1, cR, cG, cB;
	int bVal = (bParams >> 4) * xTS * yTS << 2;

	for (i = 0; i < (bParams & 0xF); i++)
	{
		yAdd = 0;
		for (y = 0; y < yTS; y++)
		{
			for (x = 0; x < xTS; x++)
			{
				xPos = (x + 1)&(xTS-1);
				yPos = (y + 1)&(yTS-1);

				float AvColor = Layer[bVal+((x   + yAdd)<<2)]*0.23f + Layer[bVal+((x   + yAdd)<<2)+1]*0.59f + Layer[bVal+((x   + yAdd)<<2)+2]*0.18f;
				float AvXPos  = Layer[bVal+((xPos+ yAdd)<<2)]*0.23f + Layer[bVal+((xPos+ yAdd)<<2)+1]*0.59f + Layer[bVal+((xPos+ yAdd)<<2)+2]*0.18f;
				float AvYPos  = Layer[bVal+((x+yPos*xTS)<<2)]*0.23f + Layer[bVal+((x+yPos*xTS)<<2)+1]*0.59f + Layer[bVal+((x+yPos*xTS)<<2)+2]*0.18f;

				if ((fabsf(AvColor - AvXPos) > MaxDiff) ||
					(fabsf(AvColor - AvYPos) > MaxDiff))
				{
					x1 = (x - 1)&(xTS-1);
					y1 = (y - 1)&(yTS-1);

					cR = (Layer[bVal+((x1  + yAdd)<<2)  ] +
						  Layer[bVal+((xPos+ yAdd)<<2)  ] +
						  Layer[bVal+((x + y1*xTS)<<2)  ] +
						  Layer[bVal+((x+yPos*xTS)<<2)  ]) >> 2;
					cG = (Layer[bVal+((x1  + yAdd)<<2)+1] +
						  Layer[bVal+((xPos+ yAdd)<<2)+1] +
						  Layer[bVal+((x + y1*xTS)<<2)+1] +
						  Layer[bVal+((x+yPos*xTS)<<2)+1]) >> 2;
					cB = (Layer[bVal+((x1  + yAdd)<<2)+2] +
						  Layer[bVal+((xPos+ yAdd)<<2)+2] +
						  Layer[bVal+((x + y1*xTS)<<2)+2] +
						  Layer[bVal+((x+yPos*xTS)<<2)+2]) >> 2;

					Layer[bVal+((x+yAdd)<<2)  ] = cR;
					Layer[bVal+((x+yAdd)<<2)+1] = cG;
					Layer[bVal+((x+yAdd)<<2)+2] = cB;
				}
			}
			yAdd += xTS;
		}
	}
}

void TexGen::SineDist(int gL, char xK, char yK, int xAmp, int yAmp)
{
	unsigned char* temp = mTempLayer;

	int bVal = (gL * xTS * yTS)<<2;
	int x, y;

	float xZ = (xK * 2 * PI) / (float)xTS;
	float yZ = (yK * 2 * PI) / (float)yTS;

	float * distX = (float *)malloc(yTS * sizeof(float));
	float * distY = (float *)malloc(xTS * sizeof(float));
	for (x = 0; x < xTS; x++) distY[x] = yAmp * sinf(x * yZ);
	for (y = 0; y < yTS; y++) distX[y] = xAmp * sinf(y * xZ);

	unsigned char *lPos = Layer + bVal, *tPos = temp;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			GetBerpTexel_RGB(lPos, distX[y] + x, distY[x] + y, tPos, tPos+1, tPos+2);
			tPos += 4;
		}
	} tPos = temp;
	for (x = 0; x < (xTS*yTS)<<2; x++) *lPos++ = *tPos++;

	free(distX);
	free(distY);
}

void TexGen::SineDistW(int gL, int Times, int AmpXi, int AmpYi, unsigned char Wrap)
{
	float AmpX = AmpXi / 25600.0f * xTS;
	float AmpY = AmpYi / 25600.0f * yTS;
	int x, y, bVal = gL * xTS * yTS << 2;

	unsigned char* temp = mTempLayer;

	float	deltaX = 2 * PI / (float)xTS * (Wrap >>  4),
			deltaY = 2 * PI / (float)yTS * (Wrap & 0xF);

	float * distX = (float *)malloc(yTS * sizeof(float));
	float * distY = (float *)malloc(xTS * sizeof(float));
	for (y = 0; y < yTS; y++) distX[y] = -AmpX * sinf(y * deltaX);
	for (x = 0; x < xTS; x++) distY[x] = -AmpY * sinf(x * deltaY);//*/
	float distAdd;

	unsigned char *lPos = Layer + bVal, *tPos;
	for (int j = 0; j < Times; j++)
	{
		tPos = temp;
		for (y = 0; y < yTS; y++)
		{
			distAdd = distX[y];
			for (x = 0; x < xTS; x++)
			{
				GetBerpTexel_RGB(lPos, x + distAdd, (float)y, tPos, tPos + 1, tPos + 2);
				tPos += 4;
			}
		} memcpy(Layer + bVal, temp, xTS*yTS<<2);
    
		tPos = temp; distAdd = 0.0f;
		for (x = 0; x < xTS; x++)
		{
			distAdd += distY[x];
			for (y = 0; y < yTS; y++)
			{
				GetBerpTexel_RGB(lPos, (float)x, y + distAdd, tPos, tPos + 1, tPos + 2);
				tPos += 4;
			}
		} memcpy(Layer + bVal, temp, xTS*yTS<<2);
	}

	free(distX);
	free(distY);
}

void TexGen::Turbulence(unsigned char gL, unsigned char xParam, unsigned char yParam, unsigned int Scale, unsigned int Turb)
{
	int bVal = (gL * xTS * yTS)<<2;
	int cVal[256], sVal[256];
	int xOffset, yOffset;
	int x, y;

	int xScale = Scale >> 8, yScale = Scale & 0xFF;
	int xTurb  = Turb  >> 8, yTurb  = Turb  & 0xFF;

	for (int i = 0; i < 256; i++)
	{
		sVal[i] = int(-xScale * sinf(i / float(xTurb)));
		cVal[i] = int( yScale * cosf(i / float(yTurb)));
	}

	unsigned char *temp = mTempLayer;
	for (int i = 0; i < xTS*yTS<<2; i++) temp[i] = Layer[bVal+i];

	unsigned char *bPos = Layer + bVal, *xPos = Layer + (((xParam>>4)*xTS*yTS)<<2)+(xParam&0xF), *yPos = Layer + (((yParam>>4)*xTS*yTS)<<2)+(yParam&0xF);
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			xOffset =  (x + sVal[*xPos])&(xTS-1);
			yOffset = ((y + cVal[*yPos])&(yTS-1))*xTS;
			xPos += 4; yPos += 4;

			*bPos++ = temp[((xOffset+yOffset)<<2)  ];
			*bPos++ = temp[((xOffset+yOffset)<<2)+1];
			*bPos++ = temp[((xOffset+yOffset)<<2)+2];
			bPos++;
		}
	}
}

void TexGen::MapDist(unsigned char dL, unsigned char s_xParam, unsigned char s_yParam, short xK, short yK, int xChOffset, int yChOffset)
{
	unsigned char* temp = mTempLayer;

	int bVal = (dL * xTS * yTS)<<2;
	int xVal = ((s_xParam >> 4) * xTS * yTS)<<2, x_ch = s_xParam & 0xF;
	int yVal = ((s_yParam >> 4) * xTS * yTS)<<2, y_ch = s_yParam & 0xF;
	int x, y, yAdd = 0;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			int xOffset = (((  (Layer[xVal+((x+yAdd)<<2)+x_ch] - xChOffset) * xK) >> 8) + x) & (xTS - 1);
			int yOffset = (((( (Layer[yVal+((x+yAdd)<<2)+y_ch] - yChOffset) * yK) >> 8) + y) & (yTS - 1))*xTS;
			
			temp[((x+yAdd)<<2)  ] = Layer[bVal+((xOffset+yOffset)<<2)  ];
			temp[((x+yAdd)<<2)+1] = Layer[bVal+((xOffset+yOffset)<<2)+1];
			temp[((x+yAdd)<<2)+2] = Layer[bVal+((xOffset+yOffset)<<2)+2];
		}
		yAdd += xTS;
	}
	for (x = 0; x < (xTS*yTS)<<2; x++) Layer[bVal+x] = temp[x];
}

void TexGen::Offset(char gL, short xOffset, short yOffset)
{
	unsigned char* temp = mTempLayer;

	int bVal = gL * xTS * yTS << 2;
	int x, y, xPos, yPos;

	int xTS_2Pow = get2Pow(xTS);

	int xDif = xOffset;
	int yDif = yOffset;

	unsigned char *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		yPos = ((y+yDif) & (yTS-1))<<xTS_2Pow;
		for (x = 0; x < xTS; x++)
		{
			xPos = (((x+xDif)&(xTS-1))+yPos)<<2;

			*(temp+xPos  ) = *lPos++;
			*(temp+xPos+1) = *lPos++;
			*(temp+xPos+2) = *lPos++;
			lPos++;
		}
	}

	for (x = 0; x < xTS*yTS<<2; x++) Layer[bVal+x] = temp[x];
}

void TexGen::Twirl(int gL, int whirlx, int whirly, int radius, unsigned char Whirls, char CosAng)
{
	if (Whirls == 128) return;

	int bVal = gL * xTS * yTS << 2;
	int x, y, yCent;
	unsigned char *temp = mTempLayer;

	__int16 YtoC, XtoC;
	char whirls = Whirls - 128;
	int yAdd = 0;

	float whirlsMul;
	if (CosAng)	whirlsMul = whirls*PI/42.6667f;
	else		whirlsMul = whirls*PI/16.0000f;
	float cAng, sAng, Dist;

	unsigned char *lPos = Layer + bVal, *tPos = temp;
	for (y = 0; y < yTS; y++)
	{
		yCent = sqr(y-whirly);
		YtoC = y-whirly;
		for (x = 0; x < xTS; x++)
		{
			Dist = sqrtf(float(sqr(x-whirlx) + yCent));
			if (Dist >= radius)
			{
				*(tPos  ) = *(lPos+((x+yAdd)<<2)  );
				*(tPos+1) = *(lPos+((x+yAdd)<<2)+1);
				*(tPos+2) = *(lPos+((x+yAdd)<<2)+2);
			} else
			{
				XtoC = x - whirlx;
				if (CosAng) Dist = ((0.8f+0.2f*cosf(whirlsMul*(radius-Dist)/(float)radius))*Dist);
				cAng = whirlsMul*sqr((radius-Dist)/(float)radius);

#if 0
				__asm
				{
					FLD cAng
					FSIN
					FSTP sAng
					FLD cAng
					FCOS
					FSTP cAng
				}
#else
				sAng = sinf(cAng);
				cAng = cosf(cAng);
#endif

				GetBerpTexel_RGB(lPos, (XtoC*cAng)-(YtoC*sAng)+whirlx, (XtoC*sAng)+(YtoC*cAng)+whirly, tPos, tPos + 1, tPos + 2);
			}
			tPos += 4;
		}
		yAdd += xTS;
	}

	tPos = temp;
	for (int i = 0; i < xTS*yTS<<2; i++)
		*lPos++ = *tPos++;
}

/* Color Filters */
void TexGen::NMapSobel(unsigned char dstLayer, unsigned char srcLParams, bool isInverted)
{
	int bVal = (srcLParams >>  4) * xTS * yTS << 2, ch = srcLParams & 0xF;
	int tVal = dstLayer * xTS * yTS << 2;

	int x, y;
	int Xp1, Yp1, Xm1, Ym1;
	float dX, dY;

	int yAdd, yAdd_p1, yAdd_m1;

	float nx, ny, nz;

	unsigned char *temp = mTempLayer;
	for (int i = 0, iend = xTS*yTS; i < iend; ++i)
		temp[i] = Layer[bVal+(i<<2)+ch];

	for (y = 0; y < yTS; y++)
	{
		Yp1 = (y+1) & (yTS-1);
		Ym1 = (y-1) & (yTS-1);

		yAdd	=  y *xTS;
		yAdd_p1	= Yp1*xTS;
		yAdd_m1	= Ym1*xTS;

		for (x = 0; x < xTS; x++)
		{
			Xp1 = (x+1) & (xTS-1);
			Xm1 = (x-1) & (xTS-1);

			dY  = temp[Xm1+yAdd_p1] / 255.0f * (-1.0f);
			dY += temp[ x +yAdd_p1] / 255.0f * (-2.0f);
			dY += temp[Xp1+yAdd_p1] / 255.0f * (-1.0f);

			dY += temp[Xm1+yAdd_m1] / 255.0f * ( 1.0f);
			dY += temp[ x +yAdd_m1] / 255.0f * ( 2.0f);
			dY += temp[Xp1+yAdd_m1] / 255.0f * ( 1.0f);

			dX  = temp[Xm1+yAdd_m1] / 255.0f * (-1.0f);
			dX += temp[Xm1+  yAdd ] / 255.0f * (-2.0f);
			dX += temp[Xm1+yAdd_p1] / 255.0f * (-1.0f);

			dX += temp[Xp1+yAdd_m1] / 255.0f * ( 1.0f);
			dX += temp[Xp1+  yAdd ] / 255.0f * ( 2.0f);
			dX += temp[Xp1+yAdd_p1] / 255.0f * ( 1.0f);

			nx = -dX;
			ny = -dY;
			nz = 1.0f;
	
			if (isInverted)
			{
				nx = -nx;
				ny = -ny;
			}

			float oLen = 1.0f / sqrtf(nx*nx + ny*ny + nz*nz);
			nx *= oLen; ny *= oLen; nz *= oLen;

			Layer[tVal+((x+yAdd)<<2)  ] = (unsigned char)(nx * 127) + 128;
			Layer[tVal+((x+yAdd)<<2)+1] = (unsigned char)(ny * 127) + 128;
			Layer[tVal+((x+yAdd)<<2)+2] = (unsigned char)(nz * 127) + 128;
		}
	}
}

void TexGen::DUDVSobel(unsigned char dstLayer, unsigned char srcLParams)
{
	int bVal = (srcLParams >>  4) * xTS * yTS << 2, ch = srcLParams & 0xF;
	int tVal = dstLayer * xTS * yTS << 2;

	int x, y;
	int Xp1, Yp1, Xm1, Ym1;
	float dX, dY;

	int yAdd, yAdd_p1, yAdd_m1;

	float nx, ny;

	unsigned char *temp = mTempLayer;
	for (int i = 0, iend = xTS*yTS; i < iend; ++i)
		temp[i] = Layer[bVal+(i<<2)+ch];

	for (y = 0; y < yTS; y++)
	{
		Yp1 = (y+1) & (yTS-1);
		Ym1 = (y-1) & (yTS-1);

		yAdd	=  y *xTS;
		yAdd_p1	= Yp1*xTS;
		yAdd_m1	= Ym1*xTS;

		for (x = 0; x < xTS; x++)
		{
			Xp1 = (x+1) & (xTS-1);
			Xm1 = (x-1) & (xTS-1);

			dY  = temp[Xm1+yAdd_p1] / 255.0f * (-1.0f);
			dY += temp[ x +yAdd_p1] / 255.0f * (-2.0f);
			dY += temp[Xp1+yAdd_p1] / 255.0f * (-1.0f);

			dY += temp[Xm1+yAdd_m1] / 255.0f * ( 1.0f);
			dY += temp[ x +yAdd_m1] / 255.0f * ( 2.0f);
			dY += temp[Xp1+yAdd_m1] / 255.0f * ( 1.0f);

			dX  = temp[Xm1+yAdd_m1] / 255.0f * (-1.0f);
			dX += temp[Xm1+  yAdd ] / 255.0f * (-2.0f);
			dX += temp[Xm1+yAdd_p1] / 255.0f * (-1.0f);

			dX += temp[Xp1+yAdd_m1] / 255.0f * ( 1.0f);
			dX += temp[Xp1+  yAdd ] / 255.0f * ( 2.0f);
			dX += temp[Xp1+yAdd_p1] / 255.0f * ( 1.0f);

			nx = -dX;
			ny = -dY;

			float oLen = 1.0f / sqrtf(nx*nx + ny*ny + 1);
			nx *= oLen; ny *= oLen;

			Layer[tVal+((x+y*xTS)<<2)  ] = unsigned char(nx * 127);
			Layer[tVal+((x+y*xTS)<<2)+1] = unsigned char(ny * 127);
		}
	}
}

void TexGen::FindEdges(int gL)
{
	int bVal = gL * xTS * yTS * 4;
	int x, y;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			int xPos1 = x + 1, yPos1 = y + 1;
			xPos1 = xPos1&(xTS-1); yPos1 = yPos1&(yTS-1);
			if ((Layer[bVal+(x+y*xTS)*4+0] == Layer[bVal+(xPos1+y*xTS)*4+0]) &&
				(Layer[bVal+(x+y*xTS)*4+1] == Layer[bVal+(xPos1+y*xTS)*4+1]) &&
				(Layer[bVal+(x+y*xTS)*4+2] == Layer[bVal+(xPos1+y*xTS)*4+2]) &&
				(Layer[bVal+(x+y*xTS)*4+0] == Layer[bVal+(x+yPos1*xTS)*4+0]) &&
				(Layer[bVal+(x+y*xTS)*4+1] == Layer[bVal+(x+yPos1*xTS)*4+1]) &&
				(Layer[bVal+(x+y*xTS)*4+2] == Layer[bVal+(x+yPos1*xTS)*4+2]))
			{
				Layer[bVal+(x+y*xTS)*4+0] = 0;
				Layer[bVal+(x+y*xTS)*4+1] = 0;
				Layer[bVal+(x+y*xTS)*4+2] = 0;
			}
		}
	}
}

void TexGen::Outline(unsigned char lParam, unsigned char Amp)
{
	int bVal = (lParam>>4) * xTS * yTS << 2;
	int x, y, ch = lParam & 0xF;

	int xPos, yPos;
	int ind0, ind1, ind2;

	int cAmp = sqr(Amp);

	for (y = 0; y < yTS; y++)
	{
		yPos = ((y+1)&(yTS-1))*xTS;
		for (x = 0; x < xTS; x++)
		{
			xPos = (x+1)&(xTS-1);

			ind0 = bVal+((   x+y*xTS)<<2);
			ind1 = bVal+((xPos+y*xTS)<<2);
			ind2 = bVal+((   x+ yPos)<<2);

			if ((sqr(Layer[ind0+ch]-Layer[ind1+ch]) >= cAmp) ||
				(sqr(Layer[ind0+ch]-Layer[ind2+ch]) >= cAmp))
			{
				Layer[ind0  ] = 255;
				Layer[ind0+1] = 255;
				Layer[ind0+2] = 255;
			} else
			{
				Layer[ind0  ] = 0;
				Layer[ind0+1] = 0;
				Layer[ind0+2] = 0;
			}
		}
	}
}

void TexGen::Wood(int gL, unsigned char mK)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y, yAdd = 0;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			Layer[bVal+((x+yAdd)<<2)  ] = ((Layer[bVal+((x+yAdd)<<2)  ] * mK) >> 3) & 255;
			Layer[bVal+((x+yAdd)<<2)+1] = ((Layer[bVal+((x+yAdd)<<2)+1] * mK) >> 3) & 255;
			Layer[bVal+((x+yAdd)<<2)+2] = ((Layer[bVal+((x+yAdd)<<2)+2] * mK) >> 3) & 255;
		}
		yAdd += xTS;
	}
}

void TexGen::ScaleColor(unsigned char gL, int rK, int gK, int bK)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y;

	unsigned char *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			*(lPos  ) = clamp<int>((rK**(lPos  )) >> 6, 0, 255);
			*(lPos+1) = clamp<int>((gK**(lPos+1)) >> 6, 0, 255);
			*(lPos+2) = clamp<int>((bK**(lPos+2)) >> 6, 0, 255);
			lPos += 4;
		}
	}
}

void TexGen::Pixelize(int gL, int PStep)
{
	if (PStep == 0) return;

	unsigned char* temp = mTempLayer;

	int bVal = gL * xTS * yTS << 2;
	int x, y, i, j;
	for (x = 0; x < xTS * yTS << 2; x++) temp[x] = Layer[bVal+x];

	int cR, cG, cB;
	int xPos, yPos;

	for (y = 0; y < yTS; y += PStep)
	{
		for (x = 0; x < xTS; x += PStep)
		{
			cR = cG = cB = 0;
			for (j = 0; j < PStep; j++)
			{
				for (i = 0; i < PStep; i++)
				{
					xPos = (x + i)&(xTS-1);
					yPos = (y + j)&(yTS-1);
					
					cR += temp[((xPos+yPos*xTS)<<2)  ];
					cG += temp[((xPos+yPos*xTS)<<2)+1];
					cB += temp[((xPos+yPos*xTS)<<2)+2];
				}
			}
			cR /= PStep*PStep;
			cG /= PStep*PStep;
			cB /= PStep*PStep;

			for (j = 0; j < PStep; j++)
			{
				for (i = 0; i < PStep; i++)
				{
					xPos = (x + i)&(xTS-1);
					yPos = (y + j)&(yTS-1);
					
					Layer[bVal+((xPos+yPos*xTS)<<2)  ] = cR;
					Layer[bVal+((xPos+yPos*xTS)<<2)+1] = cG;
					Layer[bVal+((xPos+yPos*xTS)<<2)+2] = cB;
				}
			}
		}
	}
}

void TexGen::Emboss(char gL, unsigned char Strength)
{
	unsigned char* temp = mTempLayer;

	int bVal = (gL * xTS * yTS)<<2;
	int x, y, yAdd = 0;
	for (x = 0; x < xTS*yTS<<2; x++) temp[x] = Layer[bVal+x];
	char xTS_2Pow = get2Pow(xTS);

	int y1, y2, x1, x2;
	int clR, clG, clB;

	int ind[8];

	unsigned char *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		y1 = ((y-1)&(yTS-1))<<xTS_2Pow; y2 = ((y+1)&(yTS-1))<<xTS_2Pow;
		for (x = 0; x < xTS; x++)
		{
			x1 = (x-1)&(xTS-1); x2 = (x+1)&(xTS-1);

			*(ind  ) = (x +	y2 )<<2;
			*(ind+1) = (x2+yAdd)<<2;
			*(ind+2) = (x1+yAdd)<<2;
			*(ind+3) = (x1+	y1 )<<2;
			*(ind+4) = (x1+	y2 )<<2;
			*(ind+5) = (x +	y1 )<<2;
			*(ind+6) = (x2+	y1 )<<2;
			*(ind+7) = (x2+	y2 )<<2;

			clR = ((temp[*(ind  )  ] +
					temp[*(ind+1)  ] -
					temp[*(ind+2)  ] -
					temp[*(ind+3)  ] -
					temp[*(ind+4)  ] -
					temp[*(ind+5)  ] +
					temp[*(ind+6)  ] +
					temp[*(ind+7)  ]) * Strength >> 5) + 128;
			clG = ((temp[*(ind  )+1] +
					temp[*(ind+1)+1] -
					temp[*(ind+2)+1] -
					temp[*(ind+3)+1] -
					temp[*(ind+4)+1] -
					temp[*(ind+5)+1] +
					temp[*(ind+6)+1] +
					temp[*(ind+7)+1]) * Strength >> 5) + 128;
			clB = ((temp[*(ind  )+2] +
					temp[*(ind+1)+2] -
					temp[*(ind+2)+2] -
					temp[*(ind+3)+2] -
					temp[*(ind+4)+2] -
					temp[*(ind+5)+2] +
					temp[*(ind+6)+2] +
					temp[*(ind+7)+2]) * Strength >> 5) + 128;


			*(lPos  ) = clamp<int>(clR, 0, 255);
			*(lPos+1) = clamp<int>(clG, 0, 255);
			*(lPos+2) = clamp<int>(clB, 0, 255);
			lPos += 4;
		}
		yAdd += xTS;
	}
}

void TexGen::Diff(unsigned char src_xLC, unsigned char Strength)
{
	unsigned char* temp = mTempLayer;

	int bVal = ((src_xLC >> 4) * xTS * yTS)<<2, ch = src_xLC & 0xF;
	int x, y, yAdd = 0;
	for (x = 0; x < xTS*yTS; x++) temp[x] = Layer[bVal+(x<<2)+ch];

	int y1, y2, x1, x2;
	int color;

	unsigned char *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		y1 = ((y-1)&(yTS-1))*xTS; y2 = ((y+1)&(yTS-1))*xTS;
		for (x = 0; x < xTS; x++)
		{
			x1 = (x-1)&(xTS-1); x2 = (x+1)&(xTS-1);

			int dX, dY;
			dX = temp[x+y2] - temp[x+y1];
			dY = temp[x2+yAdd] - temp[x1+yAdd];
			color = (int)sqrtf((float)dX*dX + (float)dY*dY);

			*(lPos+ch) = clamp<int>(color, 0, 255);
			lPos += 4;
		}
		yAdd += xTS;
	}
}

void TexGen::SineColor(unsigned char lParams, float K)
{
	int bVal = ((lParams >> 4) * xTS * yTS) << 2;
	char ch = lParams & 0xF;
	int x, y, yAdd = 0;
	float z = (K * PI) / 255.0f;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
			Layer[bVal+((x+yAdd)<<2)+ch] = int(255 * fabsf(sinf(z * Layer[bVal+((x+yAdd)<<2)+ch])));

		yAdd += xTS;
	}
}

void TexGen::SquareEdges(int gL, unsigned char r, unsigned char g, unsigned char b, int times)
{
	int bVal = gL * xTS * yTS << 2;
	int i, x, y, yAdd = 0, xTS2Pow = get2Pow(xTS);

	unsigned char cR, cG, cB;

	for (i = 0; i < times; i++)
	{
		y = 0 + i;
		yAdd = y * xTS;
		for (x = 1 + i; x < xTS - i; x++)
		{
			cR = r * (times - i) / times + Layer[bVal+((x+yAdd)<<2)  ] * i / times;
			cG = g * (times - i) / times + Layer[bVal+((x+yAdd)<<2)+1] * i / times;
			cB = b * (times - i) / times + Layer[bVal+((x+yAdd)<<2)+2] * i / times;
			Layer[bVal+((x+yAdd)<<2)  ] = cR;
			Layer[bVal+((x+yAdd)<<2)+1] = cG;
			Layer[bVal+((x+yAdd)<<2)+2] = cB;
		}
		y = yTS - i - 1;
		yAdd = y * xTS;
		for (x = 0 + i; x < (xTS+1) - i; x++)
		{
			cR = r * (times - i) / times + Layer[bVal+((x+yAdd)<<2)  ] * i / times;
			cG = g * (times - i) / times + Layer[bVal+((x+yAdd)<<2)+1] * i / times;
			cB = b * (times - i) / times + Layer[bVal+((x+yAdd)<<2)+2] * i / times;
			Layer[bVal+((x+yAdd)<<2)  ] = cR;
			Layer[bVal+((x+yAdd)<<2)+1] = cG;
			Layer[bVal+((x+yAdd)<<2)+2] = cB;
		}
		x = i;
		for (y = i; y < yTS - i; y++)
		{
			cR = r * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)  ]*i/times;
			cG = g * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)+1]*i/times;
			cB = b * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)+2]*i/times;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)  ] = cR;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)+1] = cG;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)+2] = cB;
		}
		x = xTS - i - 1;
		for (y = i; y < yTS - i; y++)
		{
			cR = r * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)  ]*i/times;
			cG = g * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)+1]*i/times;
			cB = b * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)+2]*i/times;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)  ] = cR;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)+1] = cG;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)+2] = cB;
		}
	}
}

void TexGen::SquareEdgesChannel(unsigned char lParams, unsigned char c, int times)
{
	int bVal = (lParams >> 4) * xTS * yTS << 2, bChannel = (lParams & 0xF);
	int i, x, y, yAdd = 0, xTS2Pow = get2Pow(xTS);

	unsigned char cF;

	for (i = 0; i < times; i++)
	{
		y = 0 + i;
		yAdd = y * xTS;
		for (x = 1 + i; x < xTS - i; x++)
		{
			cF = c * (times - i) / times + Layer[bVal+((x+yAdd)<<2)+bChannel] * i / times;
			Layer[bVal+((x+yAdd)<<2)+bChannel] = cF;
		}
		y = yTS - i - 1;
		yAdd = y * xTS;
		for (x = 0 + i; x < (xTS+1) - i; x++)
		{
			cF = c * (times - i) / times + Layer[bVal+((x+yAdd)<<2)  ] * i / times;
			Layer[bVal+((x+yAdd)<<2)+bChannel] = cF;
		}
		x = i;
		for (y = i; y < yTS - i; y++)
		{
			cF = c * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)  ]*i/times;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)+bChannel] = cF;
		}
		x = xTS - i - 1;
		for (y = i; y < yTS - i; y++)
		{
			cF = c * (times-i)/times + Layer[bVal+((x+(y<<xTS2Pow))<<2)  ]*i/times;
			Layer[bVal+((x+(y<<xTS2Pow))<<2)+bChannel] = cF;
		}
	}
}

void TexGen::Plastic(int gL, int times)
{
	int bVal = gL * xTS * yTS << 2;

	unsigned char *temp = mTempLayer;
	int ix, iy, x, y, j, c;

	unsigned char *tPos, *lPos;
	for (j = 0; j < times; j++)
	{
		memcpy(temp, Layer+bVal, (xTS*yTS<<2)*sizeof(unsigned char));
		tPos = temp; lPos = Layer + bVal;
		for (y = 0; y < yTS; y++)
		{
			iy = ((y+1)&(yTS-1))*xTS;
			for (x = 0; x < xTS; x++)
			{
				ix = (x + 1)&(xTS-1);
			
				c = ((ix+iy)<<2);
/*				*lPos++ = clamp(*tPos+++(m_abs<int>((*tPos)-temp[c  ])<<1), 0, 255);
				*lPos++ = clamp(*tPos+++(m_abs<int>((*tPos)-temp[c+1])<<1), 0, 255);
				*lPos++ = clamp(*tPos+++(m_abs<int>((*tPos)-temp[c+2])<<1), 0, 255);*/
				*lPos++ = clamp(*tPos+(m_abs<int>((*tPos)-temp[c  ])<<1), 0, 255); tPos++;
				*lPos++ = clamp(*tPos+(m_abs<int>((*tPos)-temp[c+1])<<1), 0, 255); tPos++;
				*lPos++ = clamp(*tPos+(m_abs<int>((*tPos)-temp[c+2])<<1), 0, 255); tPos++;

				tPos++; lPos++;
			}
		}
	}
}

void TexGen::InvertCh(unsigned char layerCh)
{
	unsigned char *lPos = Layer + ((layerCh >> 4) * xTS * yTS << 2) + (layerCh & 0xF);
	for (int i = 0; i < xTS*yTS; i++)
	{
		*lPos = ~*lPos;
		lPos += 4;
	}
}
void TexGen::Invert(int gL)
{
	unsigned char *lPos = Layer + (gL * xTS * yTS << 2);
	for (int i = 0; i < xTS*yTS<<2; i++)
		*lPos++ = ~*lPos;
}
void TexGen::Invert_RGB(int gL)
{
	unsigned char *lPos = Layer + (gL * xTS * yTS << 2);
	for (int i = 0; i < xTS*yTS; i++)
	{
		*lPos++ = ~*lPos;
		*lPos++ = ~*lPos;
		*lPos++ = ~*lPos;
		++lPos;
	}
}

void TexGen::Bias(int gL, unsigned char Val, unsigned char *Array)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y;

	unsigned char *lPos = Layer + bVal;
	unsigned char *aPos = Array + (Val << 8);
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			*lPos = *(aPos + *lPos); lPos++;
			*lPos = *(aPos + *lPos); lPos++;
			*lPos = *(aPos + *lPos); lPos++;
			lPos++;
		}
	}
}

void TexGen::Bright(int gL, unsigned char k)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y;

	unsigned char *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			*lPos = clamp<int>((k**lPos) >> 5, 0, 255); lPos++;
			*lPos = clamp<int>((k**lPos) >> 5, 0, 255); lPos++;
			*lPos = clamp<int>((k**lPos) >> 5, 0, 255); lPos++;
			lPos++;
		}
	}
}
void TexGen::Bright512(int gL, int k)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y;

	unsigned char *lPos = Layer + bVal;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			*lPos = clamp<int>((k**lPos) >> 9, 0, 255); lPos++;
			*lPos = clamp<int>((k**lPos) >> 9, 0, 255); lPos++;
			*lPos = clamp<int>((k**lPos) >> 9, 0, 255); lPos++;
			lPos++;
		}
	}
}

void TexGen::Balance(unsigned char lParam, unsigned char cBeg, unsigned char cEnd)
{
	int bVal = (lParam >> 4) * xTS * yTS * 4;
	int i, ch = lParam & 0xF;

	int cDif = cEnd - cBeg;

	unsigned char *lPos = Layer + bVal + ch;
	for (i = 0; i < xTS*yTS; i++)
	{
		*lPos = clamp<int>(((cDif**lPos)>>8)+cBeg, 0, 255);
		lPos += 4;
	}
}

void TexGen::rgb2hsv(unsigned char  r, unsigned char  g, unsigned char  b,
			 unsigned char *h, unsigned char *s, unsigned char *v)
{
	int maxR, maxG, maxB, delta;
	unsigned char mx, mn;
	mx = r; if (g > mx) mx = g; if (b > mx) mx = b;
	mn = r; if (g < mn) mn = g; if (b < mn) mn = b;

	*v = mx;
	*s = 0;

	int th;

	if (mx != 0)
		*s = clamp<int>(((mx-mn)<<8)/mx, 0, 255);
	if (*s == 0)
		*h = -1;
	else
	{
		delta= mx - mn;

		maxR = mx - r;
		maxG = mx - g;
		maxB = mx - b;

		if		(r == mx) th = ((maxB-maxG)<<8) / delta;
		else if (g == mx) th = 512 + ((maxR-maxB)<<8) / delta;
		else			  th = 1024 + ((maxG-maxR)<<8) / delta;

		th = (th * 43) >> 8;
		
		*h = th&255;
	}
}

void TexGen::hsv2rgb(unsigned char  h, unsigned char  s, unsigned char  v,
			 unsigned char *r, unsigned char *g, unsigned char *b)
{
	if (s == 0)
		*r = *g = *b = v;
	else
	{
		unsigned char p, q, t, f, fh;
		fh = h / 43;
		f = int(255*(h/43.0f-fh));
		p = v*(255-s)>>8;
		q = v*(255-(s*f>>8))>>8;
		t = v*(255-(s*(255-f)>>8))>>8;

		if		(fh == 0) { *r = v; *g = t; *b = p; }
		else if	(fh == 1) { *r = q; *g = v; *b = p; }
		else if	(fh == 2) { *r = p; *g = v; *b = t; }
		else if	(fh == 3) { *r = p; *g = q; *b = v; }
		else if	(fh == 4) { *r = t; *g = p; *b = v; }
		else if	(fh == 5) { *r = v; *g = p; *b = q; }
	}
}

void TexGen::AdjustHSV(int gL, unsigned char H, unsigned char S, unsigned char V)
{
	int i, bVal = gL * xTS * yTS << 2;
	unsigned char hc, sc, vc;
	unsigned int h, s, v;

	unsigned char *lPos = Layer + bVal;
	for (i = 0; i < xTS*yTS; i++)
	{
		int r = *(lPos+(i<<2)), g = *(lPos+(i<<2)+1), b = *(lPos+(i<<2)+2);
		int delta;
		unsigned char mx, mn;
		mx = r; if (g > mx) mx = g; if (b > mx) mx = b;
		mn = r; if (g < mn) mn = g; if (b < mn) mn = b;

		vc = mx;
		sc = 0;

		int th;

		// Convert to HSV
		if (mx != 0)
			sc = clamp<int>(((mx-mn)<<8)/mx, 0, 255);

		if (sc == 0)
			hc = -1;
		else
		{
			delta= mx - mn;

			// This is not needed
			//maxR = mx - r;
			//maxG = mx - g;
			//maxB = mx - b;
			// Ultimately, e.g. (maxB - maxG) boils down to (g - b)

			if		(r == mx) th = ((g-b)<<8) / delta;
			else if (g == mx) th = 512 + ((b-r)<<8) / delta;
			else			  th = 1024 + ((r-g)<<8) / delta;

			th = (th * 43) >> 8;
			
			hc = th&255;
		}
		
		// Adjust HSV
		h = (hc + H)&255;
		s = (sc * S) >> 8;
		if (s > 255) s = 255;
		v = (vc * V) >> 8;
		if (v > 255) v = 255;

		// Convert back to RGB
		unsigned char *rp = lPos+(i<<2), *gp = lPos+(i<<2)+1, *bp = lPos+(i<<2)+2;
		if (s == 0)
			*rp = *gp = *bp = v;
		else
		{
			unsigned char p, q, t, f, fh;
			fh = h / 43;
			f = int(255*(h/43.0f-fh));
			p = v*(255-s)>>8;
			q = v*(255-(s*f>>8))>>8;
			t = v*(255-(s*(255-f)>>8))>>8;

			if		(fh == 0) { *rp = v; *gp = t; *bp = p; }
			else if	(fh == 1) { *rp = q; *gp = v; *bp = p; }
			else if	(fh == 2) { *rp = p; *gp = v; *bp = t; }
			else if	(fh == 3) { *rp = p; *gp = q; *bp = v; }
			else if	(fh == 4) { *rp = t; *gp = p; *bp = v; }
			else if	(fh == 5) { *rp = v; *gp = p; *bp = q; }
		}
	}
}

void TexGen::NormalizeCh(unsigned char lParams, int minVal, int maxVal)
{
	int bVal = (lParams >> 4) * xTS * yTS << 2;
	unsigned char ch = lParams & 0xF;
	int x, y, yAdd = 0;

	int min = 255, max = 0;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			unsigned char val = Layer[bVal+((x+yAdd)<<2)+ch];
			if (min > val)
				min = val;
			if (max < val)
				max = val;
		}
		yAdd += xTS;
	}

	float mul = (maxVal - minVal) / (float)(max - min);

	yAdd = 0;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			int val = (int)((Layer[bVal+((x+yAdd)<<2)+ch] - min) * mul) + minVal;
			Layer[bVal+((x+yAdd)<<2)+ch] = clamp<int>(val, 0, 255);
		}
		yAdd += xTS;
	}
}

void TexGen::Colorize(int gL, unsigned char r, unsigned char g, unsigned char b)
{
	int bVal = gL * xTS * yTS << 2;
	int x, y, yAdd = 0;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			Layer[bVal+((x+yAdd)<<2)  ] = clamp<int>((Layer[bVal+((x+yAdd)<<2)  ] * r) >> 8, 0, 255);
			Layer[bVal+((x+yAdd)<<2)+1] = clamp<int>((Layer[bVal+((x+yAdd)<<2)+1] * g) >> 8, 0, 255);
			Layer[bVal+((x+yAdd)<<2)+2] = clamp<int>((Layer[bVal+((x+yAdd)<<2)+2] * b) >> 8, 0, 255);
		}
		yAdd += xTS;
	}
}

void TexGen::ColorizeBW(unsigned char lParam, unsigned char bR, unsigned char bG, unsigned char bB,
									  unsigned char eR, unsigned char eG, unsigned char eB)
{
	int i, bVal = (lParam >> 4) * xTS * yTS << 2;

	int rDif = eR - bR, gDif = eG - bG, bDif = eB - bB;
	unsigned char *lPos = Layer + bVal + (lParam & 0xF);
	unsigned char *bPos = Layer + bVal, iC;
	for (i = 0; i < xTS*yTS; i++)
	{
		iC = *lPos;
		*bPos++ = clamp<int>((((rDif*iC)>>8)+bR), 0, 255);
		*bPos++ = clamp<int>((((gDif*iC)>>8)+bG), 0, 255);
		*bPos++ = clamp<int>((((bDif*iC)>>8)+bB), 0, 255);
		lPos += 4; bPos++;
	}
}

void TexGen::Contrast(int gL, unsigned short c, unsigned short k)
{
	int i, bVal = gL * xTS * yTS << 2;

	c = 255 - c;
	int cBeg = 128 - (k << 3);
	int cDif = (k << 4);

	unsigned char *lPos = Layer + bVal;
	for (i = 0; i < xTS*yTS; i++)
	{
		*lPos++ = clamp<int>((cDif*((c**lPos)>>7)>>8)+cBeg, 0, 255);
		*lPos++ = clamp<int>((cDif*((c**lPos)>>7)>>8)+cBeg, 0, 255);
		*lPos++ = clamp<int>((cDif*((c**lPos)>>7)>>8)+cBeg, 0, 255);
		lPos++;
	}
}

void TexGen::DelChannel(char gL, char Chnl)
{
	int x, y;
	unsigned char *lPos = Layer + (gL * xTS * yTS << 2) + Chnl;

	for (y = 0; y < yTS; y++)
		for (x = 0; x < xTS; x++)
		{
			*lPos = 0;
			lPos += 4;
		}
}

void TexGen::MkChannel(unsigned char lParams)
{
	int bVal = ((lParams >> 4) * xTS * yTS) << 2;
	int i, t_col;
	int ch = lParams & 0xF;

	unsigned char *lPos = Layer + bVal;
	for (i = 0; i < xTS*yTS; i++)
	{
		t_col = *(lPos + ch);

		*(lPos  ) = t_col;
		*(lPos+1) = t_col;
		*(lPos+2) = t_col;

		lPos+=4;
	}
}

/* Layer Operations */
void TexGen::FlipH(int gL)
{
	unsigned char* temp = mTempLayer;
	int bVal = gL * xTS * yTS << 2;
	int x, y, yAdd = 0;
	
	for (x = 0; x < xTS*yTS<<2; x++)
	{
		temp[x] = Layer[bVal+x];
	}

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			Layer[bVal+((x+yAdd)<<2)  ] = temp[(((xTS-1-x)+yAdd)<<2)  ];
			Layer[bVal+((x+yAdd)<<2)+1] = temp[(((xTS-1-x)+yAdd)<<2)+1];
			Layer[bVal+((x+yAdd)<<2)+2] = temp[(((xTS-1-x)+yAdd)<<2)+2];
		}
		yAdd += xTS;
	}
}

void TexGen::FlipV(int gL)
{
	unsigned char* temp = mTempLayer;
	int x, y, yAdd = 0, dim = xTS*yTS;
	int bVal = (gL * dim) << 2;
	
	for (x = 0; x < (dim<<2); x++)
	{
		temp[x] = Layer[bVal+x];
	}

	dim -= xTS;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			Layer[bVal+((x+yAdd)<<2)  ] = temp[((x+dim-yAdd)<<2)  ];
			Layer[bVal+((x+yAdd)<<2)+1] = temp[((x+dim-yAdd)<<2)+1];
			Layer[bVal+((x+yAdd)<<2)+2] = temp[((x+dim-yAdd)<<2)+2];
		}
		yAdd += xTS;
	}
}

void TexGen::Shade(unsigned char gL, unsigned char sParams)
{
	int bVal = gL * xTS * yTS << 2;
	int dVal = (sParams >> 4) * xTS * yTS << 2, ch = sParams & 0x0F;
    int ind, x, y, xTS_2 = get2Pow(xTS);

	unsigned char *temp = mTempLayer;

	for (x = 0; x < xTS*yTS<<2; x++) temp[x] = Layer[dVal+x];
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			ind = ((x+(y<<xTS_2))<<2);

			if (Layer[dVal+ind+ch] < 128)
			{
				Layer[bVal+ind  ] = clamp<int>((Layer[bVal+ind  ]*temp[ind+ch])>>7, 0, 255);
				Layer[bVal+ind+1] = clamp<int>((Layer[bVal+ind+1]*temp[ind+ch])>>7, 0, 255);
				Layer[bVal+ind+2] = clamp<int>((Layer[bVal+ind+2]*temp[ind+ch])>>7, 0, 255);
			} else
			{
				Layer[bVal+ind  ] = clamp<int>(Layer[bVal+ind  ]+((temp[ind+ch]-128)<<1), 0, 255);
				Layer[bVal+ind+1] = clamp<int>(Layer[bVal+ind+1]+((temp[ind+ch]-128)<<1), 0, 255);
				Layer[bVal+ind+2] = clamp<int>(Layer[bVal+ind+2]+((temp[ind+ch]-128)<<1), 0, 255);
			}
		}
	}
}

void TexGen::Blend(unsigned char dstLayer, unsigned char srcLayer, BlendMode::Values mode)
{
	int dstVal = dstLayer * xTS * yTS << 2;
	int srcVal = srcLayer * xTS * yTS << 2;
	int ind, x, y, xTS_2 = get2Pow(xTS);

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			ind = ((x+(y<<xTS_2))<<2);

			int srcR = Layer[srcVal+ind  ];
			int srcG = Layer[srcVal+ind+1];
			int srcB = Layer[srcVal+ind+2];
			int dstR = Layer[dstVal+ind  ];
			int dstG = Layer[dstVal+ind+1];
			int dstB = Layer[dstVal+ind+2];

			int resR, resG, resB;

			switch (mode)
			{
			case BlendMode::ADD:
				{
					resR = srcR + dstR;
					resG = srcG + dstG;
					resB = srcB + dstB;
					break;
				}
			case BlendMode::DIFF:
				{
					resR = srcR - dstR;
					resG = srcG - dstG;
					resB = srcB - dstB;
					break;
				}
			case BlendMode::MUL:
				{
					resR = ((srcR * dstR) >> 8);
					resG = ((srcG * dstG) >> 8);
					resB = ((srcB * dstB) >> 8);
					break;
				}
			case BlendMode::DARKEN:
				{
					resR = m_min(srcR, dstR);
					resG = m_min(srcG, dstG);
					resB = m_min(srcB, dstB);
					break;
				}
			case BlendMode::LIGHTEN:
				{
					resR = m_max(srcR, dstR);
					resG = m_max(srcG, dstG);
					resB = m_max(srcB, dstB);
					break;
				}
			case BlendMode::LINEAR_BURN:
				{
					resR = srcR + dstR - 255;
					resG = srcG + dstG - 255;
					resB = srcB + dstB - 255;
					break;
				}
			case BlendMode::SCREEN:
				{
					resR = (srcR + dstR) - ((srcR * dstR) >> 8);
					resG = (srcG + dstG) - ((srcG * dstG) >> 8);
					resB = (srcB + dstB) - ((srcB * dstB) >> 8);
					break;
				}
			};

			Layer[dstVal+ind  ] = clamp<int>(resR, 0, 255);
			Layer[dstVal+ind+1] = clamp<int>(resG, 0, 255);
			Layer[dstVal+ind+2] = clamp<int>(resB, 0, 255);
		}
	}
}

void TexGen::BlendCh(unsigned char dstLayerParams, unsigned char srcLayerParams, BlendMode::Values mode)
{
	int dstVal = (dstLayerParams >> 4) * xTS * yTS << 2, dstCh = dstLayerParams & 0xF;
	int srcVal = (srcLayerParams >> 4) * xTS * yTS << 2, srcCh = srcLayerParams & 0xF;
	int ind, x, y, xTS_2 = get2Pow(xTS);

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			ind = ((x+(y<<xTS_2))<<2);

			int src = Layer[srcVal+ind+srcCh];
			int dst = Layer[dstVal+ind+dstCh];

			int res;

			switch (mode)
			{
			case BlendMode::ADD:
				{
					res = src + dst;
					break;
				}
			case BlendMode::DIFF:
				{
					res = src - dst;
					break;
				}
			case BlendMode::MUL:
				{
					res = ((src * dst) >> 8);
					break;
				}
			case BlendMode::DARKEN:
				{
					res = m_min(src, dst);
					break;
				}
			case BlendMode::LIGHTEN:
				{
					res = m_max(src, dst);
					break;
				}
			case BlendMode::LINEAR_BURN:
				{
					res = src + dst - 255;
					break;
				}
			case BlendMode::SCREEN:
				{
					res = (src + dst) - ((src * dst) >> 8);
					break;
				}
			};

			Layer[dstVal+ind+dstCh] = clamp<int>(res, 0, 255);
		}
	}
}

void TexGen::MixMap(unsigned char dst_srcL0, unsigned char srcL1, unsigned char dL_Param, unsigned char reversed)
{
	int dstVal = dst_srcL0 * xTS * yTS << 2;
	int srcVal = srcL1 * xTS * yTS << 2;
	int mixVal = (dL_Param >> 4) * xTS * yTS << 2, ch = dL_Param & 0xF;
	int x, y, yAdd = 0;

	int nR, nG, nB;
	int coeff[2];

	unsigned char *dstPos = Layer + dstVal, *srcPos = Layer + srcVal, * mixPos = Layer + mixVal + ch;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			coeff[reversed] = *mixPos;
			coeff[1-reversed] = 255 - coeff[reversed];
			mixPos += 4;

			nR = (coeff[0]**(dstPos  ) + coeff[1]**(srcPos++)) >> 8;
			nG = (coeff[0]**(dstPos+1) + coeff[1]**(srcPos++)) >> 8;
			nB = (coeff[0]**(dstPos+2) + coeff[1]**(srcPos++)) >> 8;

			*dstPos++ = clamp<int>(nR, 0, 255);
			*dstPos++ = clamp<int>(nG, 0, 255);
			*dstPos++ = clamp<int>(nB, 0, 255);

			dstPos++; srcPos++;
		}
		yAdd += xTS;
	}
}

void TexGen::Mix(unsigned char gL, unsigned char dL, unsigned char p1, unsigned char p2)
{
	int bVal = gL * xTS * yTS << 2;
	int dVal = dL * xTS * yTS << 2;
	int x, y, yAdd = 0;

	int nR, nG, nB;

	unsigned char *bPos = Layer + bVal, *dPos = Layer + dVal;
	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			nR = (p1**(bPos  ) + p2**(dPos++)) >> 8;
			nG = (p1**(bPos+1) + p2**(dPos++)) >> 8;
			nB = (p1**(bPos+2) + p2**(dPos++)) >> 8;

			*bPos++ = clamp<int>(nR, 0, 255);
			*bPos++ = clamp<int>(nG, 0, 255);
			*bPos++ = clamp<int>(nB, 0, 255);

			bPos++; dPos++;
		}
		yAdd += xTS;
	}
}

void TexGen::Add(unsigned char lParam)
{
	int bVal = ((lParam >> 4) * xTS * yTS)<<2;
	int dVal = ((lParam & 15) * xTS * yTS)<<2;
	int j, nR, nG, nB;

	unsigned char *lPos = Layer + bVal, *dPos = Layer + dVal;
	for (j = 0; j < xTS*yTS; j++)
	{
		nR = *dPos+*(lPos  ); dPos++;
		nG = *dPos+*(lPos+1); dPos++;
		nB = *dPos+*(lPos+2); dPos++;

		*lPos++ = clamp<int>(nR, 0, 255);
		*lPos++ = clamp<int>(nG, 0, 255);
		*lPos++ = clamp<int>(nB, 0, 255);
		lPos++; dPos++;
	}
}

void TexGen::Xchg(unsigned char src_xLC, unsigned char dst_xLC)
{
	unsigned char *lD = Layer + (((dst_xLC >> 4) * xTS * yTS)<<2)+(dst_xLC&0xF) - 4,
				  *lS = Layer + (((src_xLC >> 4) * xTS * yTS)<<2)+(src_xLC&0xF) - 4;

	for (int i = 0; i < xTS*yTS; i++)
		*(lD += 4) = *(lS += 4);
}

void TexGen::Mul(int gL, int dL)
{
	int bVal = gL * xTS * yTS << 2;
	int dVal = dL * xTS * yTS << 2;
	int i;

	unsigned char *bPos = Layer + bVal, *dPos = Layer + dVal;
	for (i = 0; i < xTS*yTS; i++)
	{
		*bPos++ = clamp<int>((*bPos**dPos++)>>8, 0, 255);
		*bPos++ = clamp<int>((*bPos**dPos++)>>8, 0, 255);
		*bPos++ = clamp<int>((*bPos**dPos++)>>8, 0, 255);
		bPos++; dPos++;
	}
}

void TexGen::Merge(int gL, int dL, unsigned char valCol)
{
	int bVal = gL * xTS * yTS << 2;
	int dVal = dL * xTS * yTS << 2;
	int x, y;

	for (y = 0; y < yTS; y++)
	{
		for (x = 0; x < xTS; x++)
		{
			int oR = Layer[dVal+((x+(y*xTS))<<2)  ],
				oG = Layer[dVal+((x+(y*xTS))<<2)+1],
				oB = Layer[dVal+((x+(y*xTS))<<2)+2];
			int nR, nG, nB;
			if ((oR < valCol) && (oG < valCol) && (oB < valCol))
			{
				nR = Layer[bVal+((x+(y*xTS))<<2)  ],
				nG = Layer[bVal+((x+(y*xTS))<<2)+1],
				nB = Layer[bVal+((x+(y*xTS))<<2)+2];
			} else
			{
				nR = oR; nG = oG; nB = oB;
			}
			Layer[bVal+((x+(y*xTS))*4)+0] = nR;
			Layer[bVal+((x+(y*xTS))*4)+1] = nG;
			Layer[bVal+((x+(y*xTS))*4)+2] = nB;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// Private methods
//////////////////////////////////////////////////////////////////////////

void TexGen::Init()
{
	Layer = (unsigned char *)malloc( ((mNumLayers * xTS * yTS) << 2) * sizeof(unsigned char) );
	mTempLayer = (unsigned char *)malloc( ((xTS * yTS) << 2) * sizeof(unsigned char) );

	memset(Layer, 0, ((mNumLayers * xTS * yTS) << 2) * sizeof(unsigned char));
}

void TexGen::Deinit()
{
	if (Layer)
		free(Layer);

	if (mTempLayer)
		free(mTempLayer);
}

float TexGen::PArcTan(float X, float Y)
{
#if 0
	__asm
	{
		FLD X
		FLD Y
		FPATAN
		FSTP X
	}
#else
	X = atan2f(Y, X);
#endif
	return X;
}

void TexGen::GetBerpTexel_RGB(unsigned char *Pos, float u, float v, unsigned char *r, unsigned char *g, unsigned char *b)
{
	while (u < 0.0f) u += xTS;
	while (v < 0.0f) v += yTS;

	int u0 = (int)u, v0 = (int)v;
	int u1 = (u0+1)&(xTS-1), v1 = (v0+1)&(yTS-1);
	int uf = int((u - u0)*256), vf = int((v - v0)*256);

	u0 = u0 & (xTS-1); v0 = v0 & (yTS-1);

	v0 *= xTS; v1 *= xTS;
	int pos[4] =
	{
		(u0 + v0) << 2,
		(u1 + v0) << 2,
		(u0 + v1) << 2,
		(u1 + v1) << 2
	};
	// u0 = (1.0 - uf) * 256
	u0 = 256 - uf; v0 = 256 - vf;
	int w[4] = 
	{
		u0 * v0,
		uf * v0,
		u0 * vf,
		uf * vf
	};

	int tmpcol;
	unsigned char *pPnt1 = Pos+*(pos  ),
				  *pPnt2 = Pos+*(pos+1),
				  *pPnt3 = Pos+*(pos+2),
				  *pPnt4 = Pos+*(pos+3);
	unsigned char *ch[3] = { r, g, b };

#define FILL_CHANNEL(chNum)\
	tmpcol  = *(pPnt1 + chNum)**(w    );\
	tmpcol += *(pPnt2 + chNum)**(w + 1);\
	tmpcol += *(pPnt3 + chNum)**(w + 2);\
	tmpcol += *(pPnt4 + chNum)**(w + 3);\
	**(ch + chNum) = tmpcol >> 16;

	FILL_CHANNEL(0);
	FILL_CHANNEL(1);
	FILL_CHANNEL(2);

#undef FILL_CHANNEL
}

}
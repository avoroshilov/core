/*****************************************************************************

	Created: 19:46 06.11.2004

*****************************************************************************/
#pragma once 

#define MAX_TG_LAYERS 6

#define USE_FONT	0

#if (USE_FONT == 1)
#	include <Windows.h>
#endif

#include <math.h>
#include "helpers/Common.h"
#include "math/AuxMath.h"

namespace textures
{

	class TexPregen
	{
	public:

		unsigned char EnvArray[256*256];
		unsigned char BiasArray[256*256];

		uint RawChunkOfMemSize;
		unsigned char * RawChunkOfMem;

		TexPregen():
			RawChunkOfMem(0),
			RawChunkOfMemSize(0)
		{
		}
	
		~TexPregen()
		{
			DeinitRawMemChunk();
		}

		void InitRawMemChunk(int numBytes)
		{
			RawChunkOfMemSize = numBytes;
			RawChunkOfMem = new unsigned char [numBytes];
		}
		void DeinitRawMemChunk()
		{
			if (RawChunkOfMem)
			{
				delete [] RawChunkOfMem;
				RawChunkOfMem = 0;
			}
		}

		void InitEnv(void)
		{
			int x, y;
			int yPos0, yPos1, yC;
			unsigned char color;

			for (y = 0; y < 128; y++)
			{
				yPos0 = y<<8;
				yPos1 = (255-y)<<8;
				yC = y - 128; yC *= yC;

				for (x = 0; x < 128; x++)
				{
					color = clamp(int(2.0f * sqrtf(float(sqr(x - 128) + yC))), 0, 255);
					EnvArray[     x +yPos0] = color;
					EnvArray[(255-x)+yPos0] = color;
					EnvArray[(255-x)+yPos1] = color;
					EnvArray[     x +yPos1] = color;
				}
			}

		}

		void InitBias(void)
		{
			int x, y;
			float Pow = 2.0f, yAdd = 1 / 128.0f;

			unsigned char *bArray = BiasArray;
			for (y = 0; y < 256; y++)
			{
				for (x = 0; x < 256; x++)
					*bArray++ = int(255 * powf(x / 255.0f, Pow));

				Pow -= yAdd;
			}

		}
	};

	class TexGen
	{
	protected:

		int mNumLayers;

		unsigned char * mTempLayer;

		// WARNING! not thread safe
		static unsigned char * & GetRawChunk()
		{
			static unsigned char * sRawChunk;
			return sRawChunk;
		}
		static uint & GetRawChunkSize()
		{
			static uint sRawChunkSize;
			return sRawChunkSize;
		}

	#if (USE_FONT == 1)
		// Font:
		HDC mdc;
		HFONT hf;
		HBITMAP bm;
		BITMAPINFO bmi;
		RECT r;
	#endif

		void Init();
		void Deinit();

		float PArcTan(float X, float Y);

		void GetBerpTexel_RGB(unsigned char *Pos, float u, float v, unsigned char *r, unsigned char *g, unsigned char *b);

		inline int CatmRomInt(int v0, int v1, int v2, int v3, int x, int distance)
		{
			return ((v1<<1)+((v2-v0)*x)/distance+(((v0<<1)-(v1*5)+(v2<<2)-v3)*x*x)/(sqr(distance))+(((v1*3)-v0-(v2*3)+v3)*x*x*x)/(sqr(distance)*distance)) >> 1;
		}
		inline int LinearInt(int v1, int v2, int x, int dist)
		{
			return (((v2 - v1) * x) >> dist) + v1;
		}

		inline void WrapDist(int & dx, int & dy)
		{
			if (dx < 0) dx = -dx;
			if (dy < 0) dy = -dy;

			if (dx > (xTS >> 1)) dx = xTS - dx;
			if (dy > (yTS >> 1)) dy = yTS - dy;
		}

		void CalcGaussianWeights(float * coeffs, float slope, int halfRad)
		{
			// Slope of the blur could be changed by changing this coefficient
			float sigma = 1.0f + slope * (halfRad - 2.0f) / 128.0f;
			float sqrt2PI = sqrtf(_2PI);

			float sumCoeff = 0.0f;
			for (int i = 0; i <= halfRad; ++i)
			{
				coeffs[i] = expf(-(i*i / (2.0f * sigma*sigma))) / (sqrt2PI * sigma);
				if (i != 0)
				{
					sumCoeff += coeffs[i];
				}
			}
			sumCoeff *= 2.0f;
			sumCoeff += coeffs[0];

			// Normalize coefficients
			for (int i = 0; i <= halfRad; ++i)
			{
				coeffs[i] /= sumCoeff;
			}
		}

	public:

		struct BlendMode
		{
			enum Values
			{
				ADD = 0,
				DIFF,
				MUL,
				DARKEN,
				LIGHTEN,
				LINEAR_BURN,
				SCREEN,

				LAST_VAL
			};
		};

		static void SetRawChunk(unsigned char * rawChunk, uint rawChunkSize)
		{
			uint & tmpRawChunkSize = GetRawChunkSize();
			unsigned char * & tmpRawChunk = GetRawChunk();

			tmpRawChunkSize = rawChunkSize;
			tmpRawChunk = rawChunk;
		}

		int xTS, yTS;
		unsigned char * Layer;

		int GetNumLayers() const { return mNumLayers; }

		TexGen(int nxTS, int nyTS, int maxNumLayers):
			Layer(0),
			mTempLayer(0),
			mNumLayers(maxNumLayers)
		{
			xTS = nxTS;
			yTS = nyTS;

			Init();
		}

		TexGen(int nxTS, int nyTS):
			Layer(0),
			mTempLayer(0),
			mNumLayers(MAX_TG_LAYERS)
		{
			xTS = nxTS;
			yTS = nyTS;

			Init();
		}

		TexGen(int nTS):
			Layer(0),
			mTempLayer(0),
			mNumLayers(MAX_TG_LAYERS)
		{
			xTS = nTS;
			yTS = nTS;

			Init();
		}

		TexGen():
			Layer(0),
			mTempLayer(0),
			mNumLayers(MAX_TG_LAYERS)
		{
			xTS = 256;
			yTS = 256;

			Init();
		}
	
		~TexGen()
		{
			Deinit();
		}

		void ReInit(int newTS_X, int newTS_Y)
		{
			Deinit();

			xTS = newTS_X;
			yTS = newTS_Y;

			Init();
		}

		unsigned char * GetLayer(int numLayer) { return Layer + (numLayer * xTS * yTS << 2); }
		int	GetWidth (void) const { return xTS; }
		int	GetHeight(void) const { return yTS; }

		void SaveToTGA(unsigned char layer, unsigned int num);
		void SaveToTGA(unsigned char layer, const char * name);
		void SaveToTGAS(int cL, const char *fmt, ...);

		void MakeAlpha(unsigned char cLayers);

		void InitFont(char *fontname, int size, int flags);
		void InitFont(char *fontname, int size, int flags, int *Widths);
		void RenderFont(int gL, int x, int y, char *str);
		void FormFont(int x, int y, char *str);
		void GetFontDIB(int gL);
		void DeleteFont(void);

		void Copy(int gL, unsigned char *src);
		void CopySub(int gL, int origX, int origY, unsigned char * src, int sizeX, int sizeY);

		void GetBerpTexel(int gL, float u, float v, unsigned char *r, unsigned char *g, unsigned char *b)
		{
			unsigned char *lPos = Layer + ((gL * xTS * yTS) << 2);
			GetBerpTexel_RGB(lPos, u, v, r, g, b);
		}

		void GetNearestTexel(int gL, float u, float v, unsigned char *r, unsigned char *g, unsigned char *b)
		{
			unsigned char *lPos = Layer + ((gL * xTS * yTS) << 2);
			unsigned int iU = (unsigned int)(u + 0.5f) & (xTS - 1);
			unsigned int iV = (unsigned int)(v + 0.5f) & (yTS - 1);
			unsigned int shift = (iU + iV * xTS) << 2;
			*r = lPos[shift + 0];
			*g = lPos[shift + 1];
			*b = lPos[shift + 2];
		}

		/* Generating */
		void DrawLine(int gL, int x1, int y1, int x2, int y2);
		void GradVert(int gL, int y1, int y2, unsigned char bx, unsigned char by, unsigned char bz, unsigned char ex, unsigned char ey, unsigned char ez);
		void Rect(unsigned char lParams, int xCoord, int yCoord, int xSize, int ySize, int Color, int Angle);
		void Bricks(unsigned char pLayer, unsigned char cRow, unsigned char cCol, unsigned char vSize, unsigned char vAng, unsigned char vCol, unsigned char Shift, unsigned int Spacing);
		void Check(unsigned char lParams);
		void SineWave(int gL, int k);
		void SineVLin(int gL, int K);
		void SineHLin(int gL, int K);
		void SineHCLin(int gL, int K);
		void SineLDg(int gL, int K);
		void SineRDg(unsigned char gL, int K, unsigned char tWraps);
		void DrawMLine(int gL, int x1, int y1, int x2, int y2);
		void Cells(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, unsigned char *Array = 0);
		void CellsEven(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, unsigned char *Array = 0);
		void CellsSplits(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, unsigned char splits, unsigned char xSplitPow, unsigned char ySplitPow, unsigned char *Array = 0);
		void CellsAdv(unsigned char gL, unsigned char Mode, int numPoints, unsigned char Power, unsigned char Variance, int * xCoords, int * yCoords, unsigned char *Array = 0);
		void PointsWhite(int gL, int k);
		void PointsBlack(int gL, int k);
		void FillColor(int gL, unsigned char r, unsigned char g, unsigned char b);	// Filling texMatrix with one color
		void Angles(unsigned char lParam, int Power, unsigned char aNum);
		void Perlin(unsigned char lParam, char Step, bool absMode = false);
		void PerlinCustom(unsigned char lParam, char Step, int * gradX, int * gradY, bool absMode = false);
		void Sub(unsigned char lParam, char Step, bool absMode = false);
		void SubLin(unsigned char lParam, char Step, bool absMode = false);
		void FractalP(unsigned char lParam, unsigned char St_Oc, uint persistence = 256, unsigned char absMode = 0, unsigned char isLinear = 0);
		void SinePlasm(char gL, unsigned char params); // (colX | colY)
		void Noise(int gL, short k);
		void Random(unsigned char lParam, unsigned char maxVal);
		void Env(int gL, unsigned char Dyn1, unsigned char Dyn2, unsigned char Inversed);
		void EnvAdd(int gL, int ox, int oy, unsigned char rad, int mul, unsigned char Inversed);
		void RadGrad(unsigned char lParam, int cenX, int cenY, int * colors, unsigned int numColors);
		void RadGradRGB(int gL, int cenX, int cenY, int * colors, unsigned int numColors);
		void Rays(int gL, int RayCol, int D);
		void FillPieces(int gL, unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2);
		void Cracked(int gL, int Num, int maxLength, int Variation);

		/* Distortion */
		void RotoZoom(int gL, float AngRad, unsigned char Scale = 64);
		void RotoZoomExt(int gL, float AngRad, float ScaleX = 1.0f, float ScaleY = 1.0f, float ScrollX = 0.0f, float ScrollY = 0.0f);
		void RotoZoomMul(int gL, unsigned char times, float AngRad, float ScaleX = 1.0f, float ScaleY = 1.0f, float ScrollX = 0.0f, float ScrollY = 0.0f, unsigned char FadeR = 255, unsigned char FadeG = 255, unsigned char FadeB = 255);
		void Glass(int gL, unsigned char dParam, int k);
		void Kaleid(int gL, char cN);
		void LensDist(int gL, short slE);
		void DirBlur(unsigned char gL, unsigned char dParams, int dist);
		void Blur(int gL, int times);
		void BlurGaussV(unsigned char layer, int halfRad, unsigned char curveSlope = 64, unsigned char brighten = 16, float * extCoeffs = 0);
		void BlurGaussH(unsigned char layer, int halfRad, unsigned char curveSlope = 64, unsigned char brighten = 16, float * extCoeffs = 0);
		void BlurGauss(unsigned char layer, int halfRad, unsigned char curveSlope = 64, unsigned char brighten = 16, float * extCoeffs = 0);
		void SmartBlur(unsigned char bParams, unsigned char MaxDiff);
		void SineDist(int gL, char xK, char yK, int xAmp, int yAmp);
		void SineDistW(int gL, int Times, int AmpXi, int AmpYi, unsigned char Wrap);
		void Turbulence(unsigned char gL, unsigned char xParam, unsigned char yParam, unsigned int Scale, unsigned int Turb);
		void MapDist(unsigned char dL, unsigned char s_xParam, unsigned char s_yParam, short xK, short yK, int xChOffset = 0, int yChOffset = 0);
		void Offset(char gL, short xOffset = 0, short yOffset = 0);
		void Twirl(int gL, int whirlx, int whirly, int radius, unsigned char Whirls, char CosAng);

		/* Color Filters */
		void NMapSobel(unsigned char dstLayer, unsigned char srcLParams, bool isInverted = false);
		void DUDVSobel(unsigned char dstLayer, unsigned char srcLParams);
		void FindEdges(int gL);
		void Outline(unsigned char lParam, unsigned char Amp);
		void Wood(int gL, unsigned char mK);
		void ScaleColor(unsigned char gL, int rK, int gK, int bK);
		void Pixelize(int gL, int PStep);
		void Emboss(char gL, unsigned char Strength);
		void Diff(unsigned char src_xLC, unsigned char Strength);
		void SineColor(unsigned char lParams, float K);
		void SquareEdges(int gL, unsigned char r, unsigned char g, unsigned char b, int times);
		void SquareEdgesChannel(unsigned char lParams, unsigned char c, int times);
		void Plastic(int gL, int times);
		void InvertCh(unsigned char layerCh);
		void Invert(int gL);
		void Invert_RGB(int gL);
		void Bias(int gL, unsigned char Val, unsigned char *Array);
		void Bright(int gL, unsigned char k = 32);
	
		// TODO: remove that and tune in Bright
		void Bright512(int gL, int k = 512);
		void Balance(unsigned char lParam, unsigned char cBeg, unsigned char cEnd);
		void rgb2hsv(unsigned char  r, unsigned char  g, unsigned char  b,
					 unsigned char *h, unsigned char *s, unsigned char *v);
		void hsv2rgb(unsigned char  h, unsigned char  s, unsigned char  v,
					 unsigned char *r, unsigned char *g, unsigned char *b);
		void AdjustHSV(int gL, unsigned char H, unsigned char S, unsigned char V);
		void NormalizeCh(unsigned char lParams, int minVal, int maxVal);
		void Colorize(int gL, unsigned char r, unsigned char g, unsigned char b);
		void ColorizeBW(unsigned char lParam, unsigned char bR, unsigned char bG, unsigned char bB,
											  unsigned char eR, unsigned char eG, unsigned char eB);
		void Contrast(int gL, unsigned short c, unsigned short k = 16);
		void DelChannel(char gL, char Chnl);
		void MkChannel(unsigned char lParams);

		/* Layer Operations */
		void FlipH(int gL);
		void FlipV(int gL);
		void Shade(unsigned char gL, unsigned char sParams);
		void Blend(unsigned char dstLayer, unsigned char srcLayer, BlendMode::Values mode);
		void BlendCh(unsigned char dstLayerParams, unsigned char srcLayerParams, BlendMode::Values mode);
		void MixMap(unsigned char dst_srcL0, unsigned char srcL1, unsigned char dL_Param, unsigned char reversed);
		void Mix(unsigned char gL, unsigned char dL, unsigned char p1, unsigned char p2);
		void Add(unsigned char lParam);
		void Xchg(unsigned char src_xLC, unsigned char dst_xLC);
		void Mul(int gL, int dL);
		void Merge(int gL, int dL, unsigned char valCol);
	};

}
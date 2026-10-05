/** @file

MODULE:			RealYUV420toBGR24CCIR601Converter

TAG:			RYUVRGB24CC

FILE NAME:		RealYUV420toBGR24CCIR601Converter.cpp

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour convertion derived from	the YUV420toRGBConverter base 
			class. Assumes that the colour space of the YUV components
			are defined by the CCIR-601 standard.

REVISION HISTORY:

COPYRIGHT: 

RESTRICTIONS: 
===========================================================================*/
// Note: Removed from toolchain, it is here only as calculation referrence.
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "ConvYUV420toRGB24CCIR601Real.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


/*===========================================================================
				Macros
===========================================================================*/

#define RYUVRGB24CC_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )

//	CCIR-601: The defenition assumes limited Y and UV (CbCr) ranges but the inverse
//						conversion will treat it as full range and only compensate for the
//						shift.
//		r = (y-16) + 1.140(v-128)
//		g = (y-16) - 0.394(u-128) - 0.581(v-128)
//		b = (y-16) + 2.033(u-128)

#define RYUVRGB24CC_U0		 2.033
#define RYUVRGB24CC_U1		-0.394
#define RYUVRGB24CC_V1		-0.581
#define RYUVRGB24CC_V0		 1.140

/*===========================================================================
				Private Methods
===========================================================================*/


bool RealYUV420toBGR24CCIR601Converter::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatIn==MEDIASUBTYPE_RGB24 &&
         (VideoFormatOut==MEDIASUBTYPE_YUV420P_S || VideoFormatOut==MEDIASUBTYPE_I420 || VideoFormatOut==MEDIASUBTYPE_YV12);
}


void RealYUV420toBGR24CCIR601Converter::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
  unsigned char *optr = (unsigned char *)pRgb;
  const unsigned char *py	= (const unsigned char*)pY;
  const unsigned char *pu	= (const unsigned char*)pU;
  const unsigned char *pv	= (const unsigned char*)pV;
  int lumX = _width;
  int lumY = _height;
  int uvX = _width >> 1;

  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;

  for(y = 0,lumposy = 0,uvposy = 0,rgbposy = 0;	y < lumY;	y += 2,lumposy += tworows,uvposy += uvX,rgbposy += rgb2rows)
  {
		int lumpos0 = lumposy;					// Reset to start of rows.
		int lumpos1 = lumposy + lumX;
		int uvpos	 = uvposy;

		int rgbpos0 = rgbposy;
		int rgbpos1 = rgbposy + rgb1row;

		for(x = 0; x < lumX; x += 2)
		{
			double lum00 = (double)( (int)(py[lumpos0++]) - 16);
			double u		 = (double)( (int)(pu[uvpos]) - 128);
			double v		 = (double)( (int)(pv[uvpos++]) - 128);

			// Lum00, u and v.

  		// Fast calculation intermediate variables. 
			double cc = (RYUVRGB24CC_U0 * u) + 0.5;
			double cb = (RYUVRGB24CC_U1 * u) + (RYUVRGB24CC_V1 * v) + 0.5;
			double ca = (RYUVRGB24CC_V0 * v) + 0.5;	

			//printf("%u: cc:%.1f ca:%.1f cb:%.1f Y:%.1f;",y,cc,ca,cb,lum00);

			b = (int)(lum00 + cc);
			g = (int)(lum00 + cb);
			r = (int)(lum00 + ca);
  
			// R, G & B have range 0..255.
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));

			// Lum01.
			double lum01 = (double)( (int)(py[lumpos0++]) - 16 );

			b = (int)(lum01 + cc);
			g = (int)(lum01 + cb);
			r = (int)(lum01 + ca);
  
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));

			// Lum10.
			double lum10 = (double)( (int)(py[lumpos1++]) - 16 );

			b = (int)(lum10 + cc);
			g = (int)(lum10 + cb);
			r = (int)(lum10 + ca);
  
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));

			// Lum11.
			double lum11 = (double)( (int)(py[lumpos1++]) - 16);

			b = (int)(lum11 + cc);
			g = (int)(lum11 + cb);
			r = (int)(lum11 + ca);
  
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));

		}//end for x...
		
	}//end for y...

}//end NonRotateConvert.


void RealYUV420toBGR24CCIR601Converter::RotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
	unsigned char *optr = (unsigned char *)pRgb;
	const unsigned char *py = (const unsigned char*)pY;
	const unsigned char *pu	= (const unsigned char*)pU;
	const unsigned char *pv	= (const unsigned char*)pV;
	int		lumX	= _width;
	int		lumY	= _height;
	int		uvX		= _width >> 1;

	int x,y;
	int lumposy;
	int rgbposx;
	int uvposy;
  int r,b,g;

	int tworows  = lumX << 1;
	//int rgb1row  = (_width * 3);
	int rgb1row  = (_height * 3);

	for(y = 0,lumposy = 0,uvposy = 0,rgbposx = 0;	y < lumY;	y += 2,lumposy += tworows,uvposy += uvX,rgbposx += 6)
	{
		int lumpos0 = lumposy;					// Reset to start of rows.
		int lumpos1 = lumposy + lumX;
		int uvpos	 = uvposy;

		int rgbpos0 = rgbposx;
		int rgbpos1 = rgbposx + 3;

		for(x = 0; x < lumX; x += 2)
		{
			double lum00 = (double)( (int)(py[lumpos0++]) - 16 );
			double u		 = (double)( (int)(pu[uvpos]) - 128 );
			double v		 = (double)( (int)(pv[uvpos++]) - 128 );

			// Lum00, u and v.

  		// Fast calculation intermediate variables. 
			double cc = (RYUVRGB24CC_U0 * u) + 0.5;
			double cb = (RYUVRGB24CC_U1 * u) + (RYUVRGB24CC_V1 * v) + 0.5;
			double ca = (RYUVRGB24CC_V0 * v) + 0.5;

			b = (int)(lum00 + cc);
			g = (int)(lum00 + cb);
			r = (int)(lum00 + ca);
  
			// R, G & B have range 0..255.
			optr[rgbpos0]		= (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos0+1] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos0+2] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));
			rgbpos0 += rgb1row;

			// Lum01.
			double lum01 = (double)( (int)(py[lumpos0++]) - 16 );

			b = (int)(lum01 + cc);
			g = (int)(lum01 + cb);
			r = (int)(lum01 + ca);
  
			optr[rgbpos0]		= (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos0+1] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos0+2] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));
			rgbpos0 += rgb1row;

			// Lum10.
			double lum10 = (double)( (int)(py[lumpos1++]) - 16 );

			b = (int)(lum10 + cc);
			g = (int)(lum10 + cb);
			r = (int)(lum10 + ca);
  
			optr[rgbpos1]		= (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos1+1] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos1+2] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));
			rgbpos1 += rgb1row;

			// Lum11.
			double lum11 = (double)( (int)(py[lumpos1++]) - 16 );

			b = (int)(lum11 + cc);
			g = (int)(lum11 + cb);
			r = (int)(lum11 + ca);
  
			optr[rgbpos1]		= (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(b));
			optr[rgbpos1+1] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(g));
			optr[rgbpos1+2] = (unsigned char)(RYUVRGB24CC_RANGECHECK_0TO255(r));
			rgbpos1 += rgb1row;

		}//end for x...
		
	}//end for y...

}//end NonNonRotateConvert.



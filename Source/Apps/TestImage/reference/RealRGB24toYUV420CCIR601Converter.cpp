/** @file

MODULE:			RealRGB24toYUV420CCIR601Converter

TAG:			RRGB24YUVCC

FILE NAME:		RealRGB24toYUV420CCIR601Converter.cpp

DESCRIPTION:		Double precision floating point RGB 24 bit to YUV420 colour 
			convertion derived from the RGBtoYUV420Converter base class.
			YUV is represented with 8 bpp. Implements the CCIR-601 colour 
			conversion process. Use this implementation as the reference.

REVISION HISTORY:

COPYRIGHT:

RESTRICTIONS: 
===========================================================================*/

#include "ConvRGB24toYUV420CCIR601Real.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


/*===========================================================================
				Constants
===========================================================================*/
//	CCIR-601:
//		Y =  0.257r + 0.504g + 0.098b + 16
//		U	= -0.148r - 0.291g + 0.439b + 128		(Cb)
//		V =  0.439r - 0.368g - 0.071b + 128		(Cr)
#define RRGB24YUVCC_00		 0.257
#define RRGB24YUVCC_01		 0.504
#define RRGB24YUVCC_02		 0.098
#define RRGB24YUVCC_10		-0.148
#define RRGB24YUVCC_11		-0.291
#define RRGB24YUVCC_12		 0.439
#define RRGB24YUVCC_20		 0.439
#define RRGB24YUVCC_21		-0.368
#define RRGB24YUVCC_22		-0.071


/*===========================================================================
			Interface Methods
===========================================================================*/


bool RealRGB24toYUV420CCIR601Converter::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatIn==MEDIASUBTYPE_RGB24 &&
        (VideoFormatOut==MEDIASUBTYPE_YUV420P_S || VideoFormatOut==MEDIASUBTYPE_I420 || VideoFormatOut==MEDIASUBTYPE_YV12);
}


/** Double precision reference implementation.
The full real matix equation is used. The YUV output is represented with
8/16 bits per pel and the CCIR-601 specifies that the Y range is 16..235 
and the UV (CbCr) range is 16..240 centred on 128.
@param pRgb	: Packed RGB 888 format.
@param pY		: Lum plane.
@param pU		: Chr U plane.
@param pV		: Chr V plane.
@return			: none. */
void RealRGB24toYUV420CCIR601Converter::Convert(const unsigned char *pRgb, void* pY, void* pU, void* pV) const
{
  if (_flip)
    FlipConvert(pRgb, (unsigned char*)pY, (unsigned char*)pU, (unsigned char*)pV);
  else
    NonFlipConvert(pRgb, (unsigned char*)pY, (unsigned char*)pU, (unsigned char*)pV);
}


void RealRGB24toYUV420CCIR601Converter::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char *py = pY;
  unsigned char *pu = pU;
  unsigned char *pv = pV;
  const unsigned char* src = pRgb;

	// Y1 & Y2 have range 16..235, U & V have range 16..240.
  double	u,v;
  double	r,g,b;

	// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int	chrOff	= yb*xBlks + xb;
      int	lumOff	= (yb*_width + xb) << 1;
      //unsigned char*	t = src + lumOff*3;
      const unsigned char *t = src + ((((_height - (yb * 2) - 1) * _width) + (xb * 2)) * 3);

		// Top left pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u = 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v = 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

		// Top right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff+1] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u += 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v += 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

      lumOff += _width;
      //t = t + _width*3 - 6;
      t = t - _width*3 - 6;
		// Bottom left pel..
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u += 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v += 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

		// Bottom right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff+1] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u += 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v += 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

		// Average the 4 chr values.
      pu[chrOff] = (unsigned char)( (int)(u + 0.5)/4 );
      pv[chrOff] = (unsigned char)( (int)(v + 0.5)/4 );
    } //end for xb & yb...

} //end Convert.


void RealRGB24toYUV420CCIR601Converter::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char *py = pY;
  unsigned char *pu = pU;
  unsigned char *pv = pV;
  const unsigned char *src = pRgb;

	// Y1 & Y2 have range 16..235, U & V have range 16..240.
  double	u,v;
  double	r,g,b;

	// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int chrOff	= yb*xBlks + xb;
      int lumOff	= (yb*_width + xb) << 1;
      const unsigned char *t = src + lumOff*3;

		// Top left pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u = 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v = 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

		// Top right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff+1] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u += 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v += 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

      lumOff += _width;
      t = t + _width*3 - 6;
		// Bottom left pel..
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u += 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v += 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

		// Bottom right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff+1] = 16 + (unsigned char)(0.5 + RRGB24YUVCC_00*r + RRGB24YUVCC_01*g + RRGB24YUVCC_02*b);

      u += 128 + RRGB24YUVCC_10*r + RRGB24YUVCC_11*g + RRGB24YUVCC_12*b;
      v += 128 + RRGB24YUVCC_20*r + RRGB24YUVCC_21*g + RRGB24YUVCC_22*b;

		// Average the 4 chr values.
      pu[chrOff] = (unsigned char)( (int)(u + 0.5)/4 );
      pv[chrOff] = (unsigned char)( (int)(v + 0.5)/4 );
  }//end for xb & yb...

}//end Convert.

/** @file

MODULE				: RealRGB24toYUV420Converter

FILE NAME			: RealRGB24toYUV420Converter.cpp

DESCRIPTION			: float precision floating point RGB 24 bit to YUV420 colour 
convertion derived from the RGBtoYUV420Converter base class.
YUV is represented with 8 bpp. Use this implementation as 
the reference.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

===========================================================================*/
#include "ConvRGB24toYUV420_Real.h"

#include <uuids.h>


/*===========================================================================
				Defines
===========================================================================*/

#define RRGB_YUVCI2_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )
#define RRGB_YUVCI2_RANGECHECK_N128TO127(x) ( (((x) <= 127)&&((x) >= -128))?((x)):( ((x) > 127)?(127):(-128) ) )

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);



/*===========================================================================
				Interface Methods
===========================================================================*/


bool RealRGB24toYUV420Converter::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatIn==MEDIASUBTYPE_RGB24 &&
         (VideoFormatOut==MEDIASUBTYPE_YUV420P_S || VideoFormatOut==MEDIASUBTYPE_I420 || VideoFormatOut==MEDIASUBTYPE_YV12);
}


/** Double precision reference implementation.
The full real matix equation is used. The YUV output is represented with
8 bits per pel and the UV components are adjusted from their -128..127 range
to 0..255.
@param pRgb	: Packed RGB 888 format.
@param pY		: Lum plane.
@param pU		: Chr U plane.
@param pV		: Chr V plane.
@return			: none. */
void RealRGB24toYUV420Converter::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char*	py = (unsigned char *)pY;
  unsigned char*	pu = (unsigned char *)pU;
  unsigned char*	pv = (unsigned char *)pV;
  unsigned char* src = (unsigned char *)pRgb;

  /// Y have range 0..255, U & V have range -128..127.
  float	u,v;
  float	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int		chrOff	= yb*xBlks + xb;
      int		lumOff	= (yb*_width + xb) << 1;
      unsigned char*	t = src + ((((_height - (yb * 2) - 1) * _width) + (xb * 2)) * 3);
      /// Top left pel.
      b = (*t++);
      g = (*t++);
      r = (*t++);
      py[lumOff] = (unsigned char)RRGB_YUVCI2_RANGECHECK_0TO255((int)(0.5 + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b));

      u = RRGB_YUVCI2_10*r + RRGB_YUVCI2_11*g + RRGB_YUVCI2_12*b;
      v = RRGB_YUVCI2_20*r + RRGB_YUVCI2_21*g + RRGB_YUVCI2_22*b;

      /// Top right pel.
      b = (*t++);
      g = (*t++);
      r = (*t++);
      py[lumOff+1] = (unsigned char)RRGB_YUVCI2_RANGECHECK_0TO255((int)(0.5 + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b));

      u += RRGB_YUVCI2_10*r + RRGB_YUVCI2_11*g + RRGB_YUVCI2_12*b;
      v += RRGB_YUVCI2_20*r + RRGB_YUVCI2_21*g + RRGB_YUVCI2_22*b;

      lumOff += _width;

      t = t - _width*3 - 6;
      /// Bottom left pel.
      b = (*t++);
      g = (*t++);
      r = (*t++);
      py[lumOff] = (unsigned char)RRGB_YUVCI2_RANGECHECK_0TO255((int)(0.5 + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b));

      u += RRGB_YUVCI2_10*r + RRGB_YUVCI2_11*g + RRGB_YUVCI2_12*b;
      v += RRGB_YUVCI2_20*r + RRGB_YUVCI2_21*g + RRGB_YUVCI2_22*b;

      /// Bottom right pel.
      b = (*t++);
      g = (*t++);
      r = (*t++);
      py[lumOff+1] = (unsigned char)RRGB_YUVCI2_RANGECHECK_0TO255((int)(0.5 + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b));

      u += RRGB_YUVCI2_10*r + RRGB_YUVCI2_11*g + RRGB_YUVCI2_12*b;
      v += RRGB_YUVCI2_20*r + RRGB_YUVCI2_21*g + RRGB_YUVCI2_22*b;

      /// Average the 4 chr values.
      int iu = (int)u;
      int iv = (int)v;
      if(iu < 0)	///< Rounding.
        iu -= 2;
      else
        iu += 2;
      if(iv < 0)	///< Rounding.
        iv -= 2;
      else
        iv += 2;

      pu[chrOff] = (unsigned char)(128 + RRGB_YUVCI2_RANGECHECK_N128TO127(iu/4) );
      pv[chrOff] = (unsigned char)(128 + RRGB_YUVCI2_RANGECHECK_N128TO127(iv/4) );
    }//end for xb & yb...
}


void RealRGB24toYUV420Converter::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  unsigned char	r,g,b;
  float	rs,gs,bs;

	// Step in 2x2 pel blocks. (4 pels per block).
  const int xBlks = _width >> 1;
  int yb = _height >> 1;
  while(yb-- > 0)
  {  
    int xb = xBlks;
    while(xb-- > 0)
    {
	// Top left pel.
      bs = b = pRgb[0];
      gs = g = pRgb[1];
      rs = r = pRgb[2];
      int round = (int)(0.5f + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b);
      if(round >= 255) pY[0] = 255;
      else pY[0] = (unsigned char)round;

	// Top right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      round = (int)(0.5f + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b);
      if(round >= 255) pY[1] = 255;
      else pY[1] = (unsigned char)round;

      pRgb += _width*3;
	// Bottom left pel.
      bs += b = pRgb[0];
      gs += g = pRgb[1];
      rs += r = pRgb[2];
      round = (int)(0.5f + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b);
      if(round >= 255) pY[_width] = 255;
      else pY[_width] = (unsigned char)round;

	// Bottom right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      round = (int)(0.5f + RRGB_YUVCI2_00*r + RRGB_YUVCI2_01*g + RRGB_YUVCI2_02*b);
      if(round >= 255) pY[_width+1] = 255;
      else pY[_width+1] = (unsigned char)round;

	// Average the 4 chr values.
      round = ((int)(4*128 + 2 + RRGB_YUVCI2_10*rs + RRGB_YUVCI2_11*gs + RRGB_YUVCI2_12*bs)) / 4;	// Rounding +2.
      if(round<0) *pU++=0;
      else if(round>255) *pU++=255;
      else *pU++ = (unsigned char)round;

      round = ((int)(4*128 + 2 + RRGB_YUVCI2_20*rs + RRGB_YUVCI2_21*gs + RRGB_YUVCI2_22*bs)) / 4;
      if(round<0) *pV++=0;
      else if(round>255) *pV++=255;
      else *pV++ = (unsigned char)round;

      pRgb -= _width*3 - 6;
      pY += 2;
    } //end for xb & yb...
    pY += _width;
    pRgb += 3*_width;
  }
} //end Convert.


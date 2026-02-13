/** @file

MODULE:		RealRGB32toYUV420Converter

FILE NAME:	ConvRGB32toYUV420_Real.cpp

DESCRIPTION:	Double precision floating point RGB 32 bit to YUV420 colour 
		convertion derived from the RGBtoYUV420Converter base class.
		YUV is represented with 8 bpp. Use this implementation as 
		the reference.
					  
LICENSE:	Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
Copyright (c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the DQ nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

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
#include "ConvRGB32toYUV420_Real.h"

#include <uuids.h>


/*===========================================================================
			Interface Methods
===========================================================================*/


bool RealRGB32toYUV420Converter::isInputVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_RGB32 || VideoFormat==MEDIASUBTYPE_ARGB32;
}


void RealRGB32toYUV420Converter::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char *pU, unsigned char* pV) const
{
  unsigned char*	py = (unsigned char *)pY;
  unsigned char*	pu = (unsigned char *)pU;
  unsigned char*	pv = (unsigned char *)pV;  

  /// Y have range 0..255, U & V have range -128..127.
  float	u,v;
  float	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int chrOff	= yb*xBlks + xb;
      int lumOff	= (yb*_width + xb) << 1;
      //unsigned char*	t = src + lumOff*4;
      const unsigned char *t = pRgb + ((((_height - (yb * 2) - 1) * _width) + (xb * 2)) * 4);

      /// Top left pel.  255->0.999.
      b = (*t++);
      g = (*t++);
      r = (*t);
      t += 2;	///< Alpha channel.
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

      u = 128.0f + RRGB24YUVCI2_10*r + RRGB24YUVCI2_11*g + RRGB24YUVCI2_12*b;
      v = 128.0f + RRGB24YUVCI2_20*r + RRGB24YUVCI2_21*g + RRGB24YUVCI2_22*b;

      /// Top right pel.
      b = (*t++);
      g = (*t++);
      r = (*t);
			///< Alpha channel.
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

      u += 128.0f + RRGB24YUVCI2_10*r + RRGB24YUVCI2_11*g + RRGB24YUVCI2_12*b;
      v += 128.0f + RRGB24YUVCI2_20*r + RRGB24YUVCI2_21*g + RRGB24YUVCI2_22*b;

      lumOff += _width;
      //t = t + _width*4 + 2 - 8;
      t = t - _width*4 + 2 - 8;
      /// Bottom left pel.  255->0.999.
      b = (*t++);
      g = (*t++);
      r = (*t);
      t += 2;	///< Alpha channel.
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

      u += 128.0f + RRGB24YUVCI2_10*r + RRGB24YUVCI2_11*g + RRGB24YUVCI2_12*b;
      v += 128.0f + RRGB24YUVCI2_20*r + RRGB24YUVCI2_21*g + RRGB24YUVCI2_22*b;

      /// Bottom right pel.
      b = (*t++);
      g = (*t++);
      r = (*t);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

      u += 128.0f + RRGB24YUVCI2_10*r + RRGB24YUVCI2_11*g + RRGB24YUVCI2_12*b;
      v += 128.0f + RRGB24YUVCI2_20*r + RRGB24YUVCI2_21*g + RRGB24YUVCI2_22*b;

      /// Average the 4 chr values.
      pu[chrOff] = (unsigned char)( (int)((u + 0.5)/4) );
      pv[chrOff] = (unsigned char)( (int)((v + 0.5)/4) );
    }//end for xb & yb...
}


/** Double precision reference implementation.
The full real matix equation is used. The YUV output is represented with
8 bits per pel and the UV components are adjusted from their -128..127 range
to 0..255.
@param pRgb	: Packed RGB 8888 format.
@param pY		: Lum plane.
@param pU		: Chr U plane.
@param pV		: Chr V plane.
@return			: none. */
void RealRGB32toYUV420Converter::NonFlipConvert(const unsigned char *pRgb, unsigned char* pY, unsigned char *pU, unsigned char *pV) const
{
  unsigned char*	py = (unsigned char *)pY;
  unsigned char*	pu = (unsigned char *)pU;
  unsigned char*	pv = (unsigned char *)pV;

  /// Y have range 0..255, U & V have range -128..127.
  float	rs,gs,bs;
  unsigned char r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  const int xBlks = _width >> 1;
  const int yBlks = _height >> 1;
  const unsigned char *t = pRgb;
  for(int yb = 0; yb < yBlks; yb++)
  {
    int xb = xBlks;
    while(xb-- > 0)    
    {
      /// Top left pel.  255->0.999.
      bs = b = pRgb[0];
      gs = g = pRgb[1];
      rs = r = pRgb[2];
		///< Alpha channel.
      py[0] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

	// Top right pel.
      bs += b = pRgb[4];
      gs += g = pRgb[5];
      rs += r = pRgb[6];
		///< Alpha channel.
      py[1] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

      //lumOff += _width;
      pRgb += _width*4;
	// Bottom left pel.  255->0.999.
      bs += b = pRgb[0];
      gs += g = pRgb[1];
      rs += r = pRgb[2];
		///< Alpha channel.
      py[_width] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

	// Bottom righpRgb pel.
      bs += b = pRgb[4];
      gs += g = pRgb[5];
      rs += r = pRgb[6];
      py[_width+1] = (unsigned char)((int)(0.5 + RRGB24YUVCI2_00*r + RRGB24YUVCI2_01*g + RRGB24YUVCI2_02*b));

	// Average the 4 chr values.
      *pu++ = (unsigned char)( (int)((RRGB24YUVCI2_10*rs + RRGB24YUVCI2_11*gs + RRGB24YUVCI2_12*bs + 4*128.0f + 0.5f)/4) );
      *pv++ = (unsigned char)( (int)((RRGB24YUVCI2_20*rs + RRGB24YUVCI2_21*gs + RRGB24YUVCI2_22*bs + 4*128.0f + 0.5f)/4) );

      pRgb -= _width*4 - 8;
      py += 2;
    } //end for xb & yb...
    py += _width;
    pRgb += 4*_width;
  }
}

/** @file

MODULE:			FastFixedPointRGB24toYUV420Converter

FILE NAME:		FastFixedPointRGB24toYUV420Converter.cpp

DESCRIPTION:		Fixed-point RGB 24 bit to YUV420 colour 
			conversion derived from the RGBtoYUV420Converter base class.
			Use this implementation as the reference.

LICENSE: Software License Agreement (BSD License)

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
#include "RGB24toYUV420Converter.h"

#include "typedfs.h"

#include <uuids.h>

/*===========================================================================
			Constants
===========================================================================*/

#define iPrecision 12
const int32_t iRRGB24YUVCI2_00 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_00 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_01 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_01 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_02 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_02 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_10 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_10 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_11 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_11 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_12 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_12 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_20 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_20 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_21 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_21 * (2 << iPrecision));
const int32_t iRRGB24YUVCI2_22 = static_cast<int32_t>(0.5 + RRGB24YUVCI2_22 * (2 << iPrecision));


/*===========================================================================
			Interface Methods
===========================================================================*/


bool ConvRGB24toYUV420FixP::isInputVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_RGB24;
}


void ConvRGB24toYUV420FixP::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  int32_t r,g,b;
  int16_t rs,gs,bs;

  pRgb += (3*_width) * (_height-1);
	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      bs = b = pRgb[0];
      gs = g = pRgb[1];
      rs = r = pRgb[2];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[0]=255;		// y is only positive
            else pY[0] = (unsigned char)y;

	// Top right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[1]=255;		// y is only positive
            else pY[1] = (unsigned char)y;

      pRgb -= 3 * _width;

	// Bottom left pel.
      bs += b = pRgb[0];
      gs += g = pRgb[1];
      rs += r = pRgb[2];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[_width]=255;	// y is only positive
            else pY[_width] = (unsigned char)y;

	// Bottom right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[_width+1]=255;	// y is only positive
            else pY[_width+1] = (unsigned char)y;

	// Average the 4 chr values.
      y = ((iRRGB24YUVCI2_10*rs + iRRGB24YUVCI2_11*gs + iRRGB24YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pU++=0;
      else if(y>=4*255) *pU++=255;
      else *pU++ = (unsigned char)(y>>2);

      y = ((iRRGB24YUVCI2_20*rs + iRRGB24YUVCI2_21*gs + iRRGB24YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pV++=0;
      else if(y>=4*255) *pV++=255;
      else *pV++ = (unsigned char)(y>>2);

      pRgb += 3*_width + 6;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb -= 3 * 3 * _width;
  }
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
void ConvRGB24toYUV420FixP::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  int32_t r,g,b;
  int16_t rs,gs,bs;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      bs = b = pRgb[0];
      gs = g = pRgb[1];
      rs = r = pRgb[2];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[0]=255;		// y is only positive
            else pY[0] = (unsigned char)y;

	// Top right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[1]=255;		// y is only positive
            else pY[1] = (unsigned char)y;

      pRgb += 3 * _width;
	// Bottom left pel.
      bs += b = pRgb[0];
      gs += g = pRgb[1];
      rs += r = pRgb[2];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[_width]=255;	// y is only positive
            else pY[_width] = (unsigned char)y;

	// Bottom right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) pY[_width+1]=255;	// y is only positive
            else pY[_width+1] = (unsigned char)y;

	// Average the 4 chr values.
      y = ((iRRGB24YUVCI2_10*rs + iRRGB24YUVCI2_11*gs + iRRGB24YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pU++=0;
      else if(y>=4*255) *pU++=255;
      else *pU++ = (unsigned char)(y>>2);

      y = ((iRRGB24YUVCI2_20*rs + iRRGB24YUVCI2_21*gs + iRRGB24YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pV++=0;
      else if(y>=4*255) *pV++=255;
      else *pV++ = (unsigned char)(y>>2);

      pRgb -= 3*_width - 6;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb += 3 * _width;
  }
} //end Convert.


/** @file

MODULE:			FastFixedPointRGB24toYUV420Converter

FILE NAME:		FastFixedPointRGB24toYUV420Converter.cpp

DESCRIPTION:		Fixed-point RGB 24 bit to YUV420 CCCIR601 colour 
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
#include "ConvRGB24toYUV420CCIR601v16FixP.h"

#include "typedfs.h"

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
#define RRGB24YUVCCV16_00		 0.257
#define RRGB24YUVCCV16_01		 0.504
#define RRGB24YUVCCV16_02		 0.098
#define RRGB24YUVCCV16_10		-0.148
#define RRGB24YUVCCV16_11		-0.291
#define RRGB24YUVCCV16_12		 0.439
#define RRGB24YUVCCV16_20		 0.439
#define RRGB24YUVCCV16_21		-0.368
#define RRGB24YUVCCV16_22		-0.071


#define iPrecision 12
const int32_t iRRGB_YUVCI2_00 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_00 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_01 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_01 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_02 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_02 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_10 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_10 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_11 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_11 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_12 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_12 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_20 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_20 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_21 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_21 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_22 = static_cast<int32_t>(0.5 + RRGB24YUVCCV16_22 * (2 << iPrecision));


/*===========================================================================
			Interface Methods
===========================================================================*/


bool ConvRGB24toYUV420CCIR601V16FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return true;
  //return VideoFormatIn==MEDIASUBTYPE_RGB24);
	// && (VideoFormatOut==MEDIASUBTYPE_YUV420P_S || VideoFormatOut==MEDIASUBTYPE_I420 || VideoFormatOut==MEDIASUBTYPE_YV12);
}



void ConvRGB24toYUV420CCIR601V16FixP::FlipConvert(const unsigned char* pRgb, uint16_t* pY, uint16_t *pU, uint16_t *pV) const
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
      bs = pRgb[0];
      gs = pRgb[1];
      rs = pRgb[2];
      y = ((iRRGB_YUVCI2_00*rs + iRRGB_YUVCI2_01*gs + iRRGB_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
      pY[0] = y;		// y is only positive

	// Top right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      pY[1] = y;		// y is only positive

      pRgb -= 3 * _width;

	// Bottom left pel.
      bs += b = pRgb[0];
      gs += g = pRgb[1];
      rs += r = pRgb[2];
      y = 16 + ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      pY[_width] = y;		// y is only positive

	// Bottom right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = 16 + ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      pY[_width+1] = y;		// y is only positive

	// Average the 4 chr values.
      y = ((iRRGB_YUVCI2_10*rs + iRRGB_YUVCI2_11*gs + iRRGB_YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      *pU++ = (y>>2);

      y = ((iRRGB_YUVCI2_20*rs + iRRGB_YUVCI2_21*gs + iRRGB_YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      *pV++ = (y>>2);

      pRgb += 3*_width + 6;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb -= 3 * 3 * _width;
  }
}


/** Double precision reference implementation.
The full real matix equation is used. The YUV output is represented with 8 bits
per pel and the UV components are adjusted from their -128..127 range to 0..255.
 @param[in] pRgb	Packed RGB 888 format.
 @param[out] pY		Lum plane.
 @param[out] pU		Chr U plane.
 @param[out] pV		Chr V plane.
 @return		none */
void ConvRGB24toYUV420CCIR601V16FixP::NonFlipConvert(const unsigned char* pRgb, uint16_t* pY, uint16_t *pU, uint16_t *pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      bs = pRgb[0];
      gs = pRgb[1];
      rs = pRgb[2];
      y = 16 + ((iRRGB_YUVCI2_00*rs + iRRGB_YUVCI2_01*gs + iRRGB_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
      pY[0] = y;	// y is only positive

	// Top right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = 16 + ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));      
      pY[1] = y;	// y is only positive

      pRgb += 3 * _width;
	// Bottom left pel.
      bs += b = pRgb[0];
      gs += g = pRgb[1];
      rs += r = pRgb[2];
      y = 16 + ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      pY[_width] = y;	// y is only positive

	// Bottom right pel.
      bs += b = pRgb[3];
      gs += g = pRgb[4];
      rs += r = pRgb[5];
      y = 16 + ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));      
      pY[_width+1] = y;	// y is only positive

	// Average the 4 chr values.
      y = ((iRRGB_YUVCI2_10*rs + iRRGB_YUVCI2_11*gs + iRRGB_YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      *pU++ = (y>>2);

      y = ((iRRGB_YUVCI2_20*rs + iRRGB_YUVCI2_21*gs + iRRGB_YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      *pV++ = (y>>2);

      pRgb -= 3*_width - 6;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb += 3 * _width;
  }
} //end Convert.


/** @file

MODULE:			FastLookupTableRGB24toYUV420Converter

FILE NAME:		FastLookupTableRGB24toYUV420Converter.cpp

DESCRIPTION:		Lookup-table-based RGB 24 bit to YUV420 colour 
			conversion derived from the RGBtoYUV420Converter base class.
			YUV is represented with 16 bpp. Use this implementation as the reference.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.

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

// Note: Accessing a table instead of multiplication looks like smart idea.
//      Unfortunatelly this algorithm is not performing good.
// RGB32 to YUV to resolution 1920,1080 
// Win32 time double 13.762[ms] LUT 20.548[ms] fix point 10.705[ms] orig 17.957[ms]
// Win64 time double 12.332[ms] LUT 19.772[ms] fix point 9.089[ms] orig 16.788[ms]
// Nowadays CPU can do multiplication very fast. It makes no sense to create table.

#include "ConvRGB24toYUV420_LUT.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


/*===========================================================================
			Defines
===========================================================================*/


void ConvRGB24toYUV42_float_LUT::initLookupTable(void)
{
  for(unsigned i = 0; i<256; ++i)
  {
    m_RRGB_YUVCI2_00[i] = i * RRGB_YUVCI2_00;
    m_RRGB_YUVCI2_01[i] = i * RRGB_YUVCI2_01;
    m_RRGB_YUVCI2_02[i] = i * RRGB_YUVCI2_02;
    m_RRGB_YUVCI2_10[i] = i * RRGB_YUVCI2_10;
    m_RRGB_YUVCI2_11[i] = i * RRGB_YUVCI2_11;
    m_RRGB_YUVCI2_12[i] = i * RRGB_YUVCI2_12;
    m_RRGB_YUVCI2_20[i] = i * RRGB_YUVCI2_20;
    m_RRGB_YUVCI2_21[i] = i * RRGB_YUVCI2_21;
    m_RRGB_YUVCI2_22[i] = i * RRGB_YUVCI2_22;
  }
}

/*===========================================================================
			Interface Methods
===========================================================================*/

bool ConvRGB24toYUV42_float_LUT::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
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
void ConvRGB24toYUV42_float_LUT::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  float u, v;
		/// Y has range 0..255, U & V have range -128..127.
  pRgb += (3*_width) * (_height-1);

  /// Step in 2x2 pel blocks. (4 pels per block).
  const int xBlks = _width >> 1;
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = xBlks;
    while(xb-- > 0)
    {
		// Top left pel
      unsigned char b = pRgb[0];
      unsigned char g = pRgb[1];
      unsigned char r = pRgb[2];
      int round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[0] = 255;
      else pY[0] = (unsigned char)round;

      u = m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v = m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

		// Top right pel
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[1] = 255;
      else pY[1] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

      pRgb -= 3 * _width;      

		// Bottom left pel
      b = pRgb[0];
      g = pRgb[1];
      r = pRgb[2];
      round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[_width] = 255;
      else pY[_width] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Bottom right pel.
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[_width+1] = 255;
      pY[_width+1] = (unsigned char)round;      

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

		// Average the 4 chr values.
      round = ((int)(4*128 + 2 + u)) / 4;	// Rounding +2.
      if(round<0) *pU++=0;
      else if(round>255) *pU++=255;
      else *pU++ = (unsigned char)round;

      round = ((int)(4*128 + 2 + v)) / 4;
      if(round<0) *pV++=0;
      else if(round>255) *pV++=255;
      else *pV++ = (unsigned char)round;

      pRgb += 3*_width + 6;
      pY += 2;
    }//end for xb & yb...

    pY += _width;
    pRgb -= 3 * 3 * _width;
  }
}


void ConvRGB24toYUV42_float_LUT::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  float u,v;
	// Y has range 0..255, U & V have range -128..127.

	// Step in 2x2 pel blocks. (4 pels per block).
  const int xBlks = _width >> 1;
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = xBlks;
    while(xb-- > 0)
    {
	// Top left pel.
      unsigned char b = pRgb[0];
      unsigned char g = pRgb[1];
      unsigned char r = pRgb[2];
      int round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[0] = 255;
      else pY[0] = (unsigned char)round;

      u = m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v = m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Top right pel.
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[1] = 255;
      else pY[1] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

      pRgb += _width*3;

	// Bottom left pel.
      b = pRgb[0];
      g = pRgb[1];
      r = pRgb[2];
      round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[_width] = 255;
      else pY[_width] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Bottom right pel.
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (int)(0.5 + m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b]);
      if(round >= 255) pY[_width+1] = 255;
      pY[_width+1] = (unsigned char)round;      

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Average the 4 chr values.
      round = ((int)(4*128 + 2 + u)) / 4;	// Rounding +2.
      if(round<0) *pU++=0;
      else if(round>255) *pU++=255;
      else *pU++ = (unsigned char)round;

      round = ((int)(4*128 + 2 + v)) / 4;
      if(round<0) *pV++=0;
      else if(round>255) *pV++=255;
      else *pV++ = (unsigned char)round;

      pRgb -= _width*3 - 6;
      pY += 2;
    } //end for xb...

    pRgb += 3*_width;
    pY += _width;
  }
} //end Convert.


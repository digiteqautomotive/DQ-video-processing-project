/** @file

MODULE:			FastLookupTableRGB24toYUV420Converter

FILE NAME:		FastLookupTableRGB24toYUV420Converter.cpp

DESCRIPTION:		Lookup-table-based RGB 24 bit to YUV420 colour 
			conversion derived from the RGBtoYUV420Converter base class.
			YUV is represented with 16 bpp. Use this implementation as the reference.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/

// Note: Accessing a table instead of multiplication looks like smart idea.
//      Unfortunatelly this algorithm is not performing good.
// RGB32 to YUV to resolution 1920,1080 
// Win32 time double 13.762[ms] LUT 20.548[ms] fix point 10.705[ms] orig 17.957[ms]
// Win64 time double 12.332[ms] LUT 19.772[ms] fix point 9.089[ms] orig 16.788[ms]
// Nowadays CPU can do multiplication very fast. It makes no sense to create table.

#include "ConvRGB24toYUV420_fixpLUT.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


/*===========================================================================
			Defines
===========================================================================*/

#define iPrecision 12


void ConvRGB24toYUV42_fixp_LUT::initLookupTable(void)
{
  for(unsigned i = 0; i<256; ++i)
  {
    m_RRGB_YUVCI2_00[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_00 * (2 << iPrecision));
    m_RRGB_YUVCI2_01[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_01 * (2 << iPrecision));
    m_RRGB_YUVCI2_02[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_02 * (2 << iPrecision));
    m_RRGB_YUVCI2_10[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_10 * (2 << iPrecision));
    m_RRGB_YUVCI2_11[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_11 * (2 << iPrecision));
    m_RRGB_YUVCI2_12[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_12 * (2 << iPrecision));
    m_RRGB_YUVCI2_20[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_20 * (2 << iPrecision));
    m_RRGB_YUVCI2_21[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_21 * (2 << iPrecision));
    m_RRGB_YUVCI2_22[i] = static_cast<int32_t>(0.5 + i*RRGB_YUVCI2_22 * (2 << iPrecision));
  }
}

/*===========================================================================
			Interface Methods
===========================================================================*/

bool ConvRGB24toYUV42_fixp_LUT::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
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
void ConvRGB24toYUV42_fixp_LUT::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  int32_t u, v;
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
      int32_t round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[0] = 255;
      else pY[0] = (unsigned char)round;

      u = m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v = m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

		// Top right pel
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[1] = 255;
      else pY[1] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

      pRgb -= 3 * _width;      

		// Bottom left pel
      b = pRgb[0];
      g = pRgb[1];
      r = pRgb[2];
      round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[_width] = 255;
      else pY[_width] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

		// Bottom right pel.
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision))  >> (iPrecision + 1);
      if(round >= 255) pY[_width+1] = 255;
      pY[_width+1] = (unsigned char)round;      

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

		// Average the 4 chr values.
      round = (u >> (iPrecision + 1)) + 4*128 + 2;	// Rounding +2.
      if(round<0) *pU++=0;
      else if(round>4*255) *pU++=255;
      else *pU++ = (unsigned char)(round>>2);

      round = (v >> (iPrecision + 1)) + 4*128 + 2;
      if(round<0) *pV++=0;
      else if(round>4*255) *pV++=255;
      else *pV++ = (unsigned char)(round>>2);

      pRgb += 3*_width + 6;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb -= 3 * 3 * _width;
  }
}


void ConvRGB24toYUV42_fixp_LUT::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  int32_t u,v;
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
      int32_t round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[0] = 255;
      else pY[0] = (unsigned char)round;

      u = m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v = m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Top right pel.
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[1] = 255;
      else pY[1] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

      pRgb += _width*3;

	// Bottom left pel.
      b = pRgb[0];
      g = pRgb[1];
      r = pRgb[2];
      round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[_width] = 255;
      else pY[_width] = (unsigned char)round;

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Bottom right pel.
      b = pRgb[3];
      g = pRgb[4];
      r = pRgb[5];
      round = (m_RRGB_YUVCI2_00[r] + m_RRGB_YUVCI2_01[g] + m_RRGB_YUVCI2_02[b] + (1<<iPrecision)) >> (iPrecision + 1);
      if(round >= 255) pY[_width+1] = 255;
      pY[_width+1] = (unsigned char)round;      

      u += m_RRGB_YUVCI2_10[r] + m_RRGB_YUVCI2_11[g] + m_RRGB_YUVCI2_12[b];
      v += m_RRGB_YUVCI2_20[r] + m_RRGB_YUVCI2_21[g] + m_RRGB_YUVCI2_22[b];

	// Average the 4 chr values.
      round = (u >> (iPrecision + 1)) + 4*128 + 2;	// Rounding +2.
      if(round<0) *pU++=0;
      else if(round>4*255) *pU++=255;
      else *pU++ = (unsigned char)(round>>2);

      round = (v >> (iPrecision + 1)) + 4*128 + 2;
      if(round<0) *pV++=0;
      else if(round>4*255) *pV++=255;
      else *pV++ = (unsigned char)(round>>2);

      pRgb -= _width*3 - 6;
      pY += 2;
    } //end for xb...

    pRgb += 3*_width;
    pY += _width;
  }
} //end Convert.


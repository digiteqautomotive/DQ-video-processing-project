/** @file

MODULE:			FastFixedPointRGB555toYUV420Converter

FILE NAME:		FastFixedPointRGB555toYUV420Converter.cpp

DESCRIPTION:		Fixed-point RGB 24 bit to YUV420 colour 
			conversion derived from the RGBtoYUV420Converter base class.
			Use this implementation as the reference.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#include "ConvRGB555toYUV420.h"

#include "typedfs.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


/*===========================================================================
			Constants
===========================================================================*/

#define iPrecision 12
const int32_t iRRGB555_YUVCI2_00 = static_cast<int32_t>(0.5 + 8.2258*RRGB_YUVCI2_00 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_01 = static_cast<int32_t>(0.5 + 8.2258*RRGB_YUVCI2_01 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_02 = static_cast<int32_t>(0.5 + 8.2258*RRGB_YUVCI2_02 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_10 = static_cast<int32_t>(0.5 + 4*8.2258*RRGB_YUVCI2_10 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_11 = static_cast<int32_t>(0.5 + 4*8.2258*RRGB_YUVCI2_11 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_12 = static_cast<int32_t>(0.5 + 4*8.2258*RRGB_YUVCI2_12 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_20 = static_cast<int32_t>(0.5 + 4*8.2258*RRGB_YUVCI2_20 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_21 = static_cast<int32_t>(0.5 + 4*8.2258*RRGB_YUVCI2_21 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_22 = static_cast<int32_t>(0.5 + 4*8.2258*RRGB_YUVCI2_22 * (2 << iPrecision));


/*===========================================================================
			Interface Methods
===========================================================================*/


bool ConvRGB555toYUV420FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatIn==MEDIASUBTYPE_RGB555 &&
        (VideoFormatOut==MEDIASUBTYPE_YUV420P_S || VideoFormatOut==MEDIASUBTYPE_I420 || VideoFormatOut==MEDIASUBTYPE_YV12);
}


void ConvRGB555toYUV420FixP::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  int32_t r,g,b;
  int16_t rs,gs,bs;
  uint16_t val;

  pRgb += (2*_width) * (_height-1);
	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      val = LD_UINT16_LO(pRgb);
      rs = (val>>10) & 0x1F;
      gs = (val>>5) & 0x1F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*rs + iRRGB555_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[0]=255;		// y is only positive
            else pY[0] = (unsigned char)y;

	// Top right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs += r = (val>>10) & 0x1F;
      gs += g = (val>>5) & 0x1F;
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[1]=255;		// y is only positive
            else pY[1] = (unsigned char)y;

      pRgb -= 2 * _width;

	// Bottom left pel.
      val = LD_UINT16_LO(pRgb);      
      rs += r = (val>>10) & 0x1F;
      gs += g = (val>>5) & 0x1F;     
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width]=255;	// y is only positive
            else pY[_width] = (unsigned char)y;

	// Bottom right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs += r = (val>>10) & 0x1F;
      bs += b = (val>>5) & 0x1F;
      gs += g = val & 0x1F;

      y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width+1]=255;	// y is only positive
            else pY[_width+1] = (unsigned char)y;

	// Average the 4 chr values.
      y = ((iRRGB555_YUVCI2_10*rs + iRRGB555_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pU++=0;
      else if(y>=4*255) *pU++=255;
      else *pU++ = (unsigned char)(y>>2);

      y = ((iRRGB555_YUVCI2_20*rs + iRRGB555_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pV++=0;
      else if(y>=4*255) *pV++=255;
      else *pV++ = (unsigned char)(y>>2);

      pRgb += 2*_width + 4;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb -= 3 * 2 * _width;
  }
}


/** Fixed point precision reference implementation.
The full real matix equation is used. The YUV output is represented with
8 bits per pel and the UV components are adjusted from their -128..127 range
to 0..255.
@param pRgb: Packed RGB 888 format.
@param pY: Lum plane.
@param pU: Chr U plane.
@param pV: Chr V plane.
@return: none. */
void ConvRGB555toYUV420FixP::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;
  uint16_t val;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      val = LD_UINT16_LO(pRgb);
      rs = (val>>10) & 0x1F;
      gs = (val>>5) & 0x1F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*rs + iRRGB555_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[0]=255;		// y is only positive
            else pY[0] = (unsigned char)y;

	// Top right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs = (val>>10) & 0x1F;
      gs = (val>>5) & 0x1F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[1]=255;		// y is only positive
            else pY[1] = (unsigned char)y;

      pRgb += 2 * _width;
	// Bottom left pel.
      val = LD_UINT16_LO(pRgb);
      rs = (val>>10) & 0x1F;
      gs = (val>>5) & 0x1F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width]=255;	// y is only positive
            else pY[_width] = (unsigned char)y;

	// Bottom right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs = (val>>10) & 0x1F;
      gs = (val>>5) & 0x1F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width+1]=255;	// y is only positive
            else pY[_width+1] = (unsigned char)y;

	// Average the 4 chr values.
      y = ((iRRGB555_YUVCI2_10*rs + iRRGB555_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pU++=0;
      else if(y>=4*255) *pU++=255;
      else *pU++ = (unsigned char)(y>>2);

      y = ((iRRGB555_YUVCI2_20*rs + iRRGB555_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pV++=0;
      else if(y>=4*255) *pV++=255;
      else *pV++ = (unsigned char)(y>>2);

      pRgb -= 2*_width - 4;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb += 2 * _width;
  }
} //end Convert.


/////////////////////////////////////////////////////////////////////////////////////////////////////

const int32_t iRRGB565_YUVCI2_01 = static_cast<int32_t>(0.5 + 4.04762*RRGB_YUVCI2_01 * (2 << iPrecision));
const int32_t iRRGB565_YUVCI2_11 = static_cast<int32_t>(0.5 + 2*4.04762*RRGB_YUVCI2_11 * (2 << iPrecision));
const int32_t iRRGB565_YUVCI2_21 = static_cast<int32_t>(0.5 + 2*4.04762*RRGB_YUVCI2_21 * (2 << iPrecision));


bool ConvRGB565toYUV420FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatIn==MEDIASUBTYPE_RGB565 &&
        (VideoFormatOut==MEDIASUBTYPE_YUV420P_S || VideoFormatOut==MEDIASUBTYPE_I420 || VideoFormatOut==MEDIASUBTYPE_YV12);
}


void ConvRGB565toYUV420FixP::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  int32_t r,g,b;
  int16_t rs,gs,bs;
  uint16_t val;

  pRgb += (2*_width) * (_height-1);
	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      val = LD_UINT16_LO(pRgb);
      rs = (val>>11) & 0x1F;
      gs = (val>>5) & 0x3F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*rs + iRRGB565_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[0]=255;		// y is only positive
            else pY[0] = (unsigned char)y;

	// Top right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs += r = (val>>11) & 0x1F;
      gs += g = (val>>5) & 0x3F;
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[1]=255;		// y is only positive
            else pY[1] = (unsigned char)y;

      pRgb -= 2 * _width;

	// Bottom left pel.
      val = LD_UINT16_LO(pRgb);      
      rs += r = (val>>11) & 0x1F;
      gs += g = (val>>5) & 0x3F;     
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width]=255;	// y is only positive
            else pY[_width] = (unsigned char)y;

	// Bottom right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs += r = (val>>11) & 0x1F;
      bs += b = (val>>5) & 0x3F;
      gs += g = val & 0x1F;

      y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width+1]=255;	// y is only positive
            else pY[_width+1] = (unsigned char)y;

	// Average the 4 chr values.
      y = ((iRRGB555_YUVCI2_10*rs + iRRGB565_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pU++=0;
      else if(y>=4*255) *pU++=255;
      else *pU++ = (unsigned char)(y>>2);

      y = ((iRRGB555_YUVCI2_20*rs + iRRGB565_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pV++=0;
      else if(y>=4*255) *pV++=255;
      else *pV++ = (unsigned char)(y>>2);

      pRgb += 2*_width + 4;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb -= 3 * 2 * _width;
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
void ConvRGB565toYUV420FixP::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;
  uint16_t val;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      val = LD_UINT16_LO(pRgb);
      rs = (val>>11) & 0x1F;
      gs = (val>>5) & 0x3F;
      bs = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*rs + iRRGB565_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[0]=255;		// y is only positive
            else pY[0] = (unsigned char)y;

	// Top right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs += r = (val>>11) & 0x1F;
      gs += g = (val>>5) & 0x3F;
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[1]=255;		// y is only positive
            else pY[1] = (unsigned char)y;

      pRgb += 2 * _width;
	// Bottom left pel.
      val = LD_UINT16_LO(pRgb);
      rs += r = (val>>11) & 0x1F;
      gs += g = (val>>5) & 0x3F;
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width]=255;	// y is only positive
            else pY[_width] = (unsigned char)y;

	// Bottom right pel.
      val = LD_UINT16_LO(pRgb+2);
      rs += r = (val>>11) & 0x1F;
      gs += g = (val>>5) & 0x3F;
      bs += b = val & 0x1F;
      y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
      if(y>=255) pY[_width+1]=255;	// y is only positive
            else pY[_width+1] = (unsigned char)y;

	// Average the 4 chr values.
      y = ((iRRGB555_YUVCI2_10*rs + iRRGB565_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pU++=0;
      else if(y>=4*255) *pU++=255;
      else *pU++ = (unsigned char)(y>>2);

      y = ((iRRGB555_YUVCI2_20*rs + iRRGB565_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;
      if(y<=0) *pV++=0;
      else if(y>=4*255) *pV++=255;
      else *pV++ = (unsigned char)(y>>2);

      pRgb -= 2*_width - 4;
      pY += 2;
    } //end for xb...

    pY += _width;
    pRgb += 2 * _width;
  }
} //end Convert.


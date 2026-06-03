/** @file

MODULE:			FastFixedPointRGB24toYUV420Converter

FILE NAME:		FastFixedPointRGB24toYUV420Converter.cpp

DESCRIPTION:		Fixed-point RGB 24 bit to YUV420 colour 
			conversion derived from the RGBtoYUV420Converter base class.
			Use this implementation as the reference.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#include "ConvRGBtoYUYV.h"

#include "typedfs.h"

#include <uuids.h>


/*===========================================================================
			Constants
===========================================================================*/

#define iPrecision 12
const int32_t iRRGB_YUVCI2_00 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_00 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_01 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_01 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_02 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_02 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_10 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_10 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_11 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_11 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_12 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_12 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_20 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_20 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_21 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_21 * (2 << iPrecision));
const int32_t iRRGB_YUVCI2_22 = static_cast<int32_t>(0.5 + RRGB_YUVCI2_22 * (2 << iPrecision));


/*===========================================================================
			Interface Methods
===========================================================================*/


bool ConvRGB24toYUYV_FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatIn==MEDIASUBTYPE_RGB24 &&
        (VideoFormatOut==MEDIASUBTYPE_YUYV || VideoFormatOut==MEDIASUBTYPE_YUY2);
}


void ConvRGB24toYUYV_FixP::Convert(const void* pBufferIn, void* pBufferOut) const
{
  const unsigned char *pRgb = (const unsigned char*)pBufferIn;
  unsigned char *pYUYV = (unsigned char*)pBufferOut;
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height;

  if(_flip)
  {
    pRgb += 3*_width * (_height-1);
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
        if(y>=255) pYUYV[0]=255;		// y is only positive
              else pYUYV[0] = (unsigned char)y;

		// Top right pel.
        bs += b = pRgb[3];
        gs += g = pRgb[4];
        rs += r = pRgb[5];
        y = ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
              else pYUYV[2] = (unsigned char)y;

		// Average the 4 chr values.
        y = ((iRRGB_YUVCI2_10*rs + iRRGB_YUVCI2_11*gs + iRRGB_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB_YUVCI2_20*rs + iRRGB_YUVCI2_21*gs + iRRGB_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 6;
        pYUYV += 4;
      } //end for xb...
      pRgb -= 2 * 3 * _width;
    }
  }
  else
  {
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
        if(y>=255) pYUYV[0]=255;		// y is only positive
            else pYUYV[0] = (unsigned char)y;

	// Top right pel.
        bs += b = pRgb[3];
        gs += g = pRgb[4];
        rs += r = pRgb[5];
        y = ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
             else pYUYV[2] = (unsigned char)y;

		// Average the 2 chr values.
        y = ((iRRGB_YUVCI2_10*rs + iRRGB_YUVCI2_11*gs + iRRGB_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB_YUVCI2_20*rs + iRRGB_YUVCI2_21*gs + iRRGB_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 6;
        pYUYV += 4;
      } //end for xb...
    }
  }
} //end Convert.


//////////////////////////////////////////////////////////////////


bool ConvRGB32toYUYV_FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatIn==MEDIASUBTYPE_RGB32 || VideoFormatIn==MEDIASUBTYPE_ARGB32) &&
         (VideoFormatOut==MEDIASUBTYPE_YUYV || VideoFormatOut==MEDIASUBTYPE_YUY2);
}


void ConvRGB32toYUYV_FixP::Convert(const void* pBufferIn, void* pBufferOut) const
{
  const unsigned char *pRgb = (const unsigned char*)pBufferIn;
  unsigned char *pYUYV = (unsigned char*)pBufferOut;
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height;

  if(_flip)
  {
    pRgb += 4*_width * (_height-1);
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
        if(y>=255) pYUYV[0]=255;		// y is only positive
              else pYUYV[0] = (unsigned char)y;

		// Top right pel.
        bs += b = pRgb[4];
        gs += g = pRgb[5];
        rs += r = pRgb[6];
        y = ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
              else pYUYV[2] = (unsigned char)y;

		// Average the 4 chr values.
        y = ((iRRGB_YUVCI2_10*rs + iRRGB_YUVCI2_11*gs + iRRGB_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB_YUVCI2_20*rs + iRRGB_YUVCI2_21*gs + iRRGB_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 8;
        pYUYV += 4;
      } //end for xb...
      pRgb -= 2 * 4 * _width;
    }
  }
  else
  {
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
        if(y>=255) pYUYV[0]=255;		// y is only positive
            else pYUYV[0] = (unsigned char)y;

	// Top right pel.
        bs += b = pRgb[4];
        gs += g = pRgb[5];
        rs += r = pRgb[6];
        y = ((iRRGB_YUVCI2_00*r + iRRGB_YUVCI2_01*g + iRRGB_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
             else pYUYV[2] = (unsigned char)y;

		// Average the 2 chr values.
        y = ((iRRGB_YUVCI2_10*rs + iRRGB_YUVCI2_11*gs + iRRGB_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB_YUVCI2_20*rs + iRRGB_YUVCI2_21*gs + iRRGB_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 8;
        pYUYV += 4;
      } //end for xb...
    }
  }
} //end Convert.


//////////////////////////////////////////////////////////////////

// https://learn.microsoft.com/en-us/windows/win32/directshow/uncompressed-rgb-video-subtypes
//	High-order byte:    Low-order byte: 
//	X R R R R R G G     G G G B B B B B 

const int32_t iRRGB555_YUVCI2_00 = static_cast<int32_t>(0.5 + 8.2258*RRGB_YUVCI2_00 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_01 = static_cast<int32_t>(0.5 + 8.2258*RRGB_YUVCI2_01 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_02 = static_cast<int32_t>(0.5 + 8.2258*RRGB_YUVCI2_02 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_10 = static_cast<int32_t>(0.5 + 2*8.2258*RRGB_YUVCI2_10 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_11 = static_cast<int32_t>(0.5 + 2*8.2258*RRGB_YUVCI2_11 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_12 = static_cast<int32_t>(0.5 + 2*8.2258*RRGB_YUVCI2_12 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_20 = static_cast<int32_t>(0.5 + 2*8.2258*RRGB_YUVCI2_20 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_21 = static_cast<int32_t>(0.5 + 2*8.2258*RRGB_YUVCI2_21 * (2 << iPrecision));
const int32_t iRRGB555_YUVCI2_22 = static_cast<int32_t>(0.5 + 2*8.2258*RRGB_YUVCI2_22 * (2 << iPrecision));


bool ConvRGB555toYUYV_FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatIn==MEDIASUBTYPE_RGB555) &&
         (VideoFormatOut==MEDIASUBTYPE_YUYV || VideoFormatOut==MEDIASUBTYPE_YUY2);
}


void ConvRGB555toYUYV_FixP::Convert(const void* pBufferIn, void* pBufferOut) const
{
  const uint16_t *pRgb = (const uint16_t*)pBufferIn;
  unsigned char *pYUYV = (unsigned char*)pBufferOut;
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;
  uint16_t val;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height;

  if(_flip)
  {
    pRgb += _width * (_height-1);
    while(yb-- > 0)
    {
      int xb = _width >> 1;
      while(xb-- > 0)
      {
		// Left pel.
        val = LD_UINT16_LO(pRgb);
        rs = (val>>10) & 0x1F;
        gs = (val>>5) & 0x1F;
        bs = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*rs + iRRGB555_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[0]=255;		// y is only positive
              else pYUYV[0] = (unsigned char)y;

		// Right pel.
        val = LD_UINT16_LO(pRgb+1);
        rs += r = (val>>10) & 0x1F;
        gs += g = (val>>5) & 0x1F;
        bs += b = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
              else pYUYV[2] = (unsigned char)y;

		// Average the 4 chr values.
        y = ((iRRGB555_YUVCI2_10*rs + iRRGB555_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB555_YUVCI2_20*rs + iRRGB555_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 2;
        pYUYV += 4;
      } //end for xb...
      pRgb -= 2 * _width;
    }
  }
  else
  {
    while(yb-- > 0)
    {
      int xb = _width >> 1;
      while(xb-- > 0)
      {
		// Left pel.
        val = LD_UINT16_LO(pRgb);
        rs = (val>>10) & 0x1F;
        gs = (val>>5) & 0x1F;
        bs = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*rs + iRRGB555_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[0]=255;		// y is only positive
            else pYUYV[0] = (unsigned char)y;

		// Right pel.
        val = LD_UINT16_LO(pRgb+1);
        rs += r = (val>>10) & 0x1F;
        gs += g = (val>>5) & 0x1F;
        bs += b = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*r + iRRGB555_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
             else pYUYV[2] = (unsigned char)y;

		// Average the 2 chr values.
        y = ((iRRGB555_YUVCI2_10*rs + iRRGB555_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB555_YUVCI2_20*rs + iRRGB555_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 2;
        pYUYV += 4;
      } //end for xb...
    }
  }
} //end Convert.


//////////////////////////////////////////////////////////////////


const int32_t iRRGB565_YUVCI2_01 = static_cast<int32_t>(0.5 + 4.04762*RRGB_YUVCI2_01 * (2 << iPrecision));
const int32_t iRRGB565_YUVCI2_11 = static_cast<int32_t>(0.5 + 2*4.04762*RRGB_YUVCI2_11 * (2 << iPrecision));
const int32_t iRRGB565_YUVCI2_21 = static_cast<int32_t>(0.5 + 2*4.04762*RRGB_YUVCI2_21 * (2 << iPrecision));


bool ConvRGB565toYUYV_FixP::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatIn==MEDIASUBTYPE_RGB565) &&
         (VideoFormatOut==MEDIASUBTYPE_YUYV || VideoFormatOut==MEDIASUBTYPE_YUY2);
}


void ConvRGB565toYUYV_FixP::Convert(const void* pBufferIn, void* pBufferOut) const
{
  const uint16_t *pRgb = (const uint16_t*)pBufferIn;
  unsigned char *pYUYV = (unsigned char*)pBufferOut;
	// Y has range 0..255, U & V have range -128..127.
  int32_t y;
  uint32_t r,g,b;
  int16_t rs,gs,bs;
  uint16_t val;

	// Step in 2x2 pel blocks. (4 pels per block).
  int yb = _height;

  if(_flip)
  {
    pRgb += _width * (_height-1);
    while(yb-- > 0)
    {
      int xb = _width >> 1;
      while(xb-- > 0)
      {
		// Left pel.
        val = LD_UINT16_LO(pRgb);
        rs = (val>>11) & 0x1F;
        gs = (val>>5) & 0x3F;
        bs = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*rs + iRRGB565_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[0]=255;		// y is only positive
              else pYUYV[0] = (unsigned char)y;

		// Right pel.
        val = LD_UINT16_LO(pRgb+1);
        rs += r = (val>>11) & 0x1F;
        gs += g = (val>>5) & 0x3F;
        bs += b = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
              else pYUYV[2] = (unsigned char)y;

		// Average the 4 chr values.
        y = ((iRRGB555_YUVCI2_10*rs + iRRGB565_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB555_YUVCI2_20*rs + iRRGB565_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 2;
        pYUYV += 4;
      } //end for xb...
      pRgb -= 2 * _width;
    }
  }
  else
  {
    while(yb-- > 0)
    {
      int xb = _width >> 1;
      while(xb-- > 0)
      {
		// Left pel.
        val = LD_UINT16_LO(pRgb);
        rs = (val>>11) & 0x1F;
        gs = (val>>5) & 0x3F;
        bs = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*rs + iRRGB565_YUVCI2_01*gs + iRRGB555_YUVCI2_02*bs + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[0]=255;		// y is only positive
              else pYUYV[0] = (unsigned char)y;

		// Right pel.
        val = LD_UINT16_LO(pRgb+1);
        rs += r = (val>>11) & 0x1F;
        gs += g = (val>>5) & 0x3F;
        bs += b = val & 0x1F;
        y = ((iRRGB555_YUVCI2_00*r + iRRGB565_YUVCI2_01*g + iRRGB555_YUVCI2_02*b + (1<<iPrecision)) >> (iPrecision + 1));
        if(y>=255) pYUYV[2]=255;		// y is only positive
              else pYUYV[2] = (unsigned char)y;

		// Average the 2 chr values.
        y = ((iRRGB555_YUVCI2_10*rs + iRRGB565_YUVCI2_11*gs + iRRGB555_YUVCI2_12*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[1]=0;
        else if(y>=2*255) pYUYV[1]=255;
        else pYUYV[1] = (unsigned char)(y>>1);

        y = ((iRRGB555_YUVCI2_20*rs + iRRGB565_YUVCI2_21*gs + iRRGB555_YUVCI2_22*bs) >> (iPrecision + 1)) + 2*128 + 1;
        if(y<=0) pYUYV[3]=0;
        else if(y>=2*255) pYUYV[3]=255;
        else pYUYV[3] = (unsigned char)(y>>1);

        pRgb += 2;
        pYUYV += 4;
      } //end for xb...
    }
  }
} //end Convert.

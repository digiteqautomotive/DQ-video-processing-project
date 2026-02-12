/** @file

MODULE:		ConvRGB32toYUV420_FixP

FILE NAME:	ConvRGB32toYUV420_FixP.cpp

DESCRIPTION:	Fixed-point RGB 32 bit to YUV420 colour 
		conversion derived from the RGBtoYUV420Converter base class.

LICENSE:	Software License Agreement (BSD License)

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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "RGB32toYUV420Converter.h"

/*===========================================================================
			Constants
===========================================================================*/

#define iPrecision 12
const int iRRGB24YUVCI2_00 = static_cast<int>(0.5 + RRGB24YUVCI2_00 * (2 << iPrecision));
const int iRRGB24YUVCI2_01 = static_cast<int>(0.5 + RRGB24YUVCI2_01 * (2 << iPrecision));
const int iRRGB24YUVCI2_02 = static_cast<int>(0.5 + RRGB24YUVCI2_02 * (2 << iPrecision));
const int iRRGB24YUVCI2_10 = static_cast<int>(0.5 + RRGB24YUVCI2_10 * (2 << iPrecision));
const int iRRGB24YUVCI2_11 = static_cast<int>(0.5 + RRGB24YUVCI2_11 * (2 << iPrecision));
const int iRRGB24YUVCI2_12 = static_cast<int>(0.5 + RRGB24YUVCI2_12 * (2 << iPrecision));
const int iRRGB24YUVCI2_20 = static_cast<int>(0.5 + RRGB24YUVCI2_20 * (2 << iPrecision));
const int iRRGB24YUVCI2_21 = static_cast<int>(0.5 + RRGB24YUVCI2_21 * (2 << iPrecision));
const int iRRGB24YUVCI2_22 = static_cast<int>(0.5 + RRGB24YUVCI2_22 * (2 << iPrecision));

#define RRGB24YUVCI2_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )
#define RRGB24YUVCI2_RANGECHECK_N128TO127(x) ( (((x) <= 127)&&((x) >= -128))?((x)):( ((x) > 127)?(127):(-128) ) )


/*===========================================================================
				Interface Methods
===========================================================================*/


/** Double precision reference implementation.
The full real matix equation is used. The YUV output is represented with
8 bits per pel and the UV components are adjusted from their -128..127 range
to 0..255.
@param pRgb	: Packed RGB 888 format.
@param pY		: Lum plane.
@param pU		: Chr U plane.
@param pV		: Chr V plane.
@return			: none. */
void ConvRGB32toYUV420FixP::NonFlipConvert(const unsigned char *pRgb, void* pY, void* pU, void* pV) const
{
  yuvType*	py = (yuvType *)pY;
  yuvType*	pu = (yuvType *)pU;
  yuvType*	pv = (yuvType *)pV;

  /// Y have range 0..255, U & V have range -128..127.
  int	y;
  int	rs,gs,bs;
  int	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  //const int xBlks = _width >> 1;
  int yb = _height >> 1;
  while(yb-- > 0)
  {
    int xb = _width >> 1;
    while(xb-- > 0)
    {
	// Top left pel.
      bs = b = (int)pRgb[0];
      gs = g = (int)pRgb[1];
      rs = r = (int)pRgb[2];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) py[0]=255;		// y is only positive
      else py[0] = (yuvType)y;

	// Top right pel.
      bs += b = (int)pRgb[4];
      gs += g = (int)pRgb[5];
      rs += r = (int)pRgb[6];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) py[1]=255;		// y is only positive
      else py[1] = (yuvType)y;

      pRgb += _width*4;
	// Bottom left pel.
      bs += b = (int)pRgb[0];
      gs += g = (int)pRgb[1];
      rs += r = (int)pRgb[2];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) py[_width]=255;	// y is only positive
      else py[_width] = (yuvType)y;

	// Bottom right pel.
      bs += b = (int)pRgb[4];
      gs += g = (int)pRgb[5];
      rs += r = (int)pRgb[6];
      y = ((iRRGB24YUVCI2_00*r + iRRGB24YUVCI2_01*g + iRRGB24YUVCI2_02*b)>> (iPrecision + 1));
      if(y>=255) py[_width+1]=255;	// y is only positive
            else py[_width+1] = (yuvType)y;

	// Average the 4 chr values.
      y = ((iRRGB24YUVCI2_10*rs + iRRGB24YUVCI2_11*gs + iRRGB24YUVCI2_12*bs) >> (iPrecision + 1)) + 4*128 + 2;	// +2 ... rounding.
      if(y<=0) *pu++=0;
      else if(y>=4*255) *pu++=255;
      else *pu++ = (yuvType)(y>>2);

      y = ((iRRGB24YUVCI2_20*rs + iRRGB24YUVCI2_21*gs + iRRGB24YUVCI2_22*bs) >> (iPrecision + 1)) + 4*128 + 2;	// +2 ... rounding.
      if(y<=0) *pv++=0;
      else if(y>=4*255) *pv++=255;
      else *pv++ = (yuvType)(y>>2);

      pRgb -= _width*4 - 8;
      py += 2;
    } //end for xb

    py += _width;
    pRgb += 4*_width;
  }
}//end Convert.


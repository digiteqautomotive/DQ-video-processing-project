/** @file

MODULE:			ConvYUV420_16toBGR24_CCIR601
                        
FILE NAME:		ConvYUV420_16toBGR24_CCIR601.cpp

DESCRIPTION:		Integer implementation of implementation of YUV420 (16 bpp) to RGB 24 bit 
			colour convertion derived from	the YUV420toRGBConverter base 
			class. Assumes that the colour space of the YUV components
			are defined by the CCIR-601 standard.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
==========================================================================*/
#include "ConvYUV420_16toRGB.h"


//  CCIR-601: The defenition assumes limited Y and UV (CbCr) ranges but the inverse
//		  conversion will treat it as full range and only compensate for the shift.
//  r = (y-16) + 1.140(v-128)			~	r*256 = 256*(y-16) + 292*(v-128)
//  g = (y-16) - 0.394(u-128) - 0.581(v-128)	~	g*256 = 256*(y-16) - 101*(u-128) - 149*(v-128)
//  b = (y-16) + 2.033(u-128)			~	b*256 = 256*(y-16) + 520*(u-128)

//#define RYUVRGB24CC_U0	 2.033
//#define RYUVRGB24CC_U1	-0.394
//#define RYUVRGB24CC_V1	-0.581
//#define RYUVRGB24CC_V0	 1.140



/*===========================================================================
	Private Methods.
==========================================================================*/


void ConvYUV420_16toBGR24_CCIR601::FlipConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const
{
int32_t X;
pRgb += 3*_width*(_height-1);

  for(int row=0; row<_height; row++)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const int32_t U = *u++ - 128;
      const int32_t V = *v++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *(y++) - 16*256;
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;

      Y = 256 * *(y++) - 16*256;

      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[3]=255;
      else if(X<0) pRgb[3]=0;
      else pRgb[3] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;

      pRgb += 6;
    }
    pRgb -= 6*_width;
    if((row&1) == 0)
    {
      u -= _width/2;
      v -= _width/2;
    }  
  }
} //end FlipConvert


void ConvYUV420_16toBGR24_CCIR601::NonRotateConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const
{
int32_t X;
const unsigned width3 = _width * 3;

  for(int row=0; row<_height; row+=2)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const int32_t U = *u++ - 128;
      const int32_t V = *v++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *y - 16*256;

      //printf("%u: cc:%.1f ca:%.1f cb:%.1f Y:%.1f; ",row,cc/256.0,ca/256.0,cb/256.0,Y/256.0);

      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;

      Y = 256 * y[1] - 16*256;
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[3]=255;
      else if(X<0) pRgb[3]=0;
      else pRgb[3] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;

      pRgb += width3;

      Y = 256 * y[_width] - 16*256;
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;

      Y = 256 * y[_width+1] - 16*256;
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[3]=255;
      else if(X<0) pRgb[3]=0;
      else pRgb[3] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;

      pRgb = pRgb - width3 + 6;
      y += 2;
    }
    pRgb += width3;
    y += _width;
  }
} //end NonRotateConvert.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

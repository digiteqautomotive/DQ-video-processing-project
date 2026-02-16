/** @file

MODULE:			YUV420toRGB24Converter
                        
FILE NAME:		ConvYUV420toRGB24CReal.cpp

DESCRIPTION:		Integer implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour conversion derived from the YUV420toRGBConverter base 	class.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025, Digiteq Automotive All rights reserved.

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
#include "ConvYUV420toRGB.h"

#include "typedfs.h"


/*===========================================================================
	Private Methods.
==========================================================================*/

void YUV420toRGB24Converter::FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const
{
int32_t X;
const unsigned char *y = (const unsigned char *)pY;
const signed char *u = (const signed char *)pU;
const signed char *v = (const signed char *)pV;
unsigned char *OutData = (unsigned char *)pRgb + 3*_width*(_height-1);

  for(int row=0; row<_height; row++)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *u++ - 128;
      const signed char V = *v++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *(y++);
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) OutData[2]=255;
      else if(X<0) OutData[2]=0;
      else OutData[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[1]=255;
      else if(X<0) OutData[1]=0;
      else OutData[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) OutData[0]=255;
      else if(X<0) OutData[0]=0;
      else OutData[0] = X;

      Y = 256 * *(y++);

      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) OutData[5]=255;
      else if(X<0) OutData[5]=0;
      else OutData[5] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[4]=255;
      else if(X<0) OutData[4]=0;
      else OutData[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) OutData[3]=255;
      else if(X<0) OutData[3]=0;
      else OutData[3] = X;

      OutData += 6;
    }
    OutData -= 6*_width;
    if((row&1) == 0)
    {
      u -= _width/2;
      v -= _width/2;
    }  
  }
} //end FlipConvert


void YUV420toRGB24Converter::NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const
{
int32_t X;
const unsigned width3 = _width * 3;
const unsigned char *y = (const unsigned char *)pY;
const signed char *u = (const signed char *)pU;
const signed char *v = (const signed char *)pV;
unsigned char *OutData = (unsigned char *)pRgb;

  for(int row=0; row<_height; row+=2)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *u++ - 128;
      const signed char V = *v++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *y;
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) OutData[2]=255;
      else if(X<0) OutData[2]=0;
      else OutData[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[1]=255;
      else if(X<0) OutData[1]=0;
      else OutData[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) OutData[0]=255;
      else if(X<0) OutData[0]=0;
      else OutData[0] = X;

      Y = 256 * y[1];
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) OutData[5]=255;
      else if(X<0) OutData[5]=0;
      else OutData[5] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[4]=255;
      else if(X<0) OutData[4]=0;
      else OutData[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) OutData[3]=255;
      else if(X<0) OutData[3]=0;
      else OutData[3] = X;

      OutData += width3;

      Y = 256 * y[_width];
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) OutData[2]=255;
      else if(X<0) OutData[2]=0;
      else OutData[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[1]=255;
      else if(X<0) OutData[1]=0;
      else OutData[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) OutData[0]=255;
      else if(X<0) OutData[0]=0;
      else OutData[0] = X;

      Y = 256 * y[_width+1];
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) OutData[5]=255;
      else if(X<0) OutData[5]=0;
      else OutData[5] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[4]=255;
      else if(X<0) OutData[4]=0;
      else OutData[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) OutData[3]=255;
      else if(X<0) OutData[3]=0;
      else OutData[3] = X;

      OutData = OutData - width3 + 6;
      y += 2;
    }
    OutData += width3;
    y += _width;
  }
} //end NonRotateConvert.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void YUV420toRGB32Converter::FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const
{
int32_t X;
const unsigned char *y = (const unsigned char *)pY;
const signed char *u = (const signed char *)pU;
const signed char *v = (const signed char *)pV;
unsigned char *OutData = pRgb + 4*_width*(_height-1);

  for(int row=0; row<_height; row++)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *u++ - 128;
      const signed char V = *v++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;
      int32_t Y = 256 * *(y++);

      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) OutData[2]=255;
      else if(X<0) OutData[2]=0;
      else OutData[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[1]=255;
      else if(X<0) OutData[1]=0;
      else OutData[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) OutData[0]=255;
      else if(X<0) OutData[0]=0;
      else OutData[0] = X;
      OutData[3] = 0;			// A

      Y = 256 * *(y++);

      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) OutData[6]=255;
      else if(X<0) OutData[6]=0;
      else OutData[6] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[5]=255;
      else if(X<0) OutData[5]=0;
      else OutData[5] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) OutData[4]=255;
      else if(X<0) OutData[4]=0;
      else OutData[4] = X;
      OutData[7] = 0;			// A

      OutData += 8;
    }
    OutData -= 8*_width;
    if((row&1) == 0)
    {
      u -= _width/2;
      v -= _width/2;
    }  
  }
} //end FlipConvert


void YUV420toRGB32Converter::NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const
{
int32_t X;
const unsigned width4 = _width * 4;
const unsigned char *y = (const unsigned char *)pY;
const signed char *u = (const signed char *)pU;
const signed char *v = (const signed char *)pV;
unsigned char *OutData = pRgb;

  for(int row=0; row<_height; row+=2)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *u++ - 128;
      const signed char V = *v++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *y;
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) OutData[2]=255;
      else if(X<0) OutData[2]=0;
      else OutData[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[1]=255;
      else if(X<0) OutData[1]=0;
      else OutData[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) OutData[0]=255;
      else if(X<0) OutData[0]=0;
      else OutData[0] = X;

      OutData[3] = 0;			// A

      Y = 256 * y[1];
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) OutData[6]=255;
      else if(X<0) OutData[6]=0;
      else OutData[6] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[5]=255;
      else if(X<0) OutData[5]=0;
      else OutData[5] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) OutData[4]=255;
      else if(X<0) OutData[4]=0;
      else OutData[4] = X;

      OutData[7] = 0;			// A

      OutData += width4;

      Y = 256 * y[_width];
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) OutData[2]=255;
      else if(X<0) OutData[2]=0;
      else OutData[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[1]=255;
      else if(X<0) OutData[1]=0;
      else OutData[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) OutData[0]=255;
      else if(X<0) OutData[0]=0;
      else OutData[0] = X;

      OutData[3] = 0;			// A

      Y = 256 * y[_width+1];
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) OutData[6]=255;
      else if(X<0) OutData[6]=0;
      else OutData[6] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) OutData[5]=255;
      else if(X<0) OutData[5]=0;
      else OutData[5] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) OutData[4]=255;
      else if(X<0) OutData[4]=0;
      else OutData[4] = X;

      OutData[7] = 0;			// A

      OutData = OutData - width4 + 8;
      y += 2;
    }
    OutData += width4;
    y += _width;
  }
} //end NonRotateConvert.


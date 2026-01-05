/** @file

MODULE				:YUV420toRGB24Converter

FILE NAME			:YUV420toRGB24Converter.cpp

DESCRIPTION			: Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
					colour conversion derived from the YUV420toRGBConverter base 
					class.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025, Digiteq Automotive All rights reserved.

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

==========================================================================*/

#include "RealYUV420toRGB24Converter.h"


extern "C" void YUV_BGR(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount);



/*===========================================================================
	Private Methods.
==========================================================================*/

void YUV420toRGB24Converter::FlipConvert(const void* pY, const void* pU, const void* pV, void* pRgb) const
{
const unsigned char *y = (const unsigned char *)pY;
const signed char *u = (const signed char *)pU;
const signed char *v = (const signed char *)pV;
unsigned char *OutData = (unsigned char *)pRgb + 3*_width*(_height-1);
int row = _height / 2;

  while(row-- > 0)
  {
    YUV_BGR(OutData, y, (const unsigned char *)u, (const unsigned char *)v, _width);
    y += _width;
    OutData -= 3*_width;

    YUV_BGR(OutData, y, (const unsigned char *)u, (const unsigned char *)v, _width);
    y += _width;
    OutData -= 3*_width;

    u += _width/2;
    v += _width/2;
  }

} //end FlipConvert


void YUV420toRGB24Converter::NonRotateConvert(const void* pY, const void* pU, const void* pV, void* pRgb) const
{
const unsigned char *y = (const unsigned char *)pY;
const signed char *u = (const signed char *)pU;
const signed char *v = (const signed char *)pV;
unsigned char *OutData = (unsigned char *)pRgb;
int row = _height / 2;

  while(row-- > 0)
  {
    YUV_BGR(OutData, y, (const unsigned char *)u, (const unsigned char *)v, _width);
    y += _width;
    OutData += 3*_width;

    YUV_BGR(OutData, y, (const unsigned char *)u, (const unsigned char *)v, _width);
    y += _width;
    OutData += 3*_width;

    u += _width/2;
    v += _width/2;
  }
} //end NonRotateConvert.

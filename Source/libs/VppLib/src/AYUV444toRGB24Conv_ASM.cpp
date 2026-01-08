/** @file

MODULE				: AYUV420toRGB24Converter

FILE NAME			: AYUV420toRGB24Converter.cpp

DESCRIPTION			: Implementation of YUV420 (8 bpp) to RGB 24 bit 
				  colour conversion derived from the YUV420toRGBConverter base class.
					  
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

===========================================================================
*/
#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>
#else

#endif
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "AYUV444toRGB24Converter.h"


extern "C"
{
 void ConvertRowAsmAYUV_ARGB24(void *pDst, const void *pSrc, unsigned Pixels);
 void ConvertRowAsmAYUV_ARGB32(void *pDst, const void *pSrc, unsigned Pixels);

 void ConvertRowAsmYV24_RGB24(void *pDst, const void *pSrc, unsigned Pixels);
 void ConvertRowAsmYV24_RGB32(void *pDst, const void *pSrc, unsigned Pixels);
}


/*
===========================================================================
	Private Methods.
===========================================================================
*/

//Note: https://topic.alibabacloud.com/a/about-the-common-ayuv-formats-in-rgb-yuy2-yuyv-yvyu-uyvy-directshow-rgbyuv_8_8_10271129.html
// The ¨AYUV format has an alpha channel, and the YUV component is extracted for each pixel, and the image Data format is as follows:
// A0 Y0 U0 V0 A1 Y1 U1 V1 ... 


void AYUV444toRGB24Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
  if(_flip)
  {
    unsigned y = _height;
    const unsigned w = _width;
    pRgb = (char*)(pRgb) + w*(y-1)*3;
    while(y-- > 0)
    {      
      ConvertRowAsmAYUV_ARGB24(pRgb, pInputBuffer, w);
      pRgb = (char*)(pRgb) - w*3;				// RGB
      pInputBuffer = (char*)(pInputBuffer) + w*4;		// AYUV
    }
  }
  else
      ConvertRowAsmAYUV_ARGB24(pRgb, pInputBuffer, _width*_height);
}


void AYUV444toARGB32Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
  if(_flip)
  {
    unsigned y = _height;
    const unsigned w = _width;
    pRgb = (char*)(pRgb) + w*(y-1)*4;
    while(y-- > 0)
    {      
      ConvertRowAsmAYUV_ARGB32(pRgb, pInputBuffer, w);
      pRgb = (char*)(pRgb) - w*4;
      pInputBuffer = (char*)(pInputBuffer) + w*4;
    }
  }
  else
      ConvertRowAsmAYUV_ARGB32(pRgb, pInputBuffer, _width*_height);
}


/*
void YV24toRGB24Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
  if(_flip)
  {
    unsigned y = _height;
    const unsigned w = _width;
    pRgb = (char*)(pRgb) + w*(y-1)*3;
    while(y-- > 0)
    {      
      ConvertRowAsmYV24_RGB24(pRgb, pInputBuffer, w);
      pRgb = (char*)(pRgb) - w*3;				// RGB
      pInputBuffer = (char*)(pInputBuffer) + w*3;		// AYUV
    }
  }
  else
      ConvertRowAsmYV24_RGB24(pRgb, pInputBuffer, _width*_height);
}


void YV24toRGB32Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
  if(_flip)
  {
    unsigned y = _height;
    const unsigned w = _width;
    pRgb = (char*)(pRgb) + w*(y-1)*4;
    while(y-- > 0)
    {      
      ConvertRowAsmYV24_RGB32(pRgb, pInputBuffer, w);
      pRgb = (char*)(pRgb) - w*4;
      pInputBuffer = (char*)(pInputBuffer) + w*3;
    }
  }
  else
      ConvertRowAsmAYUV_ARGB32(pRgb, pInputBuffer, _width*_height);
}
*/
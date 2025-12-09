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

/*
===========================================================================
	Macros.
===========================================================================
*/
#define RYUVRGB24C_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )


/*
===========================================================================
	Private Methods.
===========================================================================
*/

//Note: https://topic.alibabacloud.com/a/about-the-common-ayuv-formats-in-rgb-yuy2-yuyv-yvyu-uyvy-directshow-rgbyuv_8_8_10271129.html
// The ¨AYUV format has an alpha channel, and the YUV component is extracted for each pixel, and the image Data format is as follows:
// A0 Y0 U0 V0 A1 Y1 U1 V1 ... 


#ifndef int32_t
 #define int32_t int
#endif

void AYUV444toRGB24Converter::Convert(const void* pInputBuffer, void* pRgb)
{
unsigned char * Out = (unsigned char *)pRgb;
const unsigned char * In = (const unsigned char *)pInputBuffer;
int numberOfPixels = _width * _height;
int32_t X;

  while(numberOfPixels-->0)
  {
    const int32_t Y = 256 * In[1];
    const signed char U = In[2] - 128; 
    const signed char V = In[3] - 128;
    

    X = (Y + 454*U) >> 8;	// B Y + 1.772	Y + 2.034*U;
    if(X>255) Out[0]=255;
    else if(X<0) Out[0]=0;
    else Out[0] = X;
    X = (Y -88*U - 183*V) >> 8; // G Y - 0.344*U - 0.714 *V;   Y - 0.397*U - 0.58*V;
    if(X>255) Out[1]=255;
    else if(X<0) Out[1]=0;
    else Out[1] = X;
    X = (Y + 359*V) >> 8;	// R Y + 1.402*V;   Y + 1.137*V;
    if(X>255) Out[2]=255;
    else if(X<0) Out[2]=0;
    else Out[2] = X;

    Out+=3;
    In+=4;
  }
}

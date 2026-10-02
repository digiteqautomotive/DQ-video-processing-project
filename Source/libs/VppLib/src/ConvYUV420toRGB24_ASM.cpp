/** @file

MODULE:			YUV420toRGB24Converter

FILE NAME:		YUV420toRGB24Converter.cpp

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour conversion derived from the YUV420toRGBConverter base 
			class.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.

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
#include "ConvYUV420toRGB_ASM.h"
#include <uuids.h>


extern "C"
{
 void YUV_RGB(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount);
 void PelYUV_RGB24(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB32(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount);
 void PelYUV_RGB24_CMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB32_CMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB24_CMOV_SSE(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB_Flip(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB24_FlipCMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB32_Flip(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
 void PelYUV_RGB32_FlipCMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount);
}

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


/*===========================================================================
	Private Methods.
==========================================================================*/


bool ConvYUV420toRGB24_ASM::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatIn==MEDIASUBTYPE_YUV420P_S || VideoFormatIn==MEDIASUBTYPE_I420 || VideoFormatIn==MEDIASUBTYPE_YV12) &&
         (VideoFormatOut==MEDIASUBTYPE_RGB24);
}


void ConvYUV420toRGB24_ASM::FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
pRgb += 3*_width*(_height-1);
int row = _height / 2;

  while(row-- > 0)
  {
    PelYUV_RGB24_FlipCMOV(pRgb,  pY,  pU, pV, _width);
    pRgb -= 6*_width;
    pY += 2*_width;
    pU += _width/2;
    pV += _width/2;
  }

} //end FlipConvert


void ConvYUV420toRGB24_ASM::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
int row = _height / 2;  

  while(row-- > 0)
  {
    //PelYUV_RGB24(pRgb,  pY,  pU, pV, _width);
    PelYUV_RGB24_CMOV(pRgb,  pY,  pU, pV, _width);
    pY += 2*_width;
    pRgb += 6*_width;
    pU += _width/2;
    pV += _width/2;
  }
} //end NonRotateConvert.


#if defined(USE_SSE) && defined(_M_IX86)
void ConvYUV420toRGB24_ASM_SSE2::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
int row = _height / 2;  

  while(row-- > 0)
  {
    PelYUV_RGB24_CMOV_SSE(pRgb,  pY,  pU, pV, _width);
    pY += 2*_width;
    pRgb += 6*_width;
    pU += _width/2;
    pV += _width/2;
  }
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////

bool ConvYUV420toRGB32_ASM::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatIn==MEDIASUBTYPE_YUV420P_S || VideoFormatIn==MEDIASUBTYPE_I420 || VideoFormatIn==MEDIASUBTYPE_YV12) &&
         (VideoFormatOut==MEDIASUBTYPE_RGB32 || VideoFormatOut==MEDIASUBTYPE_ARGB32);
}


void ConvYUV420toRGB32_ASM::FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
pRgb += 4*_width*(_height-1);
int row = _height / 2;

  while(row-- > 0)
  {
    PelYUV_RGB32_FlipCMOV(pRgb,  pY,  pU, pV, _width);
    pRgb -= 8*_width;
    pY += 2*_width;
    pU += _width/2;
    pV += _width/2;
  }

} //end FlipConvert


void ConvYUV420toRGB32_ASM::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
int row = _height / 2;  

  while(row-- > 0)
  {
    //PelYUV_RGB32(pRgb,  y,  u, v, _width);
    PelYUV_RGB32_CMOV(pRgb,  pY,  pU, pV, _width);
    pY += 2*_width;
    pRgb += 8*_width;
    pU += _width/2;
    pV += _width/2;
  }
} //end NonRotateConvert.
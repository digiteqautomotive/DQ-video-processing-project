/** @file

MODULE:			YUV420toRGB24Converter

FILE NAME:		ConvYUV420toRGB24.h

DESCRIPTION:		Fixed point implementation of YUV420 (16 bpp) to RGB 24 bit 
			colour convertion derived from the YUV420toRGBConverter base class.
					  
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive programmed by Jaroslav Fojtik		All rights reserved.

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
===========================================================================*/
#ifndef _YUV420_16_TORGB24_CONVERTER_H
#define _YUV420_16_TORGB24_CONVERTER_H

#include "ConvYUVtoRGB.h"
#include "typedfs.h"


class YUV420_16toRGB_Converter: public CVideoConverter
{
public:
	/// Construction and destruction.
	YUV420_16toRGB_Converter(void)		{}
 	YUV420_16toRGB_Converter(int width, int height): CVideoConverter(width,height)	{}

		// Interface.
	virtual void Convert(const void* pYUV, void* pRgb) const
        {
          const unsigned BlkYsz = _width*_height;
          const int16_t * const pU = (const int16_t*)pYUV + BlkYsz;
          const int16_t * const pV = pU + BlkYsz/4;
          if(_flip) FlipConvert((int16_t*)pYUV, pU, pV, (unsigned char*)pRgb);
               else NonRotateConvert((int16_t*)pYUV, pU, pV, (unsigned char*)pRgb);
        } //end Convert.

        void Convert(const void* pY, const void* pU, const void* pV, void* pRgb)
        {
          if(_flip) FlipConvert((const int16_t*)pY, (const int16_t*)pU, (const int16_t*)pV, (unsigned char*)pRgb);
          else NonRotateConvert((const int16_t*)pY, (const int16_t*)pU, (const int16_t*)pV, (unsigned char*)pRgb);
        }

	virtual bool isVideoFormatSupported(const GUID &,const GUID &) const {return false;}

		// Member interface.
protected:
	virtual void FlipConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const = 0;
	virtual void NonRotateConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const = 0;
}; //end RGBtoYUV420_16Converter.


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base class. */
class ConvYUV420_16toBGR24_CCIR601: public YUV420_16toRGB_Converter
{
public:
	// Construction and destruction.
  ConvYUV420_16toBGR24_CCIR601(void) {}
  ConvYUV420_16toBGR24_CCIR601(int width, int height):  YUV420_16toRGB_Converter(width,height) {}

	// Interface.
  void FlipConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const;
  void NonRotateConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24.


class ConvYUV420_16toBGR24: public YUV420_16toRGB_Converter
{
public:
	// Construction and destruction.
  ConvYUV420_16toBGR24(void) {}
  ConvYUV420_16toBGR24(int width, int height): YUV420_16toRGB_Converter(width,height) {}

	// Interface.
  void FlipConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const;
  void NonRotateConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24.


#endif	// _YUV420_TORGB24_CONVERTER_H

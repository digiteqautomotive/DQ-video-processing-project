/** @file

MODULE:			YUV420toRGB24Converter

FILE NAME:		ConvYUV420toRGB24.h

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour convertion derived from the YUV420toRGBConverter base 
			class.
					  
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
Copyright (c) 2025-2026, Digiteq Automotive		All rights reserved.

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
#ifndef _YUV420_TORGB24_CONVERTER_H
#define _YUV420_TORGB24_CONVERTER_H

#include "ConvYUVtoRGB.h"


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base class. */
class ConvYUV420toRGB24: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  ConvYUV420toRGB24(void) {}
  ConvYUV420toRGB24(int width, int height): YUV420toRGBConverter(width,height) {}

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
	virtual void FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const;
	virtual void NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24.


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base class. */
class YUV420toRGB32Converter: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  YUV420toRGB32Converter(void) {}
  YUV420toRGB32Converter(int width, int height): YUV420toRGBConverter(width,height) {}

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
	virtual void FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const;
	virtual void NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24.



/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base class. */
class ConvYUV420toBGR24_CCIR601: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  ConvYUV420toBGR24_CCIR601(void) {}
  ConvYUV420toBGR24_CCIR601(int width, int height): YUV420toRGBConverter(width,height) {}

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
  virtual void FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const;
  virtual void NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24.


#endif	// _YUV420_TORGB24_CONVERTER_H

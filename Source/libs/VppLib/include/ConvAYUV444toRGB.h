/** @file

MODULE:			ConvAYUV444toRGB

FILE NAME:		ConvAYUV444toRGB.h

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 & 32 bit 
			colour convertion derived from the CVideoConverter base class.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.

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
==========================================================================*/
#ifndef _ConvAYUV444toRGB24_H
#define _ConvAYUV444toRGB24_H
#include <DirectShow/stdafx.h>
#include "ConvYUVtoRGB.h"


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the CVideoConverter base class. */
class ConvAYUV444toRGB24: public CVideoConverter
{
public:
	 // Construction and destruction.
  ConvAYUV444toRGB24(void) { }
  ConvAYUV444toRGB24(int width, int height): CVideoConverter(width,height) {}

  virtual ~ConvAYUV444toRGB24(void) {}
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

	// Interface.
  virtual void Convert(const void* pInputBuffer, void* pRgb) const;

};//end ConvAYUV444toRGB24.


/** \ingroup ImageLib
* Integer implementation of AYUV444 (32 bpp) to RGB 32 bit 
* colour convertion derived from the CVideoConverter base class. */
class AYUV444toARGB32Converter: public CVideoConverter
{
public:
	// Construction and destruction.
  AYUV444toARGB32Converter(void) {}
  AYUV444toARGB32Converter(int width, int height): CVideoConverter(width,height) {}

	// Interface
  //virtual bool isVideoFormatSupported(const GUID &VideoFormat) const;
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;
  virtual void Convert(const void* pInputBuffer, void* pRgb) const;

}; //end AYUV444toARGB32Converter.


//////////////////////////////////////////////////////////////


class ConvYV24toRGB24: public CVideoConverter
{
public:
	// Construction and destruction.
  ConvYV24toRGB24(void) {}
  ConvYV24toRGB24(int width, int height): CVideoConverter(width,height) {}

	// Interface
  virtual void Convert(const void* pInputBuffer, void* pRgb) const;
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;
}; //end ConvYV24toRGB24.


class ConvYV24toRGB32: public CVideoConverter
{
public:
  // Construction and destruction.
  ConvYV24toRGB32(void) {}
  ConvYV24toRGB32(int width, int height): CVideoConverter(width,height) {}  

  // Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormat) const;
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

  virtual void Convert(const void* pInputBuffer, void* pRgb) const;

};	//end ConvYV24toRGB24.


#endif	// _ConvAYUV444toRGB24_H

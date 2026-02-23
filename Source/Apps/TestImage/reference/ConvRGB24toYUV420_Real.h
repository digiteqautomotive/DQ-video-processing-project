/** @file

MODULE:			ConvRGB24toYUV420Real

FILE NAME:		ConvRGB24toYUV420_Real.h

DESCRIPTION:		Double precision floating point RGB 24 bit to YUV420 colour 
			convertion derived from the RGBtoYUV420Converter base class.
			Use this implementation as the reference.
	  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.

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
#ifndef _REALRGB24TOYUV420CONVERTER_H
#define _REALRGB24TOYUV420CONVERTER_H

#pragma once

#include "ConvRGBtoYUV420.h"

/** \ingroup ImageLib
 * Double precision floating point RGB 24 bit to YUV420 colour 
 * convertion derived from the RGBtoYUV420Converter base class.
 * Use this implementation as the reference. */
class ConvRGB24toYUV420Real: public RGBtoYUV420Converter
{
public:
    /// Construction and destruction.
    ConvRGB24toYUV420Real(void) { }
    ConvRGB24toYUV420Real(int width, int height): RGBtoYUV420Converter(width,height) {}

    bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
	/// Interface.
  virtual void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
  virtual void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
}; //end _REALRGB24TOYUV420CONVERTERIMPL2_H.


#endif

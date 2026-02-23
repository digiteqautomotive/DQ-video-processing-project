/** @file

MODULE:			YUV420toRGB24Converter

FILE NAME:		ConvYUV420toRGB24.h

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit
			colour convertion derived from the YUV420toRGBConverter base
			class.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive		All rights reserved.
===========================================================================*/
#ifndef _CONV_YUV420_TO_RGB_ASM_H_
#define _CONV_YUV420_TO_RGB_ASM_H_

#include "ConvYUVtoRGB.h"


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base 
* class. */
class ConvYUV420toRGB24_ASM: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  ConvYUV420toRGB24_ASM(void)	{}
  ConvYUV420toRGB24_ASM(int width, int height): YUV420toRGBConverter(width,height) {}

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
  virtual void FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
  virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24_ASM.


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base 
* class. */
class ConvYUV420toRGB32_ASM: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  ConvYUV420toRGB32_ASM(void) { }
  ConvYUV420toRGB32_ASM(int width, int height): YUV420toRGBConverter(width,height) { }

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
  virtual void FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
  virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB32_ASM.


#endif	// _REALYUV420TORGB24CONVERTER_H

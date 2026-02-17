/** @file

MODULE:			YUV420toRGB24Converter

FILE NAME:		ConvYUV420toRGB24.h

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour convertion derived from the YUV420toRGBConverter base 
			class.
					  
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive		All rights reserved.
===========================================================================*/
#ifndef _ASM_YUV420TORGB24CONVERTER_H
#define _ASM_YUV420TORGB24CONVERTER_H

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

protected:
  virtual void FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
  virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
}; //end ConvYUV420toRGB24_ASM.


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base 
* class. */
/*
class YUV420toRGB32Converter_ASM: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  YUV420toRGB32Converter(void) { }
  YUV420toRGB32Converter(int width, int height): YUV420toRGBConverter(width,height) { }

	// Interface.

protected:
  virtual void FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
  virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
}; //end YUV420toRGB24Converter.
*/

#endif	// _REALYUV420TORGB24CONVERTER_H

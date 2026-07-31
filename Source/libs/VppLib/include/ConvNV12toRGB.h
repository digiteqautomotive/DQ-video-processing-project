/** @file

MODULE:			ConvNV12toRGBConverter

FILE NAME:		ConvNV12toRGB.h 

DESCRIPTION:		Implementation of NV12 (8 bpp) to RGB 24 bit 
			colour convertion derived from the YUV420toRGBConverter base 
			class.					  
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive		All rights reserved.
===========================================================================*/
#ifndef _NV12TORGB24CONVERTER_H
#define _NV12TORGB24CONVERTER_H

#include "ConvYUVtoRGB.h"

/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base 
* class. */
class ConvNV12toRGB24: public CVideoConverter
{
public:
  // Construction and destruction.
  ConvNV12toRGB24(void) {}
  ConvNV12toRGB24(int width, int height): CVideoConverter(width,height) {}

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;
  virtual void Convert(const void* pInputBuffer, void *pBufferInOut) const;
}; //end ConvNV12toRGB24.


/** \ingroup ImageLib
* Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
* colour convertion derived from the YUV420toRGBConverter base 
* class. */
class ConvNV12toRGB32: public CVideoConverter
{
public:
  // Construction and destruction.
  ConvNV12toRGB32(void) { }
  ConvNV12toRGB32(int width, int height): CVideoConverter(width,height) {}

	// Interface.
  virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;
  virtual void Convert(const void* pInputBuffer, void* pBufferInOut) const;
}; //end ConvNV12toRGB32.


#endif	// _NV12TORGB24CONVERTER_H

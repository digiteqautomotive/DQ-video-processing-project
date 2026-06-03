/** @file

MODULE:		RGB555toYUV420Converter

FILE NAME:	RGB555toYUV420Converter.h

DESCRIPTION:	Fixed-point RGB 555 bit to YUV420 colour 
		conversion derived from the RGBtoYUV420Converter base class.
		Use this implementation as the reference.
	  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik  all rights resevered
===========================================================================*/
#pragma once

#include "ConvRGBtoYUV420.h"


/** \ingroup ImageLib
 * Fixed-point RGB 555 bit to YUV420 colour 
 * conversion derived from the RGBtoYUV420Converter base class.
 * Use this implementation as the reference. */
class ConvRGB555toYUV420FixP: public RGBtoYUV420Converter
{
public:
	// Construction and destruction.
    ConvRGB555toYUV420FixP(void) {}
    ConvRGB555toYUV420FixP(int width, int height): RGBtoYUV420Converter(width,height) {}

    virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
	// Interface.
    virtual void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
    virtual void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
};


class ConvRGB565toYUV420FixP: public RGBtoYUV420Converter
{
public:
	// Construction and destruction.
    ConvRGB565toYUV420FixP(void) {}
    ConvRGB565toYUV420FixP(int width, int height): RGBtoYUV420Converter(width,height) {}

    virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
	// Interface.
    virtual void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
    virtual void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
};

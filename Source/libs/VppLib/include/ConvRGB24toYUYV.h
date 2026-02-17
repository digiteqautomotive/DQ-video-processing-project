/** @file

MODULE:		RGB24toYUV420Converter

FILE NAME:	RGB24toYUV420Converter.h

DESCRIPTION:	Fixed-point RGB 24 bit to YUV420 colour 
		conversion derived from the RGBtoYUV420Converter base class.
		Use this implementation as the reference.
	  
LICENSE: Software License Agreement (BSD License)

(c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik  all rights resevered
===========================================================================*/
#pragma once

#include "ConvRGBtoYUV420.h"


/** \ingroup ImageLib
 * Fixed-point RGB 24 bit to YUV420 colour 
 * conversion derived from the RGBtoYUV420Converter base class.
 * Use this implementation as the reference. */
class ConvRGB24toYUYV_FixP: public CVideoConverter
{
public:
	// Construction and destruction.
    ConvRGB24toYUYV_FixP(void) {}
    ConvRGB24toYUYV_FixP(int width, int height): RGBtoYUV420Converter(width,height) {}

    virtual void Convert(const void* pBufferIn, void* pRgb) const;

    virtual bool isInputVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;
};

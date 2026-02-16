/** @file

MODULE:			ConvRGB24toYUV420_LUT

FILE NAME:		ConvRGB24toYUV420_LUT.h

DESCRIPTION:		Lookup-table-based  RGB 24 bit to YUV420 colour 
			conversion derived from the RGBtoYUV420Converter base class.
			Use this implementation as the reference.
	  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#pragma once

#include "ConvRGBtoYUV420.h"

#include "typedfs.h"


/** \ingroup ImageLib
 * Lookup-table based floating point RGB 24 bit to YUV420 colour 
 * conversion derived from the RGBtoYUV420Converter base class.
 * Use this implementation as the reference. */
class ConvRGB24toYUV42_fixp_LUT: public RGBtoYUV420Converter
{
public:
	// Construction and destruction.
    ConvRGB24toYUV42_fixp_LUT(void) { initLookupTable(); }
    ConvRGB24toYUV42_fixp_LUT(int width, int height): RGBtoYUV420Converter(width,height) { initLookupTable(); }

	// Interface
    bool isInputVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

protected:
    void NonFlipConvert(const unsigned char *pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
    void FlipConvert(const unsigned char *pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;


    void initLookupTable(void);
			// LUT consumes 9KB
    int32_t m_RRGB_YUVCI2_00[256];
    int32_t m_RRGB_YUVCI2_01[256];
    int32_t m_RRGB_YUVCI2_02[256];
    int32_t m_RRGB_YUVCI2_10[256];
    int32_t m_RRGB_YUVCI2_11[256];
    int32_t m_RRGB_YUVCI2_12[256];
    int32_t m_RRGB_YUVCI2_20[256];
    int32_t m_RRGB_YUVCI2_21[256];
    int32_t m_RRGB_YUVCI2_22[256];
};

/** @file

MODULE						: RealRGB24toYUV420CCIR601Converter

TAG								: RRGB24YUVCC

FILE NAME					: RealRGB24toYUV420CCIR601Converter.h

DESCRIPTION				: Double precision floating point RGB 24 bit to YUV420 colour 
										convertion derived from the RGBtoYUV420Converter base class.
										Implements the CCIR-601 colour conversion process. Use this 
										implementation as the reference.

REVISION HISTORY	:
									: 

COPYRIGHT					: 

RESTRICTIONS			: 
===========================================================================
*/
#ifndef _REALRGB24TOYUV420CCIR601CONVERTER_H
#define _REALRGB24TOYUV420CCIR601CONVERTER_H

#pragma once

#include "ConvRGBtoYUV420.h"

/*===========================================================================
						Class definition
===========================================================================*/

class RealRGB24toYUV420CCIR601Converter: public RGBtoYUV420Converter
{
public:
		// Construction and destruction.
	RealRGB24toYUV420CCIR601Converter(void) { }
	RealRGB24toYUV420CCIR601Converter(int width, int height): RGBtoYUV420Converter(width,height) {}

		// Interface.
	void Convert(const unsigned char *Rgb, void* pY, void* pU, void* pV) const;
        virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

private:
	virtual void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
	virtual void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;	

};//end _REALRGB24TOYUV420CCIR601CONVERTER_H.


#endif

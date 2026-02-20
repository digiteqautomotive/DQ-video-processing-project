/** @file

MODULE:			RealYUV420toRGB24CCIR601Converter

TAG:			RYUVRGB24CC

FILE NAME:		RealYUV420toRGB24CCIR601Converter.h

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour convertion derived from	the YUV420toRGBConverter base 
			class. Assumes that the colour space of the YUV components
			are defined by the CCIR-601 standard.

REVISION HISTORY:

COPYRIGHT:

RESTRICTIONS:
===========================================================================*/
#ifndef _REALYUV420TORGB24CCIR601CONVERTER_H
#define _REALYUV420TORGB24CCIR601CONVERTER_H

#include "ConvYUVtoRGB.h"


/*===========================================================================
			Class definition
===========================================================================*/


class RealYUV420toRGB24CCIR601Converter: public YUV420toRGBConverter
{
public:
		// Construction and destruction.
	RealYUV420toRGB24CCIR601Converter(void) { }
	RealYUV420toRGB24CCIR601Converter(int width, int height): YUV420toRGBConverter(width,height) { }

		// Interface.
protected:
	virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
	virtual void RotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;

}; //end RealYUV420toRGB24CCIR601Converter.


#endif	// _REALYUV420TORGB24CCIR601CONVERTER_H

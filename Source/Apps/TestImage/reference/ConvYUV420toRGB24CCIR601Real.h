/** @file

MODULE:			RealYUV420toBGR24CCIR601Converter

TAG:			RYUVRGB24CC

FILE NAME:		RealYUV420toBGR24CCIR601Converter.h

DESCRIPTION:		Floating point implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour convertion derived from	the YUV420toRGBConverter base 
			class. Assumes that the colour space of the YUV components
			are defined by the CCIR-601 standard.

REVISION HISTORY:

COPYRIGHT:

RESTRICTIONS:
===========================================================================*/
#ifndef _REALYUV420TOBGR24CCIR601CONVERTER_H
#define _REALYUV420TOBGR24CCIR601CONVERTER_H

#include "ConvYUVtoRGB.h"


/*===========================================================================
			Class definition
===========================================================================*/


class RealYUV420toBGR24CCIR601Converter: public YUV420toRGBConverter
{
public:
		// Construction and destruction.
	RealYUV420toBGR24CCIR601Converter(void) { }
	RealYUV420toBGR24CCIR601Converter(int width, int height): YUV420toRGBConverter(width,height) {}

        virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const;

		// Interface.
protected:
	virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
	virtual void RotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
        virtual void FlipConvert(const void *,const void *,const void *,unsigned char *) const {/* NOT implemented*/}

}; //end RealYUV420toBGR24CCIR601Converter.


#endif	// _REALYUV420TOBGR24CCIR601CONVERTER_H

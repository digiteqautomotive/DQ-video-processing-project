/** @file

MODULE				: RGBtoYUV420Converter

TAG						: RGBYUVC

FILE NAME			: RGBtoYUV420Converter.h

DESCRIPTION		: Colour convertions to YUV420 are required on the input of
								all video codecs. For embedded applications only some
								combinations of colour depths are required. This class is 
								the base class defining the minimum interface	and 
								properties for all derived classes. The conversion method
								is pure virtual and must be implemented by derived classes.

COPYRIGHT			:	(c)CSIR 2007-2010 all rights resevered

LICENSE				: Software License Agreement (BSD License)

RESTRICTIONS	: Redistribution and use in source and binary forms, with or without 
								modification, are permitted provided that the following conditions 
								are met:

								* Redistributions of source code must retain the above copyright notice, 
								this list of conditions and the following disclaimer.
								* Redistributions in binary form must reproduce the above copyright notice, 
								this list of conditions and the following disclaimer in the documentation 
								and/or other materials provided with the distribution.
								* Neither the name of the CSIR nor the names of its contributors may be used 
								to endorse or promote products derived from this software without specific 
								prior written permission.

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
#ifndef _RGBTOYUV420CONVERTER_H
#define _RGBTOYUV420CONVERTER_H

#include "YUVtoRGBConverter.h"


#ifdef _BUILD_FOR_SHORT
	typedef short yuvType;
#else
	typedef signed char yuvType;
#endif


/*===========================================================================
			Constants
===========================================================================*/
#define RRGB24YUVCI2_00		 0.299f
#define RRGB24YUVCI2_01		 0.587f
#define RRGB24YUVCI2_02		 0.114f
#define RRGB24YUVCI2_10		-0.147f
#define RRGB24YUVCI2_11		-0.289f
#define RRGB24YUVCI2_12		 0.436f
#define RRGB24YUVCI2_20		 0.615f
#define RRGB24YUVCI2_21		-0.515f
#define RRGB24YUVCI2_22		-0.100f


/*===========================================================================
		Class definition
===========================================================================*/
class RGBtoYUV420Converter: public BaseVideoConverter
{
public:
	/// Construction and destruction.
	RGBtoYUV420Converter(void) {_width = 0; _height = 0; _flip = false; }
 	RGBtoYUV420Converter(int width, int height) {_width = width; _height = height; _flip = false;}
	virtual ~RGBtoYUV420Converter(void) {}

	/// Interface.
	virtual void Convert(const void* pRgb, void* pYUV) const
        {
          const unsigned BlkYsz = _width*_height;
          unsigned char * const pU = (unsigned char*)pYUV + BlkYsz;
          unsigned char * const pV = pU + BlkYsz/4;
          if(_flip) FlipConvert((const unsigned char*)pRgb, (unsigned char*)pYUV, pU, pV);
               else NonFlipConvert((const unsigned char*)pRgb, (unsigned char*)pYUV, pU, pV);
        }//end Convert.
        virtual void Convert(const void* pRgb, void* pY, void* pU, void* pV)
        {
          if(_flip) FlipConvert((const unsigned char*)pRgb, (unsigned char*)pY, (unsigned char*)pU, (unsigned char*)pV);
          else NonFlipConvert((const unsigned char*)pRgb, (unsigned char*)pY, (unsigned char*)pU, (unsigned char*)pV);
        }
	virtual void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const = 0;
	virtual void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const = 0;

	/// Member interface.
	int GetWidth(void) const		{ return(_width); }
	int GetHeight(void) const		{ return(_height); }
	bool GetFlip() const { return _flip; }

	void SetDimensions(int width, int height)	{_width = width; _height = height;}
	void SetFlip(bool flip) { _flip = flip; }

protected:
	/// Members.
	int _width;
	int _height;
	bool _flip;
}; //end RGBtoYUV420Converter.

#endif	// _RGBTOYUV420CONVERTER_H

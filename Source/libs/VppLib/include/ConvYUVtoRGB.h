/** @file

MODULE:		YUVtoRGBConverter

TAG:		YUVRGBC

FILE NAME:	ConvYUVtoRGB.h

DESCRIPTION:	Colour convertions are required on the output of
		all video codecs. For embedded applications only some
		combinations of colour depths and inversions are required.
		This class is the base class defining the minimum interface
		and properties for all derived classes. 

COPYRIGHT:	(c) 2007-2010 CSIR
		(c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik  all rights resevered
			
LICENSE:	Software License Agreement (BSD License)

RESTRICTIONS:	Redistribution and use in source and binary forms, with or without 
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
#ifndef __YUV_TO_RGBCONVERTER_H__
#define __YUV_TO_RGBCONVERTER_H__


#include <Guiddef.h>


/*===========================================================================
  Class definition.
===========================================================================*/

class BaseVideoConverter
{
public:
	/// Interface.
	virtual void Convert(const void* pInputBuffer, void* pRgb) const = 0;

	//virtual ~BaseVideoConverter() {}
};


class CVideoConverter: public BaseVideoConverter
{
public:
	/// Construction and destruction.
	CVideoConverter(void): _flip(false), _width(0), _height(0) {}
	CVideoConverter(int width, int height);
	virtual ~CVideoConverter() {}

	/// Interface.
	virtual void Convert(const void* pInputBuffer, void* pRgb) const = 0;
        virtual bool isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const = 0;

	/// Member interface.
	unsigned GetWidth(void) const		{return(_width);}
	unsigned GetHeight(void) const		{return(_height);}
	bool GetFlip(void) const		{return _flip;}
	
	void SetDimensions(int width, int height);
	void SetFlip(bool flip)			{_flip = flip;}

protected:
		// Members
	unsigned _width;
	unsigned _height;	

	/// Windows bitmaps are bottom to top, standard YUV formats are top to bottom
	/// for our YUV to be compatible with linux we need to flip the image
	bool _flip;
}; //end YUVtoRGBConverter.


class YUV420toRGBConverter: public CVideoConverter
{
public:
    YUV420toRGBConverter(void) {}
    YUV420toRGBConverter(int width, int height): CVideoConverter(width,height) {}

    virtual void Convert(const void* pBufferIn, void* pRgb) const;
    void Convert(const void* pY, const void* pU, const void* pV, unsigned char* pRgb) const
    {if (_flip) FlipConvert(pY, pU, pV, (unsigned char*)pRgb);
     else NonRotateConvert(pY, pU, pV, (unsigned char*)pRgb);}

    virtual void FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char* pRgb) const = 0;
    virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char* pRgb) const = 0;    
};


#endif	// __YUV_TO_RGBCONVERTER_H__

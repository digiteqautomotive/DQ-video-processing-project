/** @file

MODULE				: AYUV420toRGB24Converter

FILE NAME			: AYUV420toRGB24Converter.cpp

DESCRIPTION			: Implementation of YUV420 (8 bpp) to RGB 24 bit 
				  colour conversion derived from the YUV420toRGBConverter base class.
					  
LICENSE: Software License Agreement (BSD License)

	Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.
===========================================================================*/
#include <stdlib.h>
#include "ConvYUVtoRGB.h"


CVideoConverter::CVideoConverter(int width, int height):
	    _flip(false), _width(labs(width)), _height(labs(height))
{}


void CVideoConverter::SetDimensions(int width, int height)
{
  _width = labs(width);
  _height = labs(height);
}


void YUV420toRGBConverter::Convert(const void* pBufferIn, void* pRgb) const 
{    
	//set the Y, U and V pointers to point to the correct positions within the byte array
  const unsigned nInPixelCount = _width *_height;
  const unsigned char *pU = (const unsigned char *)pBufferIn + nInPixelCount;
  const unsigned char *pV = pU + nInPixelCount/4;
        // let invert take precedence, not catering for both
  if(_flip) 
      FlipConvert(pBufferIn, pU, pV, (unsigned char*)pRgb);
  else				
      NonRotateConvert(pBufferIn, pU, pV, (unsigned char*)pRgb);
}  

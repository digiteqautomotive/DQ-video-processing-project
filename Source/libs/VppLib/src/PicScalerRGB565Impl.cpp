/** @file
MODULE				: PicScalerRGB565Impl

FILE NAME			: PicScalerRGB565Impl.cpp

DESCRIPTION			: An RGB565 implementation derived from the general
					PicScalerBase() class. Scale a packed RGB565 image
					to the dimensions of another packed RGB565 image.
					Note that the upscaling is limited to 2x either 
					dimension.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

https://learn.microsoft.com/en-us/windows/win32/directshow/working-with-16-bit-rgb
===========================================================================*/
#include <stdlib.h>

#include "PicScalerRGB565Impl.h"

#include <uuids.h>


#ifndef uint16_t
  #define uint16_t unsigned short
#endif


/*===========================================================================
	Interface Methods.
===========================================================================*/

bool PicScalerRGB565Impl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_RGB565;
}


/** Scale the size of the input image.
Scale the input image from its dimensions to that of the output image. No
memory size checking is done and is delegated to the calling process.
@param pOutImg	: Packed RGBA 8888 format main base image.
@param pInImg		: Packed RGBA 8888 format smaller sub image.
@return					: 0 = failed, 1 = success. */
int PicScalerRGB565Impl::Scale(void* pOutImg, const void* pInImg, bool VFlip)
{
  if(pOutImg==NULL || pInImg==NULL || _widthIn==0 || _heightIn==0)
      return(0);

  const uint16_t*	pSrc	= (const uint16_t*)pInImg;
  uint16_t*	pDst	= (uint16_t*)pOutImg;
	
  int x, y, posx, posy, i;
  int accuX, accuY;
  uint16_t val;

  accuY = -1;
  posy = 0;
  y = _heightOut;
  while(y-- > 0)
  {
    accuY += _heightIn;				// DDA integer only algorithm
    posy += accuY / _heightOut;
    accuY = accuY % _heightOut;

    const int pRow[3] = {						// Calculate row starts only once per row
			((posy==0) ? 0 : (_widthIn*(posy-1))),
			_widthIn*posy,
			((posy+1>=_heightIn) ? (_widthIn*(_heightIn-1)) : (_widthIn*(posy+1))) };

   accuX = -1;
   posx = 0;
   for(x = 0; x < _widthOut; x++)
   {
	/// Apply a weighted 3x3 FIR filter.
     unsigned b = 8;			// rounding offset +8
     unsigned g = 8;
     unsigned r = 8;

     accuX += _widthIn;			// DDA integer only algorithm
     posx += accuX / _widthOut;
     accuX = accuX % _widthOut;

     for(i=0; i<=2; i++)
     {				
	int aii = pRow[i] + ((posx==0) ? (0) : (posx-1));
        val = pSrc[aii];
	b += val & 0x1F;		//  RRRR RGGG GGGB BBBBB
	g += (val>>5) & 0x3F;
	r += val >> 11;
	//a += (int)(*(pSrc + (aii+3)));	// do not filter missing 'a' compound

	if(posx>0) val=pSrc[++aii];
	if(i==1)
	{
	  b += 8 * (val & 0x1F);
	  g += 8 * ((val>>5) & 0x3F);
	  r += 8 * (val >> 11);
	}
	else
	{
	  b += val & 0x1F;
	  g += (val>>5) & 0x3F;
	  r += val >> 11;
	}                                

	if(posx+1 < _widthIn) val=pSrc[++aii];        
	b += val & 0x1F;
	g += (val>>5) & 0x3F;
	r += val >> 11;
      } //end for i...

			/// Round before scaling.
      //int ao = (y*_widthOut*4) + (x*4);	//- no need to calculate for every pixel
      *pDst++ = (b>>4) | ((g<<1)&0x7E0) | ((r<<7)&0xF800);
    } //end for x...
  } //end for y...

	return(1);
}//end Scale.


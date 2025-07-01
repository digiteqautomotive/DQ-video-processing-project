/** @file

MODULE				: PicScalerRGB32ImplA

FILE NAME			: PicScalerRGB32ImplA.cpp

DESCRIPTION			: An RGB32 implementation derived from the general
					PicScalerBase() class. Scale a packed RGB32 image
					to the dimensions of another packed RGB32 image.
					Note that the upscaling is limited to 2x either 
					dimension.

NOTE:	This code replaces PicScalerRGB32Impl.cpp and must not be compilled together with this file.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025 Jaroslav Fojtik
All rights reserved.

===========================================================================
*/
#include <stdlib.h>

#include "PicScalerRGB32Impl.h"


extern "C" void ScaleRowAsm(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);

/*
===========================================================================
	Interface Methods.
===========================================================================
*/
/** Scale the size of the input image.
Scale the input image from its dimensions to that of the output image. No
memory size checking is done and is delegated to the calling process.
@param pOutImg	: Packed RGBA 8888 format main base image.
@param pInImg		: Packed RGBA 8888 format smaller sub image.
@return					: 0 = failed, 1 = success.
*/
int PicScalerRGB32Impl::Scale(void* pOutImg, void* pInImg)
{
  if((pOutImg == NULL) || (pInImg == NULL))
	return(0);

   unsigned char *pSrcRows[3];
   unsigned char *pDst		= (unsigned char*)pOutImg;	
   int y, posy;
   int accuY;

   accuY = posy = 0;
   for(y = 0; y < _heightOut; y++)
   {
      pSrcRows[0] = (unsigned char*)pInImg + ((posy==0) ? 0 : (4*_widthIn*(posy-1)));
      pSrcRows[1] = (unsigned char*)pInImg + 4*_widthIn*posy;
      pSrcRows[2] = (unsigned char*)pInImg + ((posy+1>=_heightIn) ? (4*_widthIn*(_heightIn-1)) : (4*_widthIn*(posy+1)));

      ScaleRowAsm(pDst, _widthOut, (void**)pSrcRows, _widthIn);
		
      pDst += 4 * _widthOut;
      accuY += _heightIn;				// DDA integer only algorithm
      posy += accuY / _heightOut;
      accuY = accuY % _heightOut;
   } //end for y...

	return(1);
}//end Scale.


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

#include "PicScalerYUYVImpl.h"


extern "C" void ScaleRowAsmYp2(void *pDst, unsigned _widthOut, void * const* pSrcRows, unsigned _widthIn);
extern "C" void ScaleRowAsmUVp4(void *pDst, unsigned _widthOut, void * const* pSrcRows, unsigned _widthIn);

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
int PicScalerYUYVImpl::Scale(void* pOutImg, const void* pInImg)
{
  if(pOutImg==NULL || pInImg==NULL || _widthIn==0 || _heightIn==0)
		return(0);

   const unsigned char *pSrcRows[3];
   unsigned char *pDst		= (unsigned char*)pOutImg;	
   int y, posy;
   int accuY;

   accuY = -1;
   posy = 0;
   y = _heightOut;
   while(y-- > 0)
   {
      accuY += _heightIn;				// DDA integer only algorithm
      posy += accuY / _heightOut;
      accuY = accuY % _heightOut;

      pSrcRows[0] = (const unsigned char*)pInImg + ((posy==0) ? 0 : (2*_widthIn*(posy-1)));
      pSrcRows[1] = (const unsigned char*)pInImg + 2*_widthIn*posy;
      pSrcRows[2] = (const unsigned char*)pInImg + 2*_widthIn*((posy+1>=_heightIn) ? (_heightIn-1) : (posy+1));

      ScaleRowAsmYp2(pDst, _widthOut, (void*const*)pSrcRows, _widthIn);
      pSrcRows[0]++;
      pSrcRows[1]++;
      pSrcRows[2]++;
      ScaleRowAsmUVp4(pDst+1, _widthOut/2, (void*const*)pSrcRows, _widthIn/2);
		
      pDst += 2 * _widthOut;
   } //end for y...

	return(1);
}//end Scale.


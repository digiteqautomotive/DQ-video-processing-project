/** @file

MODULE				: PicScalerARGB32SSE

FILE NAME			: PicScalerARGB32SSE.cpp

DESCRIPTION			: An RGB32 implementation derived from the general
					PicScalerBase() class. Scale a packed RGB32 image
					to the dimensions of another packed RGB32 image.
					Note that the upscaling is limited to 2x either 
					dimension.

NOTE:	This code replaces PicScalerARGB32SSE.cpp and must not be compilled together with this file.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025 Jaroslav Fojtik
All rights reserved.

===========================================================================
*/
#include <stdlib.h>
#include <string.h>

#include "PicScalerARGB32SSE.h"


#ifdef USE_SSE


extern "C" void ScaleRowAsmARGB32SSE(void *pDst, unsigned _widthOut, void *const*pSrcRows, unsigned _widthIn);


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
int PicScalerARGB32SSE::Scale(void* pOutImg, const void* pInImg, bool VFlip)
{
  if(pOutImg==NULL || pInImg==NULL || _widthIn==0 || _heightIn==0)
		return(0);

   const unsigned char *pSrcRows[3];
   unsigned char *pDst = (unsigned char*)pOutImg;
   int y, posy;
   int accuY;

   accuY = -1;
   posy = 0;
   y = _heightOut;

   if(VFlip)
   {
     pDst += 4*_widthOut*(_heightOut-1);
     while(y-- > 0)
     {
       accuY += _heightIn;				// DDA integer only algorithm
       posy += accuY / _heightOut;
       accuY = accuY % _heightOut;

       pSrcRows[0] = (const unsigned char*)pInImg + ((posy==0) ? 0 : (4*_widthIn*(posy-1)));
       pSrcRows[1] = (const unsigned char*)pInImg + 4*_widthIn*posy;
       pSrcRows[2] = (const unsigned char*)pInImg + 4*_widthIn*((posy+1>=_heightIn) ? (_heightIn-1) : (posy+1));

       ScaleRowAsmARGB32SSE(pDst, _widthOut, (void*const*)pSrcRows, _widthIn);
		
       pDst -= 4 * _widthOut;
    } //end for y...
   }
   else
   {
     while(y-- > 0)
     {
       accuY += _heightIn;				// DDA integer only algorithm
       posy += accuY / _heightOut;
       accuY = accuY % _heightOut;

       pSrcRows[0] = (const unsigned char*)pInImg + ((posy==0) ? 0 : (4*_widthIn*(posy-1)));
       pSrcRows[1] = (const unsigned char*)pInImg + 4*_widthIn*posy;
       pSrcRows[2] = (const unsigned char*)pInImg + 4*_widthIn*((posy+1>=_heightIn) ? (_heightIn-1) : (posy+1));

       ScaleRowAsmARGB32SSE(pDst, _widthOut, (void*const*)pSrcRows, _widthIn);
		
       pDst += 4 * _widthOut;
    } //end for y...
  }

   return(1);
}//end Scale.


#endif

/** @file

MODULE				: PicScalerRGB32Impl

FILE NAME			: PicScalerRGB32Impl.cpp

DESCRIPTION			: An RGB32 implementation derived from the general
					PicScalerBase() class. Scale a packed RGB32 image
					to the dimensions of another packed RGB32 image.
					Note that the upscaling is limited to 2x either 
					dimension.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025 Jaroslav Fojtik
All rights reserved.

===========================================================================
*/
#include <stdlib.h>

#include "PicScalerRGB32Impl.h"

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
	if( (pOutImg == NULL) || (pInImg == NULL) )
		return(0);

	const unsigned char*	pSrc	= (unsigned char*)pInImg;
	unsigned char*	pDst		= (unsigned char*)pOutImg;
	
	int x, y, posx, posy, i;
        int accuX, accuY;

	accuY = posy = 0;
	for(y = 0; y < _heightOut; y++)
	{
		const int pRow[3] = {						// Calculate row starts only once per row
				((posy==0) ? 0 : (4*_widthIn*(posy-1))),
				4*_widthIn*posy,
				((posy+1>=_heightIn) ? (4*_widthIn*(_heightIn-1)) : (4*_widthIn*(posy+1))) };

		accuX = posx = 0;
		for(x = 0; x < _widthOut; x++)
		{
			/// Apply a weighted 3x3 FIR filter.
			unsigned b = 8;			// rounding offset +8
			unsigned g = 8;
			unsigned r = 8;

			for(i = 0; i <= 2; i++)
			{				
				int aii = pRow[i] + 4 * ((posx==0) ? (0) : (posx - 1));
				b += (*(pSrc + aii));
				g += (*(pSrc + (aii+1)));
				r += (*(pSrc + (aii+2)));
				//a += (int)(*(pSrc + (aii+3)));	// do not filter missing 'a' compound

				if(posx>0) aii+=4;
				if(i==1)
				{
				  b += 8 * (*(pSrc + aii));
				  g += 8 * (*(pSrc + (aii+1)));
				  r += 8 * (*(pSrc + (aii+2)));
				}
				else
				{
				  b += (*(pSrc + aii));
				  g += (*(pSrc + (aii+1)));
				  r += (*(pSrc + (aii+2)));
				}                                

				if(posx+1 < _widthIn) aii+=4;
				b += (*(pSrc + aii));
				g += (*(pSrc + (aii+1)));
				r += (*(pSrc + (aii+2)));
			} //end for i...

			/// Round before scaling.
			//int ao = (y*_widthOut*4) + (x*4);	//- no need to calculate for every pixel
			pDst[0]	= (unsigned char)(b >> 4);
			pDst[1]	= (unsigned char)(g >> 4);
			pDst[2] = (unsigned char)(r >> 4);
			pDst[3] = *(pSrc + (pRow[1]+posx*4+3));		// 'a' compound is unused. (unsigned char)((a + 8) >> 4);
			pDst += 4;

			accuX += _widthIn;			// DDA integer only algorithm
			posx += accuX / _widthOut;
			accuX = accuX % _widthOut;
		} //end for x...
		
		accuY += _heightIn;				// DDA integer only algorithm
                posy += accuY / _heightOut;
                accuY = accuY % _heightOut;
	} //end for y...

	return(1);
}//end Scale.


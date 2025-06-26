/** @file

MODULE				: PicScalerARGB32Impl

FILE NAME			: PicScalerARGB32Impl.cpp

DESCRIPTION			: An ARGB32 implementation derived from the general
					PicScalerBase() class. Scale a packed ARGB32 image
					to the dimensions of another packed ARGB32 image.
					Note that the upscaling is limited to 2x either 
					dimension.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025 Jaroslav Fojtik
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

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

===========================================================================
*/
#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>
#else
#include <stdio.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "PicScalerARGB32Impl.h"

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
int PicScalerARGB32Impl::Scale(void* pOutImg, void* pInImg)
{
	if( (pOutImg == NULL) || (pInImg == NULL) )
		return(0);

	unsigned char*	pSrc		= (unsigned char*)pInImg;
	unsigned char*	pDst		= (unsigned char*)pOutImg;
	
	int x,y,posx,posy,i;
        int accuX, accuY;

	accuY = posy = 0;
	int ao = 0;			
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
			unsigned b = 8;
			unsigned g = 8;
			unsigned r = 8;
			unsigned a = 8;

			for(i = 0; i <= 2; i++)
			{				
				int aii = pRow[i] + 4 * ((posx==0) ? (0) : (posx - 1));
				b += (*(pSrc + aii));
				g += (*(pSrc + (aii+1)));
				r += (*(pSrc + (aii+2)));
                a += (*(pSrc + (aii+3)));

			    if(posx>0) aii+=4;
				if(i==1)
				{
				  b += 8 * (*(pSrc + aii));
				  g += 8 * (*(pSrc + (aii+1)));
				  r += 8 * (*(pSrc + (aii+2)));
                  a += 8 * (*(pSrc + (aii+3)));
				}
				else
				{
				  b += (*(pSrc + aii));
				  g += (*(pSrc + (aii+1)));
				  r += (*(pSrc + (aii+2)));
                  a += (*(pSrc + (aii+3)));
				}

				if(posx+1 < _widthIn) aii+=4;
			    b += (*(pSrc + aii));
				g += (*(pSrc + (aii+1)));
				r += (*(pSrc + (aii+2)));
                a += (*(pSrc + (aii+3)));				
			}//end for i...

			/// Round before scaling.
			//int ao = (y*_widthOut*4) + (x*4);
			*(pDst + ao)		= (unsigned char)(b >> 4);
			*(pDst + (ao+1))	= (unsigned char)(g >> 4);
			*(pDst + (ao+2))	= (unsigned char)(r >> 4);
			*(pDst + (ao+3))	= (unsigned char)(a >> 4);
			ao += 4;

			accuX += _widthIn;			// DDA integer only algorithm
			posx += accuX / _widthOut;
			accuX = accuX % _widthOut;
		}//end for x...
		
		accuY += _heightIn;				// DDA integer only algorithm
                posy += accuY / _heightOut;
                accuY = accuY % _heightOut;
	}//end for y...

	return(1);
}//end Scale.


/** @file

MODULE				: PicScalerRGB24Impl

FILE NAME			: PicScalerRGB24Impl.cpp

DESCRIPTION			: An RGB24 implementation derived from the general
					PicScalerBase() class. Scale a packed RGB24 image
					to the dimensions of another packed RGB24 image.
					Note that the upscaling is limited to 2x either 
					dimension.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
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
#include <stdlib.h>

#include "PicScalerRGB24Impl.h"

/*
===========================================================================
	Interface Methods.
===========================================================================
*/
/** Scale the size of the input image.
Scale the input image from its dimensions to that of the output image. No
memory size checking is done and is delegated to the calling process.
@param pOutImg	: Packed RGB 888 format main base image.
@param pInImg		: Packed RGB 888 format smaller sub image.
@return					: 0 = failed, 1 = success.
*/
int PicScalerRGB24Impl::Scale(void* pOutImg, const void* pInImg)
{
	if(pOutImg==NULL || pInImg==NULL || _widthIn==0 || _heightIn==0)
		return(0);

	const unsigned char* pSrc	= (const unsigned char*)pInImg;
	unsigned char*	pDst		= (unsigned char*)pOutImg;

	int x, y, posx, posy, i;
        int accuY, accuX;

        accuY = _heightIn / 2;
        posy = 0;
	y = labs(_heightOut);
        while(y-- > 0)	
	{
		const int pRow[3] = {						// Calculate row starts only once per row
			((posy==0) ? 0 : (3*_widthIn*(posy-1))),
			3*_widthIn*posy,
			((posy+1>=_heightIn) ? (3*_widthIn*(_heightIn-1)) : (3*_widthIn*(posy+1)))};

		accuX = _widthIn / 2;
                posx = 0;
		for(x = 0; x < _widthOut; x++)
		{
			/// Apply a weighted 3x3 FIR filter.
			unsigned b = 8;			// rounding offset +8
			unsigned g = 8;
			unsigned r = 8;

			for(i=0; i<=2; i++)
			{
				int aii = pRow[i] + 3 * ((posx==0) ? (0) : (posx - 1));
				b += (*(pSrc + aii));
				g += (*(pSrc + (aii+1)));
				r += (*(pSrc + (aii+2)));

				if(posx>0) aii+=3;
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

				if(posx+1 < _widthIn) aii+=3;
				b += (*(pSrc + aii));
				g += (*(pSrc + (aii+1)));
				r += (*(pSrc + (aii+2)));
			} //end for i...

			  /// Round before scaling.
			//int ao = (y*_widthOut*3) + (x*3);
			pDst[0]	= (unsigned char)(b >> 4);
			pDst[1]	= (unsigned char)(g >> 4);
			pDst[2]	= (unsigned char)(r >> 4);
			pDst += 3;                      

			accuX += _widthIn;			// DDA integer only algorithm
			posx += accuX / _widthOut;
			accuX = accuX % _widthOut;
		} //end for x...

		accuY += _heightIn;				// DDA integer only algorithm
                posy += accuY / _heightOut;
                accuY = accuY % _heightOut;
	} //end for y...

	return(1);
} //end Scale.

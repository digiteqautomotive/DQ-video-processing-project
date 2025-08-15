/** @file

MODULE				: PicScalerYUYVImpl

FILE NAME			: PicScalerYUYVImpl.cpp

DESCRIPTION			: An YUYV implementation derived from the general
					PicScalerBase() class. Scale a packed YUYV image
					to the dimensions of another packed YUYV image.
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
#include <stdlib.h>

#include "PicScalerYUYVImpl.h"

/*
===========================================================================
	Interface Methods.
===========================================================================
*/
/** Scale the size of the input image.
Scale the input image from its dimensions to that of the output image. No
memory size checking is done and is delegated to the calling process.
@param pOutImg	: Packed YUYV format main base image.
@param pInImg		: Packed YUYV format smaller sub image.
@return					: 0 = failed, 1 = success.
*/
int PicScalerYUYVImpl::Scale(void* pOutImg, const void* pInImg)
{
  if(pOutImg == NULL || pInImg == NULL || _widthIn==0 || _heightIn==0)
      return(0);

  const unsigned char*	pSrc	= (unsigned char*)pInImg;
  unsigned char*	pDst	= (unsigned char*)pOutImg;
	
  int x, y, posx, posy, i;
  int accuX, accuY;

  accuY = -1;
  posy = 0;
  y = _heightOut;
  while(y-- > 0)	
  {
    accuY += _heightIn;				// DDA integer only algorithm
    posy += accuY / _heightOut;
    accuY = accuY % _heightOut;

    const unsigned pRow[3] = {						// Calculate row starts only once per row
			((posy==0) ? 0 : (2*_widthIn*(posy-1))),
			2*_widthIn*posy,
			((posy+1>=_heightIn) ? (2*_widthIn*(_heightIn-1)) : (2*_widthIn*(posy+1))) };

    accuX = -1;
    posx = 0;
    for(x=0; x<_widthOut; x++)	// Y compound
    {
      accuX += _widthIn;			// DDA integer only algorithm
      posx += accuX / _widthOut;
      accuX = accuX % _widthOut;

			/// Apply a weighted 3x3 FIR filter.
      unsigned Y = 8;			// rounding offset +8
      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + 2 * ((posx==0) ? (0) : (posx - 1));
	Y += (*(pSrc + aii));

        if(posx>0) aii+=2;
        if(i==1)
        {
          Y += 8 * (*(pSrc + aii));
	}
	else
	{
          Y += (*(pSrc + aii));
	}
	if(posx+1 < _widthIn) aii+=2;
	Y += (*(pSrc + aii));
      } //end for i...
			/// Round before scaling.
      //pDst = (unsigned char*)pOutImg + (y*_widthOut*4) + (x*4);
      pDst[x*2]	= (unsigned char)(Y >> 4);
    }

    accuX = -1;
    posx = 0;
    for(x=0; x<_widthOut/2; x++)	// U and V compounds
    {
      accuX += _widthIn/2;			// DDA integer only algorithm
      posx += accuX / (_widthOut/2);
      accuX = accuX % (_widthOut/2);
      unsigned U = 8;
      unsigned V = 8;

      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + 4 * ((posx==0) ? (0) : (posx - 1));
	U += (*(pSrc + aii+1));
	V += (*(pSrc + aii+3));

        if(posx>0) aii+=4;
        if(i==1)
        {
          U += 8 * (*(pSrc + aii+1));
          V += 8 * (*(pSrc + aii+3));
	}
	else
	{
          U += (*(pSrc + aii+1));
          V += (*(pSrc + aii+3));
	}
	if(posx+1 < _widthIn/2) aii+=4;
	U += (*(pSrc + aii+1));
        V += (*(pSrc + aii+3));
      } //end for i...

      (pDst)[x*4+1] = U >> 4;
      (pDst)[x*4+3] = V >> 4;
    } //end for x...

    pDst += _widthOut * 2;
  } //end for y...

  return(1);
}//end Scale.


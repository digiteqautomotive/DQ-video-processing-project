/** @file

MODULE				: PicScalerYUV420PImpl

FILE NAME			: PicScalerYUV420PImpl.cpp

DESCRIPTION			: A YUV420P planar implementation derived from the 
					general PicScalerBase() class. Scale a YUV420P
					planar image to the dimensions of another YUV420P
					planar image.
					  
					  
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
===========================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "PicScalerYUV420PImpl.h"

#include <uuids.h>
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71); 
DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);


/*===========================================================================
	Interface Methods.
===========================================================================*/


bool PicScalerYUV420PImpl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_YUV420P_S || VideoFormat==MEDIASUBTYPE_I420 || VideoFormat==MEDIASUBTYPE_YV12;
}


/** Scale the size of the input image.
Scale the input image from its dimensions to that of the output image. The 
lum Y 8 bit pels are followed immediately by the U and then V 8 bit values.
No memory size checking is done and is delegated to the calling process.
@param pOutImg	: Planar YUV420P format output image.
@param pInImg		: Planar YUV420P format input image.
@return			: 0 = failed, 1 = success. */
int PicScalerYUV420PImpl::Scale(void* pOutImg, const void* pInImg, bool VFlip)
{
  if(_heightOut==0 || _widthOut==0) return(1);
  if(_heightIn==0 || _widthIn==0)
  {
    return(0);
  }
  if((pOutImg == NULL) || (pInImg == NULL))  return(0);

  const scaleType *pSrc = (const scaleType*)pInImg;
  scaleType *pDst = (scaleType*)pOutImg;

  int x,y,posx,posy,i;
  int accuX, accuY;	

	/// Lum 1st.
  int pRow[3] = {0,						// Calculate row starts only once per row
		0,
		(_heightIn<=1) ? 0 : _widthIn};
  accuY = posy = 0;  
  for(y=0; y<_heightOut; y++)
  {
    accuX = posx = 0;
    for(x=0; x<_widthOut; x++)
    {
      		/// Apply a weighted 3x3 FIR filter.
      unsigned Y = 8;			// rounding offset +8
      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + ((posx==0) ? (0) : (posx - 1));
	Y += pSrc[aii];

        if(posx>0) aii++;
        if(i==1)
            Y += 8 * pSrc[aii];
        else
            Y += pSrc[aii];

	if(posx+1 < _widthIn) aii++;
	Y += pSrc[aii];
      } //end for i...
      pDst[y*_widthOut + x] = (scaleType)(Y >> 4);

      accuX += _widthIn;			// DAA integer only algorithm
      posx += accuX / _widthOut;
      accuX = accuX % _widthOut;
    } //end for x...

    accuY += _heightIn;				// DAA integer only algorithm
    if(accuY >= _heightOut)
    {
      posy += accuY / _heightOut;		// increment
      accuY %= _heightOut;
      pRow[0] = _widthIn*(posy-1);		// Calculate row starts when source pos changes
      pRow[1] = pRow[0] + _widthIn;
      pRow[2] = ((posy+1>=_heightIn) ? (_widthIn*(_heightIn-1)) : (pRow[1]+_widthIn));
    }
  } //end for y...

	/// Chr U and V.
  const int chrWidthIn	= _widthIn/2;
  const int chrHeightIn	= _heightIn/2;
  const int chrWidthOut	= _widthOut/2;
  const int chrHeightOut= _heightOut/2;

  const scaleType *pSrcU = (const scaleType *)pInImg + (_widthIn*_heightIn);
  const scaleType *pSrcV = (const scaleType *)pInImg + (_widthIn*_heightIn) + (chrWidthIn*chrHeightIn);
  scaleType *pDstU	= (scaleType*)pOutImg + (_widthOut*_heightOut);
  scaleType *pDstV	= (scaleType*)pOutImg + (_widthOut*_heightOut) + (chrWidthOut*chrHeightOut);

  accuY = posy = 0;
  pRow[0] = 0;						// Calculate row starts only once per row
  pRow[1] = 0;
  pRow[2] = (chrHeightIn<=1) ? 0 : chrWidthIn;
  for(y=0; y<chrHeightOut; y++)
  {
    accuX = posx = 0;
    for(x=0; x<chrWidthOut; x++)
    {
      unsigned U = 8;			// rounding offset +8
      unsigned V = 8;			// rounding offset +8
      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + ((posx==0) ? (0) : (posx - 1));
	U += pSrcU[aii];
        V += pSrcV[aii];

        if(posx>0) aii++;
        if(i==1)
        {
            U += 8 * pSrcU[aii];
            V += 8 * pSrcV[aii];
        }
        else
        {
            U += pSrcU[aii];
            V += pSrcV[aii];
        }

	if(posx+1 < chrWidthIn) aii++;
	U += pSrcU[aii];
	V += pSrcV[aii];
      } //end for i...

      const int ao = (y*chrWidthOut) + x;
      pDstU[ao] = (scaleType)(U >> 4);
      pDstV[ao] = (scaleType)(V >> 4);

      accuX += _widthIn;			// DDA integer only algorithm, no need to use reduced scale, ratio is the same.
      posx += accuX / _widthOut;
      accuX = accuX % _widthOut;
    } //end for x...

    accuY += _heightIn;				// DDA integer only algorithm
    if(accuY >= _heightOut)
    {
      posy += accuY / _heightOut;
      accuY %= _heightOut;
      pRow[0] = chrWidthIn*(posy-1);		// Calculate row starts when source pos changes
      pRow[1] = pRow[0] + chrWidthIn;
      pRow[2] = (posy+1>=chrHeightIn) ? (chrWidthIn*(chrHeightIn-1)) : (pRow[1]+chrWidthIn);                
    }
  } //end for y...
  return(1);
} //end Scale.

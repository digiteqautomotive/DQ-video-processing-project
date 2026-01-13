/** @file

MODULE				: PicScalerYV24Impl

FILE NAME			: PicScalerYV24Impl.cpp

DESCRIPTION			: A YUV420P planar implementation derived from the 
					general PicScalerBase() class. Scale a YUV420P
					planar image to the dimensions of another YUV420P
					planar image.
					  
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, designe by Jaroslav Fojtik	All rights reserved.

===========================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "PicScalerYV24Impl.h"

#include <uuids.h>


DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);


/*===========================================================================
	Interface Methods.
===========================================================================*/


bool PicScalerYV24Impl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_YV24;
}


/** Scale the size of the input image.
Scale the input image from its dimensions to that of the output image. The 
lum Y 8 bit pels are followed immediately by the U and then V 8 bit values.
No memory size checking is done and is delegated to the calling process.
@param pOutImg	: Planar YUV420P format output image.
@param pInImg		: Planar YV24 format input image.
@return			: 0 = failed, 1 = success. */
int PicScalerYV24Impl::Scale(void* pOutImg, const void* pInImg, bool VFlip)
{
  if(_heightOut==0 || _widthOut==0) return(1);
  if(_heightIn==0 || _widthIn==0)
  {
    return(0);
  }
  if((pOutImg == NULL) || (pInImg == NULL))  return(0);

  const unsigned char *pSrc = (const unsigned char *)pInImg;
  unsigned char *pDst = (unsigned char *)pOutImg;

  int x,y,posx,posy,i;
  int accuX, accuY;
  const unsigned PlanarOffsetIn = _widthIn*_heightIn;
  const unsigned PlanarOffsetOut = _widthOut*_heightOut;

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
      unsigned U = 8;
      unsigned V = 8;
      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + ((posx==0) ? (0) : (posx - 1));
	Y += pSrc[aii];
	U += pSrc[aii+PlanarOffsetIn];
	V += pSrc[aii+2*PlanarOffsetIn];

        if(posx>0) aii++;
        if(i==1)
        {
          Y += 8 * pSrc[aii];
          U += 8 * pSrc[aii+PlanarOffsetIn];
          V += 8 * pSrc[aii+2*PlanarOffsetIn];
        }
        else
        {
          Y += pSrc[aii];
          U += pSrc[aii+PlanarOffsetIn];
          V += pSrc[aii+2*PlanarOffsetIn];
        }

	if(posx+1 < _widthIn) aii++;
	Y += pSrc[aii];
	U += pSrc[aii+PlanarOffsetIn];
        V += pSrc[aii+2*PlanarOffsetIn];
      } //end for i...

      i = y*_widthOut + x;
      pDst[i] = (unsigned char)(Y >> 4);
      i += PlanarOffsetOut;
      pDst[i] = (unsigned char)(U >> 4);
      i += PlanarOffsetOut;
      pDst[i] = (unsigned char)(Y >> 4);

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

  return(1);
} //end Scale.

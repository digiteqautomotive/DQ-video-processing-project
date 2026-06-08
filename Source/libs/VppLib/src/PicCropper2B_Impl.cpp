/** @file

MODULE:			PicCropper2B_Impl

FILE NAME:		PicCropper2B_Impl.cpp

DESCRIPTION:		A 2 BYTE implementation derived from the general
			PicCropperBase class. Insert a packed RGB555 or RGB565 image
			to be cropped.
					  			  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.
===========================================================================*/
#include "PicCropper2B_Impl.h"

#include <uuids.h>

/*===========================================================================
				Interface Methods
===========================================================================*/


bool PicCropper2B_Impl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_RGB555 || VideoFormat==MEDIASUBTYPE_RGB565;
}


/** Crop the input image into the output image.
Crop the input image by {_byX1, _byX2, _byY1, _byY2} from the {left, right,
bottom, top} and write it into the output image. Memory management is the
responsibility of the calling process. Very little checking is done to
prevent memory overwriting.
@param pInImg	: Packed RGB 8888 format input image.
@param pImg		: Packed RGB 8888 format output image.
@return				: 0 = failed, 1 = success. */
int PicCropper2B_Impl::Crop(const void* pInImg, void* pImg, bool VFlip)
{
  if( (pImg == NULL) || (pInImg == NULL) )
      return(0);

	/// Check the cropping to ensure it will fit and not overlap.
  int writeWidth = _widthIn - _byX1 - _byX2;
  int writeHeight = _heightIn - _byY1 - _byY2;
  if( (writeWidth < 0) || (writeHeight < 0) )
	return(0);
  if(writeWidth > _widthOut) writeWidth = _widthOut;
  if(writeHeight > _heightOut) writeHeight = _heightOut;

	/// Align src and dst pointers. Src pointer is offset into the input image.
  const unsigned char* pSrc = (const unsigned char*)pInImg + 2*((_byY1*_widthIn) + _byX1);
  unsigned char* pDst	= (unsigned char*)pImg;

  int y = writeHeight;
  if(VFlip)
  {
    pSrc += (writeHeight-1) * (2*_widthIn);
    while(y-- > 0)
    {
	memcpy(pDst, pSrc, 2*writeWidth);	/// Whole row at a time.
	pDst += (2*_widthOut);	/// Next row.
	pSrc -= (2*_widthIn);
    } //end for y...
  }
  else
  {
    while(y-- > 0)
    {
	memcpy((void *)pDst, (const void *)pSrc, 2*writeWidth);	/// Whole row at a time.
	pDst += (2*_widthOut);	/// Next row.
	pSrc += (2*_widthIn);
    } //end for y...
  }

  return(1);
}//end Crop.


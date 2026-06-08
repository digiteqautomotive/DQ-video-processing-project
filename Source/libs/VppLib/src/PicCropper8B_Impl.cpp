/** @file

MODULE:			PicCropper8B_Impl

FILE NAME:		PicCropper8B_Impl.cpp

DESCRIPTION:		A 6 BYTE implementation derived from the general
			PicCropperBase class. Insert a packed BR16 image
			to be cropped.
					  
LICENSE: Software License Agreement (BSD License)

(c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.
===========================================================================*/
#include "PicCropper8B_Impl.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_A16R16G16B16, 0x36314241, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_A16B16G16R16, 0x36315241, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

/*===========================================================================
			Interface Methods.
===========================================================================*/


bool PicCropper8B_Impl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_A16R16G16B16 || VideoFormat==MEDIASUBTYPE_A16B16G16R16;
}


/** Crop the input image into the output image.
Crop the input image by {_byX1, _byX2, _byY1, _byY2} from the {left, right,
bottom, top} and write it into the output image. Memory management is the
responsibility of the calling process. Very little checking is done to
prevent memory overwriting.
@param pImg		: Packed RGB 888 format output image.
@param pInImg	: Packed RGB 888 format input image.
@return				: 0 = failed, 1 = success.
*/
int PicCropper8B_Impl::Crop(const void* pInImg, void* pImg, bool VFlip)
{
  if((pImg == NULL) || (pInImg == NULL))
      return(0);

	/// Check the cropping to ensure it will fit and not overlap.
  int writeWidth = _widthIn - _byX1 - _byX2;
  int writeHeight = _heightIn - _byY1 - _byY2;
  if((writeWidth < 0) || (writeHeight < 0))
		return(0);
  if(writeWidth > _widthOut) writeWidth = _widthOut;
  if(writeHeight > _heightOut) writeHeight = _heightOut;

	/// Align src and dst pointers. Src pointer is offset into the input image.
  const unsigned char *pSrc= (const unsigned char*)pInImg + 6*((_byY1*_widthIn) + _byX1);
  unsigned char *pDst	= (unsigned char*)pImg;

  int y = writeHeight;
  if(VFlip)
  {
    pSrc += (writeHeight-1) * (6*_widthIn);
    while(y-- > 0)
    {
      memcpy(pDst, pSrc, 6*writeWidth);	/// Whole row at a time.
      pDst += (6*_widthOut);	/// Next row.
      pSrc -= (6*_widthIn);
    }//end for y...
  }
  else
  {
    while(y-- > 0)
    {
      memcpy(pDst, pSrc, 6*writeWidth);	/// Whole row at a time.
      pDst += (6*_widthOut);	/// Next row.
      pSrc += (6*_widthIn);
    }//end for y...
  }

  return(1);
}//end Crop.


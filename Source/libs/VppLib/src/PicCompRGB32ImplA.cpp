/** @file

MODULE:		PicCompareRGB32ImplA

TAG:		PCRGB32I

FILE NAME:	PicCompareRGB32ImplA.cpp

DESCRIPTION:	An RGB32 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB32 
		image to another packed RGB32 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#include "PicCompRGB32Impl.h"


extern "C" void CompareRowAsm(const void *pSRC1, const void *pSRC2, void *pDST, unsigned count);


/*=========================================================================
	Interface Methods.
==========================================================================*/

/** Compare the 2nd img to the 1st img.
Attach the 2nd image to the 1st image at the set orientation (top,
bottom, left, right) in the output image. There is no memory checking
in this method and must be handled by the calling process.
@param pImg1st	: Packed RGB 888 format base image.
@param pImg2nd	: Packed RGB 888 format image to attach.
@param pImg			: Packed RGB 888 format output image.
@return				: 0 = failed, 1 = success. */
int PicCompareRGB32Impl::Compare(const void* pImg1st, const void* pImg2nd, void *pImg, bool VFlip)
{
  if(pImg==NULL || pImg1st==NULL || pImg2nd==NULL)
      return(0);

  if(VFlip)
  {
    const unsigned char *Img1 = (const unsigned char *)pImg1st;
    const unsigned char *Img2 = (const unsigned char *)pImg2nd;
    unsigned char *ImgOut = (unsigned char *)pImg;

    ImgOut += 4 * _width * (_height-1);
    unsigned y = _height;
    while(y-- > 0)  
    {
      CompareRowAsm(Img1, Img2, ImgOut, 4*_width);
      Img1 += 4 * _width;
      Img2 += 4 * _width;
      ImgOut -= 4 * _width;
    }
  }
  else
  {
    CompareRowAsm(pImg1st, pImg2nd, pImg, 4*_width*_height);
  }

  return(1);
} //end Compare


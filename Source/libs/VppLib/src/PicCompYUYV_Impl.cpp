/** @file

MODULE:		PicCompareYUYV_Impl

TAG:		PCYUYV_I

FILE NAME:	PicCompareYUYV_Impl.cpp

DESCRIPTION:	An YUYV_ implementation derived from the general
		PicConcatBase() class. Compare an YUYV image to 
		another YUYV image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#include <stdio.h>
#include "PicCompYUYV_Impl.h"

#include "typedfs.h"


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
int PicCompareYUYV_Impl::Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip)
{
  if(pImg==NULL || pImg1st==NULL || pImg2nd==NULL)
      return(0);

  const unsigned char *Img1 = (const unsigned char *)pImg1st;
  const unsigned char *Img2 = (const unsigned char *)pImg2nd;
  unsigned char *ImgOut = (unsigned char *)pImg;
  if(VFlip)
  {
    ImgOut += 2 * _width * (_height-1);
    unsigned y = _height;
    while(y-- > 0)
    {
      unsigned values =  _width;
      while(values-- > 0)
      {
        int16_t i = (int16_t)*Img1++ - (int16_t)*Img2++ + 127;
        if(i<=0) *ImgOut=0;
        else if(i>=255) *ImgOut=255;
        else *ImgOut=i;
        ImgOut++;
        i = (int16_t)*Img1++ - (int16_t)*Img2++ + 127;
        if(i<=0) *ImgOut=0;
        else if(i>=255) *ImgOut=255;
        else *ImgOut=i;
        ImgOut++;
      }
      ImgOut -= 4 * _width;
    }
  }
  else
  {
    unsigned values =  _width * _height;
    while(values-- > 0)
    {
      int16_t i = (int16_t)*Img1++ - (int16_t)*Img2++ + 127;
      if(i<=0) *ImgOut=0;
      else if(i>=255) *ImgOut=255;
      else *ImgOut=i;
      ImgOut++;
      i = (int16_t)*Img1++ - (int16_t)*Img2++ + 127;
      if(i<=0) *ImgOut=0;
      else if(i>=255) *ImgOut=255;
      else *ImgOut=i;
      ImgOut++;
    }
  }

  return(1);
} //end Compare

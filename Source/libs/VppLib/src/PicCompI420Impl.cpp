/** @file

MODULE:		PicCompareRGB32Impl

TAG:		PCI420I

FILE NAME:	PicCompI420Impl.cpp

DESCRIPTION:	An I420 implementation derived from the general
		PicConcatBase() class. Concatenate a packed I420 
		image to another packed I420 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#include <stdio.h>
#include "PicCompI420Impl.h"

#include "typedfs.h"


/*=========================================================================
			Interface Methods
==========================================================================*/

/** Compare the 2nd img to the 1st img.
Attach the 2nd image to the 1st image at the set orientation (top,
bottom, left, right) in the output image. There is no memory checking
in this method and must be handled by the calling process.
@param pImg1st	: Packed RGB 888 format base image.
@param pImg2nd	: Packed RGB 888 format image to attach.
@param pImg			: Packed RGB 888 format output image.
@return				: 0 = failed, 1 = success. */
int PicCompareI420Impl::Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip)
{
  if(pImg==NULL || pImg1st==NULL || pImg2nd==NULL)
      return(0);

  const unsigned char *Img1 = (const unsigned char *)pImg1st;
  const unsigned char *Img2 = (const unsigned char *)pImg2nd;
  unsigned char *ImgOut = (unsigned char *)pImg;
  if(VFlip)
  {
    ImgOut += _width * (_height-1);
    unsigned y = _height;
    while(y-- > 0)
    {
      unsigned values =  _width;
      while(values-- > 0)
      {
        int16_t i = (int16_t)*Img1++ - (int16_t)*Img2++ + 128;
        if(i<=0) *ImgOut=0;
        else if(i>=255) *ImgOut=255;
        else *ImgOut=i;
        ImgOut++;
      }
      ImgOut -= 2 * _width;
    }
    ImgOut = (unsigned char *)pImg + _width*_height + (_width*(_height-2))/4;
    //Img1 = (const unsigned char *)pImg1st + _width*_height;	// already assigned
    //Img2 = (const unsigned char *)pImg2nd + _width*_height;	// already assigned
    y = _height/2;
    while(y-- > 0)
    {
      unsigned values =  _width;
      while(values-- > 0)
      {
        int16_t i = (int16_t)*Img1++ - (int16_t)*Img2++ + 128;
        if(i<=0) *ImgOut=0;
        else if(i>=255) *ImgOut=255;
        else *ImgOut=i;
        ImgOut++;
      }
      ImgOut -= _width;
    }
    ImgOut = (unsigned char *)pImg + (5*_width*_height)/4 + (_width*(_height-2))/4;
    //Img1 = (const unsigned char *)pImg1st + (5*_width*_height)/4;	// already assigned
    //Img2 = (const unsigned char *)pImg2nd + (5*_width*_height)/4;	// already assigned
    y = _height/2;
    while(y-- > 0)
    {
      unsigned values =  _width;
      while(values-- > 0)
      {
        int16_t i = (int16_t)*Img1++ - (int16_t)*Img2++ + 128;
        if(i<=0) *ImgOut=0;
        else if(i>=255) *ImgOut=255;
        else *ImgOut=i;
        ImgOut++;
      }
      ImgOut -= _width;
    }
  }
  else
  {
    unsigned values =  (6 * _width * _height) / 4;	// 12 / 8
    while(values-- > 0)
    {
      int16_t i = (int16_t)*Img1++ - (int16_t)*Img2++ + 128;
      if(i<=0) *ImgOut=0;
      else if(i>=255) *ImgOut=255;
      else *ImgOut=i;
      ImgOut++;
    }
  }

  return(1);
} //end Compare

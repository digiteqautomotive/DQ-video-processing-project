/** @file

MODULE:		PicCompareRGB32Impl

TAG:		PCRGB32I

FILE NAME:	PicCompareRGB32Impl.cpp

DESCRIPTION:	An RGB32 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB32 
		image to another packed RGB32 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive All rights reserved.

LICENSE:	Software License Agreement (BSD License)

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
#include "PicCompRGB32Impl.h"

/*
===========================================================================
	Construction and Destruction.
===========================================================================
*/

PicCompareRGB32Impl::PicCompareRGB32Impl(void) 
{
	Create();
}//end default constuctor.


PicCompareRGB32Impl::PicCompareRGB32Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd): 
	PicCompareBase(width,height,width1st,height1st,width2nd,height2nd)
{
	Create();
}//end constructor.


PicCompareRGB32Impl::~PicCompareRGB32Impl(void)
{
	if(_pPicInPic != NULL)
		delete _pPicInPic;
}//end destructor.


int PicCompareRGB32Impl::Create(void)
{
	_pPicInPic = NULL;
	_pPicInPic = new PicInPicRGB32Impl();
	if(_pPicInPic == NULL)
		return(0);
	return(1);
}//end Create.


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
int PicCompareRGB32Impl::Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip)
{
  if( (pImg == NULL) || (pImg1st == NULL) || (pImg2nd == NULL) || (_pPicInPic == NULL) )
      return(0);

  if(VFlip)
  {
    const unsigned char *Img1 = (const unsigned char *)pImg1st;
    const unsigned char *Img2 = (const unsigned char *)pImg2nd;
    unsigned char *ImgOut = (unsigned char *)pImg + 4 * _width * (_height-1);
  }
  else
  {
    const unsigned char *Img1 = (const unsigned char *)pImg1st;
    const unsigned char *Img2 = (const unsigned char *)pImg2nd;
    unsigned char *ImgOut = (unsigned char *)pImg;
    unsigned values =  4 * _width * _height;
    while(values-- > 0)
    {
      int i = *Img1++ - *Img2++ + 127;
      if(i<=0) *ImgOut=0;
      else if(i>=255) *ImgOut=255;
      else *ImgOut=i;
      ImgOut++;
    }
  }

  return(1);
} //end Compare


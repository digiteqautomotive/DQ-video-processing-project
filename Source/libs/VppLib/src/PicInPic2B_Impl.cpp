/** @file

MODULE:		PicInPic2B_Impl

TAG:		PIPRGB565I

FILE NAME:	PicInPic2B_Impl.cpp

DESCRIPTION:	An RGB565 implementation derived from the general
		PicInPicBase() class. Insert a packed RGB565 or RGB555 image
		into another packed RGB565 or RGB555 image.

COPYRIGHT:      Copyright (c) 2026 Digiteq automotive; programmed by Jaroslav Fojtik

LICENSE:	Software License Agreement (BSD License)

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

#include "PicInPic2B_Impl.h"

#include <uuids.h>


/*===========================================================================
	Interface Methods.
===========================================================================*/


bool PicInPic2B_Impl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_RGB555 || VideoFormat==MEDIASUBTYPE_RGB565;
}


/** Insert the sub image into the main image.
Write the sub image into the main image at the top-left location of
(_xPos,_yPos). Only write _writeWidth and _writeHeight pixels of the
sub image. These dimensions have been set to stay within the edges
of the main image.
@param pSubImg	: Packed RGB565 format smaller sub image.
@param pImg	: Packed RGB 565 format main base image.
@return		: none. */
void PicInPic2B_Impl::DoInsert(const void* pSubImg, void* pImg, bool VFlip) const
{
  const unsigned char*	pSrc	= (const unsigned char*)pSubImg;
  unsigned char*	pDst	= (unsigned char*)pImg;

  if(VFlip)
  {
    pDst += 2*(((_height-_yPos)*_width) + _xPos);
    for(int y = 0; y<_writeHeight; y++)
    {
      pDst -= (2*_width);		// Next row.
      memcpy((void *)pDst, (const void *)pSrc, 2*_writeWidth);	// Whole row at a time.
      pSrc += (2*_subWidth);
    } //end for y...
  }
  else
  {
    pDst += 2*((_yPos*_width) + _xPos);
    for(int y=0; y<_writeHeight; y++)
    {
      memcpy((void *)pDst, (const void *)pSrc, 2*_writeWidth);	// Whole row at a time.
      pDst += (2*_width);		// Next row.
      pSrc += (2*_subWidth);
    } //end for y...
  }
} //end Insert.


/** Insert the sub image into the main image with a border.
Write the sub image into the main image at the top-left location of
(_xPos,_yPos). Only write _writeWidth and _writeHeight pixels of the
sub image. These dimensions have been set to stay within the edges
of the main image.
@param pSubImg	: Packed RGB 8888 format smaller sub image.
@param pImg			: Packed RGB 8888 format main base image.
@return					: none. */
void PicInPic2B_Impl::DoInsertWithBorder(const void* pSubImg, void* pImg, bool VFlip) const
{
    // TODO: Just calling standard insert for now
    DoInsert(pSubImg, pImg, VFlip);
}

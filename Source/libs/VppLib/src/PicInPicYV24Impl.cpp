/** @file

MODULE						: PicInPicYV24Impl

TAG						: PIPRGB24I

FILE NAME					: PicInPicYV24Impl.cpp

DESCRIPTION			: An YV24 implementation derived from the general
			  	  PicInPicBase() class. Insert a packed YV24 image
				  into another packed YVRGB24 image.

COPYRIGHT			: (c)2025-2026 Jaroslav Fojtik

LICENSE				: Software License Agreement (BSD License)

===========================================================================*/
#include "PicInPicYV24Impl.h"

DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

/*===========================================================================
	Interface Methods.
===========================================================================*/


bool PicInPicYV24Impl::isVideoFormatSupported(const GUID &VideoFormat) const
{
  return VideoFormat==MEDIASUBTYPE_YV24;
}


/** Insert the sub image into the main image.
Write the sub image into the main image at the top-left location of
(_xPos,_yPos). Only write _writeWidth and _writeHeight pixels of the
sub image. These dimensions have been set to stay within the edges
of the main image.
@param pSubImg	: Packed YV 888 format smaller sub image.
@param pImg			: Packed YV 888 format main base image.
@return				: none. */
void PicInPicYV24Impl::DoInsert(const void* pSubImg, void* pImg, bool VFlip) const
{
const unsigned char *pSrc = (const unsigned char*)pSubImg;
unsigned char *pDst = (unsigned char*)pImg;
unsigned BlockOffsetImg = _height * _width;
unsigned BlockOffsSubImg =_subHeight * _subWidth;

  if(VFlip)
  {
    pDst += (((_height-_yPos)*_width) + _xPos);
    for(int y=0; y<_writeHeight; y++)
    {
      pDst -= _width;	// Next row.
      memcpy(pDst, (const void *)pSrc, _writeWidth);	// Whole row at a time.
      memcpy(pDst+BlockOffsetImg, pSrc+BlockOffsSubImg, _writeWidth);
      memcpy(pDst+2*BlockOffsetImg, pSrc+2*BlockOffsSubImg, _writeWidth);
      pSrc += _subWidth;
    }  //end for y...
  }
  else
  {
    pDst += ((_yPos*_width) + _xPos);
    for(int y=0; y<_writeHeight; y++)
    {
      memcpy(pDst, pSrc, _writeWidth);		// Whole row at a time.
      memcpy(pDst+BlockOffsetImg, pSrc+BlockOffsSubImg, _writeWidth);
      memcpy(pDst+2*BlockOffsetImg, pSrc+2*BlockOffsSubImg, _writeWidth);
      pDst += _width;			// Next row.
      pSrc += _subWidth;
    }  //end for y...
  }
} //end DoInsert.


/** Insert the sub image into the main image with a border.
Write the sub image into the main image at the top-left location of
(_xPos,_yPos). Only write _writeWidth and _writeHeight pixels of the
sub image. These dimensions have been set to stay within the edges
of the main image.
@param pSubImg	: Packed RGB 888 format smaller sub image.
@param pImg	: Packed RGB 888 format main base image.
@return		: none. */
void PicInPicYV24Impl::DoInsertWithBorder(const void* pSubImg, void* pImg, bool VFlip) const
{
  DoInsert(pSubImg, pImg, VFlip);
/*
  {
    unsigned char*	pSrc		= (unsigned char*)pSubImg;
    unsigned char*	pDst		= (unsigned char*)pImg + 3*((_yPos*_width) + _xPos);

    const int iBorderWidth = 5;
    for(int y = 0; y < _writeHeight; y++)
    {
        if ( y < iBorderWidth || y >= _writeHeight - iBorderWidth)
        {
            for ( int iBorderX = 0; iBorderX < (3 *_writeWidth); iBorderX += 3)
            {
                pDst[iBorderX    ] = 0;
                pDst[iBorderX + 1] = 255;
                pDst[iBorderX + 2] = 0;
            }
            //memset(pDst, 255, 3*_writeWidth);
            pDst += (3*_width);	// Next row.
            pSrc += (3*_subWidth);
        }
        else
        {
            memcpy((void *)pDst, (const void *)pSrc, 3*_writeWidth);	// Whole row at a time.
            // Make border
            for ( int iBorderX = 0; iBorderX < (3 * iBorderWidth); iBorderX += 3)
            {
                pDst[iBorderX    ] = 0;
                pDst[iBorderX + 1] = 255;
                pDst[iBorderX + 2] = 0;
                pDst[ (3 * _writeWidth - 3) - iBorderX	   ] = 0;
                pDst[ (3 * _writeWidth - 3) - iBorderX + 1 ] = 255;
                pDst[ (3 * _writeWidth - 3) - iBorderX + 2 ] = 0;
            }

            pDst += (3*_width);	// Next row.
            pSrc += (3*_subWidth);
        }
    }//end for y...
  }
*/
} //end DoInsertWithBorder.
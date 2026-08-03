/** @file

MODULE:			PicConcatRGB565Impl

TAG:			PCRGB565I

FILE NAME:		PicConcatRGB565Impl.cpp

DESCRIPTION:		An RGB565 implementation derived from the general
			PicConcatBase() class. Concatenate a packed RGB565 
			image	to another packed RGB565 image.

COPYRIGHT:		(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:		Software License Agreement (BSD License)
===========================================================================*/
#include "PicConcatRGB565Impl.h"

/*
===========================================================================
	Construction and Destruction.
===========================================================================
*/

PicConcatRGB565Impl::PicConcatRGB565Impl(void) 
{
}//end default constuctor.


PicConcatRGB565Impl::PicConcatRGB565Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient): 
	PicConcatBase(width,height,width1st,height1st,width2nd,height2nd,orient)
{
  SetDimensions(PicConcatBase::_width, PicConcatBase::_height);		/// Set dimensions of output image.
}//end constructor.


void PicConcatRGB565Impl::SetOutDimensions(int width, int height)
{
  PicConcatBase::_width=labs(width); PicConcatBase::_height=labs(height);
  SetDimensions(PicConcatBase::_width, PicConcatBase::_height);		/// Set dimensions of output image.
}


/*=========================================================================
	Interface Methods.
==========================================================================*/

/** Concatenate the 2nd img to the 1st img.
Attach the 2nd image to the 1st image at the set orientation (top,
bottom, left, right) in the output image. There is no memory checking
in this method and must be handled by the calling process.
@param pImg1st	: Packed RGB 888 format base image.
@param pImg2nd	: Packed RGB 888 format image to attach.
@param pImg			: Packed RGB 888 format output image.
@return				: 0 = failed, 1 = success. */
int PicConcatRGB565Impl::Concat(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip)
{
  if(pImg==NULL || pImg1st==NULL || pImg2nd==NULL)
      return(0);

  int posX1, posY1, posX2, posY2;	/// Coords for top-left of images in output image.
  switch(GetOrientation(VFlip))
  {
    case PicConcatBase::TOP :
			posX1 = 0;	/// Base image position.
			posY1 = 0;
			posX2 = 0;	/// Concat image position.
			posY2 = PicConcatBase::_height - _height2nd;
			if(posY2 < 0) posY2 = 0;
			break;
     case PicConcatBase::BOTTOM :
			posX1 = 0;	/// Base image position.
			posY1 = PicConcatBase::_height - _height1st;
			if(posY1 < 0) posY1 = 0;
			posX2 = 0;	/// Concat image position.
			posY2 = 0;
			break;
    case PicConcatBase::LEFT :
			posX1 = PicConcatBase::_width - _width1st;	/// Base image position.
			if(posX1 < 0) posX1 = 0;
			posY1 = 0;
			posX2 = 0;	/// Concat image position.
			posY2 = 0;
			break;
    case PicConcatBase::RIGHT :
			posX1 = 0;	/// Base image position.
			posY1 = 0;
			posX2 = PicConcatBase::_width - _width2nd;	/// Concat image position.
			if(posX2 < 0) posX2 = 0;
			posY2 = 0;
			break;
    default:
			posX1 = 0;	/// Base image position.
			posY1 = 0;
			posX2 = 0;	/// Concat image position.
			posY2 = 0;
			break;
  } //end switch _orient...

	/// Concat image written to output 1st.
  SetSubDimensions(_width2nd, _height2nd);	/// Set dim of concat image.
  SetPos(posX2, posY2);			/// Images are written from bottom to top.
  Insert(pImg2nd, pImg, VFlip);

	/// Base image written last.
  SetSubDimensions(_width1st, _height1st);	/// Set dim of base image.
  SetPos(posX1, posY1);			/// Images are written from bottom to top.
  Insert(pImg1st, pImg, VFlip);

  return(1);
}//end Concat.


/** @file

MODULE:		PicConcatPiP

TAG:		PCRGB24I

FILE NAME:	PicConcatPiP.h

DESCRIPTION:	An RGB24 implementation derived from the general
		PicConcatBase() class.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICCONCAT_PiP_H
#define _PICCONCAT_PiP_H

#include "PicConcatBase.h"
#include "PicInPicBase.h"


/** \ingroup ImageLib An RGB24 implementation derived from the general
 * PicConcatBase() class. Concatenate a packed RGB24 
 * image	to another packed RG16 image. */
class PicConcatPiP: public PicConcatBase, virtual public PicInPicBase
{
public:
	// Construction and destruction.
	PicConcatPiP(void) {}
	PicConcatPiP(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient):
	PicConcatBase(width,height,width1st,height1st,width2nd,height2nd,orient) {
          PicInPicBase::SetDimensions(PicConcatBase::_width, PicConcatBase::_height);		/// Set dimensions of output image.
        }

	// Interface.
        void SetOutDimensions(int width, int height);
	virtual int Concat(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
}; //end PicConcatPiP.


#endif // _PICCONCAT_PiP_H

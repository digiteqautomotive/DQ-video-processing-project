/** @file

MODULE:		PicConcat8B_Impl

TAG:		PCRGB24I

FILE NAME:	PicConcat8B_Impl.h

DESCRIPTION:	An RGB24 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB24 
		image to another packed RGB24 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICCONCAT_8B_IMPL_H
#define _PICCONCAT_8B_IMPL_H

#include "PicConcatPiP.h"
#include "PicInPic8B_Impl.h"


/** \ingroup ImageLib An RGB24 implementation derived from the general
 * PicConcatBase() class. Concatenate a packed RGB24 
 * image	to another packed RG16 image. */
class PicConcat8B_Impl: public PicConcatPiP, virtual public PicInPic8B_Impl
{
public:
	// Construction and destruction.
	PicConcat8B_Impl(void) {}
	PicConcat8B_Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient):
		PicConcatPiP(width,height,width1st,height1st,width2nd,height2nd,orient) {}

	// Interface.
        virtual int GetVideoFormat(void) const {return 64;}
}; //end PicConcat8B_Impl.


#endif // _PICCONCAT_8B_IMPL_H

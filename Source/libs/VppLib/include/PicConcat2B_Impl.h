/** @file

MODULE:		PicConcat2B_Impl

TAG:		PCRGB565I

FILE NAME:	PicConcat2B_Impl.h

DESCRIPTION:	An RGB565 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB565 
		image to another packed RGB565 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICCONCAT_2B_IMPL_H
#define _PICCONCAT_2B_IMPL_H

#include "PicConcatPiP.h"
#include "PicInPic2B_Impl.h"


/** \ingroup ImageLib An RGB565 implementation derived from the general
 * PicConcatBase() class. Concatenate a packed RGB565 
 * image	to another packed RGB565 image. */
class PicConcat2B_Impl: public PicConcatPiP, virtual public PicInPic2B_Impl
{
public:
		// Construction and destruction.
	PicConcat2B_Impl(void) {}
	PicConcat2B_Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient);

		// Interface.
	virtual int Concat(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);

        virtual int GetVideoFormat(void) const {return 16;}

        //virtual void SetOutDimensions(int width, int height);

protected:
}; //end PicConcat2B_Impl.


#endif	// _PICCONCAT_2B_IMPL_H

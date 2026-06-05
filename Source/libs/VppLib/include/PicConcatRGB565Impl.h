/** @file

MODULE:		PicConcatRGB565Impl

TAG:		PCRGB565I

FILE NAME:	PicConcatRGB565Impl.h

DESCRIPTION:	An RGB565 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB565 
		image to another packed RGB565 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICCONCATRGB565IMPL_H
#define _PICCONCATRGB565IMPL_H

#include "PicConcatBase.h"
#include "PicInPic2B_Impl.h"


/** \ingroup ImageLib An RGB565 implementation derived from the general
 * PicConcatBase() class. Concatenate a packed RGB565 
 * image	to another packed RGB565 image. */
class PicConcatRGB565Impl: public PicConcatBase, public PicInPic2B_Impl
{
public:
	// Construction and destruction.
	PicConcatRGB565Impl(void);
	PicConcatRGB565Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient);

	// Interface.
	virtual int Concat(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);

        virtual int GetVideoFormat(void) const {return 16;}

        virtual void SetOutDimensions(int width, int height);

protected:
}; //end PicConcatRGB565Impl.


#endif _PICCONCATRGB565IMPL_H

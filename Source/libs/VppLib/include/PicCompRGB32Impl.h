/** @file

MODULE: 	PicCompRGB32Impl

TAG:		PCRGB32I
                
FILE NAME:	PicCompRGB32Impl.h

DESCRIPTION:	An RGB32 implementation derived from the general
		PicCompareBase() class. Compare a packed RGB32 
		image to another packed RGB32 image.

REVISION HISTORY: 

COPYRIGHT:	(c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

LICENSE:	Software License Agreement (BSD License)

===========================================================================*/
#ifndef _PICCOMPARERGB32IMPL_H
#define _PICCOMPARERGB32IMPL_H

#include "PicCompareBase.h"
#include "PicInPicRGB32Impl.h"

/** \ingroup ImageLib
 * An RGB32 implementation derived from the general
 * PicCompareBase() class. Concatenate a packed RGB32 
 * image to another packed RGB32 image. */
class PicCompareRGB32Impl: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareRGB32Impl(void) {}
	PicCompareRGB32Impl(int width, int height): PicCompareBase(width,height) {}
	virtual ~PicCompareRGB32Impl(void) {}

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 32;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareRGB32Impl.


#endif _PICCOMPARERGB32IMPL_H

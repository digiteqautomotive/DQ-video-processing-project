/** @file

MODULE: 	PicCompI420Impl

TAG:		PCI420I
                
FILE NAME:	PicCompI420Impl.h

DESCRIPTION:	An I420 implementation derived from the general
		PicCompareBase() class. Compare a packed I420 
		image to another packed I420 image.

REVISION HISTORY: 

COPYRIGHT:	(c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

LICENSE:	Software License Agreement (BSD License)

===========================================================================*/
#ifndef _PICCOMPAREI420IMPL_H
#define _PICCOMPAREI420IMPL_H

#include "PicCompareBase.h"


/** \ingroup ImageLib
 * An I420 implementation derived from the general
 * PicCompareBase() class. Concatenate a packed I420 
 * image to another packed I420 image. */
class PicCompareI420Impl: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareI420Impl(void) {}
	PicCompareI420Impl(int width, int height): PicCompareBase(width,height) {}
	virtual ~PicCompareI420Impl(void) {}

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 12;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareI420Impl.


#endif _PICCOMPAREI420IMPL_H

/** @file

MODULE: 	PicCompYUYV_Impl

TAG:		PCYUYV_I
                
FILE NAME:	PicCompYUYV_Impl.h

DESCRIPTION:	An YUYV implementation derived from the general
		PicCompareBase() class. Compare a packed YUYV
		image to another packed YUYV image.

REVISION HISTORY: 

COPYRIGHT:	(c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

LICENSE:	Software License Agreement (BSD License)

===========================================================================*/
#ifndef _PICCOMPAREYUYV_IMPL_H
#define _PICCOMPAREYUYV_IMPL_H

#include "PicCompareBase.h"


/** \ingroup ImageLib
 * An YUYV_ implementation derived from the general
 * PicCompareBase() class. Concatenate a packed YUYV_ 
 * image to another packed YUYV_ image. */
class PicCompareYUYV_Impl: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareYUYV_Impl(void) {}
	PicCompareYUYV_Impl(int width, int height): PicCompareBase(width,height) {} 
	virtual ~PicCompareYUYV_Impl(void) {}

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 16;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareYUYV_Impl.


#endif _PICCOMPAREYUYV_IMPL_H

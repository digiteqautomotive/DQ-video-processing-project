/** @file

MODULE: 	PicCompRGB24Impl

TAG:		PCRGB24I
                
FILE NAME:	PicCompRGB24Impl.h

DESCRIPTION:	An RGB24 implementation derived from the general
		PicCompareBase() class. Compare a packed RGB24 
		image to another packed RGB24 image.

REVISION HISTORY: 

COPYRIGHT:	(c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

LICENSE:	Software License Agreement (BSD License)

===========================================================================*/
#ifndef _PICCOMPARERGB24IMPL_H
#define _PICCOMPARERGB24IMPL_H

#include "PicCompareBase.h"


/** \ingroup ImageLib
 * An RGB24 implementation derived from the general
 * PicCompareBase() class. Concatenate a packed RGB24 
 * image to another packed RGB24 image. */
class PicCompareRGB24Impl: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareRGB24Impl(void);
	PicCompareRGB24Impl(int width, int height);
	virtual ~PicCompareRGB24Impl(void);

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 24;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareRGB24Impl.


#endif _PICCOMPARERGB24IMPL_H

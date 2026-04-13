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
#include "PicInPicRGB24Impl.h"

/** \ingroup ImageLib
 * An RGB24 implementation derived from the general
 * PicCompareBase() class. Concatenate a packed RGB24 
 * image to another packed RGB24 image. */
class PicCompareRGB24Impl: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareRGB24Impl(void);
	PicCompareRGB24Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd);
	virtual ~PicCompareRGB24Impl(void);

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 32;}

protected:
	// Support methods.
	virtual int Create(void);

protected:
	// Members.
	PicInPicRGB24Impl* _pPicInPic;	/// Use a pic-in-pic obj to place the images.

}; //end PicCompareRGB24Impl.


#endif _PICCOMPARERGB24IMPL_H

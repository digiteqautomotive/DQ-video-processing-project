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
#ifndef _PICCOMPARERGB32_SSE2_H
#define _PICCOMPARERGB32_SSE2_H

#include "PicCompareBase.h"
#include "PicInPicBase.h"

/** \ingroup ImageLib
 * An RGB32 implementation derived from the general
 * PicCompareBase() class. Concatenate a packed RGB32 
 * image to another packed RGB32 image. */
class PicCompareRGB32_SSE2: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareRGB32_SSE2(void) {}
	PicCompareRGB32_SSE2(int width, int height): PicCompareBase(width,height) {}
	virtual ~PicCompareRGB32_SSE2(void) {}

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 32;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareRGB32_SSE2.


/** \ingroup ImageLib
 * An RGB24 implementation derived from the general
 * PicCompareBase() class. Concatenate a packed RGB24 
 * image to another packed RGB24 image. */
class PicCompareRGB24_SSE2: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareRGB24_SSE2(void) {}
	PicCompareRGB24_SSE2(int width, int height): PicCompareBase(width,height) {}
	virtual ~PicCompareRGB24_SSE2(void) {}

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 24;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareRGB24_SSE2.


/** \ingroup ImageLib
 * An YUYV implementation derived from the general
 * PicCompareBase() class. Concatenate a packed YUYV 
 * image to another packed YUYV image. */
class PicCompareYUYV_SSE2: public PicCompareBase
{
public:
	// Construction and destruction.
	PicCompareYUYV_SSE2(void) {}
	PicCompareYUYV_SSE2(int width, int height): PicCompareBase(width,height) {}
	virtual ~PicCompareYUYV_SSE2(void) {}

	// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);
        virtual int GetVideoFormat(void) const {return 24;}

protected:
	// Support methods.

	// Members.

}; //end PicCompareYUYV_SSE2.


#endif _PICCOMPARERGB32_SSE2_H

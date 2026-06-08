/** @file
MODULE:			PicInPic8B_Impl

TAG:			PIPRGB24I

FILE NAME:		PicInPic8B_Impl.h

DESCRIPTION:		An RGB24 implementation derived from the general
			PicInPicBase() class. Insert a packed RGB24 image
			into another packed RGB24 image.

COPYRIGHT:		Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:		Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICINPIC_8B_IMPL_H
#define _PICINPIC_8B_IMPL_H

#include "PicInPicBase.h"


/** \ingroup ImageLib
 * An RGB24 implementation derived from the general
 * PicInPicBase() class. Insert a packed RGB24 image
 * into another packed RGB24 image. */
class PicInPic8B_Impl: public PicInPicBase
{
public:
		// Construction and destruction.
	PicInPic8B_Impl(void) {}
	PicInPic8B_Impl(int width, int height, int subWidth, int subHeight, int xPos, int yPos): PicInPicBase(width,height,subWidth,subHeight,xPos,yPos) {}

protected:
		// Interface.
	virtual bool isVideoFormatSupported(const GUID &VideoFormat) const;
	virtual void DoInsert(const void* pSubImg, void* pImg, bool VFlip=false) const;
        virtual void DoInsertWithBorder(const void* pSubImg, void* pImg, bool VFlip=false) const;
};


#endif //end _PicInPic8B_Impl_H.

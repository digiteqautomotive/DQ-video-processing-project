/** @file

MODULE:			PicCropper6B_Impl

FILE NAME:		PicCropper6B_Impl.h

DESCRIPTION:		An RGB24 implementation derived from the general
			PicCropperBase() class. Crop a packed RGB24 image
			into another packed RGB24 image.
					  					  
LICENSE: Software License Agreement (BSD License)

(c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.
===========================================================================*/
#ifndef _PICCROPPER_6B_IMPL_H
#define _PICCROPPER_6B_IMPL_H

#include "PicCropperBase.h"

/** \ingroup ImageLib
 * An RGB24 implementation derived from the general
 * PicCropperBase() class. Crop a packed RGB24 image
 * into another packed RGB24 image. */
class PicCropper6B_Impl: public PicCropperBase
{
public:
	// Construction and destruction.
	PicCropper6B_Impl(void) { }
	PicCropper6B_Impl(int widthOut, int heightOut, int widthIn, int heightIn, int left, int right, int top, int bottom): PicCropperBase(widthOut,heightOut,widthIn,heightIn,left,right,top,bottom) { }

	// Interface.
	virtual bool isVideoFormatSupported(const GUID &VideoFormat) const;
	virtual int Crop(const void* pInImg, void* pImg, bool VFlip=false);
}; //end PicCropper6B_Impl.

#endif	// _PICCROPPER_6B_IMPL_H

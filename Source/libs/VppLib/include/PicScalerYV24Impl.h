/** @file

MODULE				: PicScalerYV24Impl

FILE NAME			: PicScalerYV24Impl.h

DESCRIPTION			: A YV24 planar implementation derived from the 
				general PicScalerBase() class. Scale a YUV420P
				planar image to the dimensions of another YUV420P
				planar image.

					  		  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, designed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#ifndef _PICSCALERYV24PIMPL_H
#define _PICSCALERYV24PIMPL_H


#include "PicScalerBase.h"


/** \ingroup ImageLib
 * A YUV420P planar implementation derived from the 
 * general PicScalerBase() class. Scale a YUV420P
 * planar image to the dimensions of another YUV420P
 * planar image. */
class PicScalerYV24Impl: public PicScalerBase
{
public:
		// Construction and destruction.
	PicScalerYV24Impl(void) {}
	PicScalerYV24Impl(int widthOut, int heightOut, int widthIn, int heightIn): PicScalerBase(widthOut,heightOut,widthIn,heightIn) { }
	virtual ~PicScalerYV24Impl(void) {}	

		// Interface.
	virtual int GetVideoFormat(void) const {return 12;}
        bool isVideoFormatSupported(const GUID &VideoFormat) const;
	int Scale(void* pOutImg, const void* pInImg, bool VFlip=false);

}; //end PicScalerYV24Impl.


#endif	// _PicScalerYV24Impl_H

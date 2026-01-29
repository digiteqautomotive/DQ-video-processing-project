/** @file

MODULE				: PicScalerRGB565Impl

FILE NAME			: PicScalerRGB565Impl.h

DESCRIPTION			: An RGB565 implementation derived from the general
					PicScalerBase() class. Scale a packed RGB565 image
					to the dimensions of another packed RGB565 image.
				  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Jaroslav Fojtik		All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/
#ifndef _PICSCALERRGB565IMPL_H
#define _PICSCALERRGB565IMPL_H

#pragma once

#include "PicScalerBase.h"

/**
 * \ingroup ImageLib
 * An RGB565 implementation derived from the general
 * PicScalerBase() class. Scale a packed RGB565 image
 * to the dimensions of another packed RGB565 image.
 */
class PicScalerRGB565Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	PicScalerRGB565Impl(void) { }
	PicScalerRGB565Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~PicScalerRGB565Impl(void) {}

	virtual int GetVideoFormat(void) const {return 32;}
        virtual bool isVideoFormatSupported(const GUID &VideoFormat) const;

	// Interface.
	int Scale(void* pOutImg, const void* pInImg, bool VFlip=false);

};//end PicScalerRGB565Impl.


#endif	// _PICSCALERRGB565IMPL_H

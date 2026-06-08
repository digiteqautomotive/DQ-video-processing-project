/** @file

MODULE:			PicCropper2B_Impl

FILE NAME:		PicCropper2B_Impl.h

DESCRIPTION:		An RGB565 implementation derived from the general
			PicCropperBase() class. Crop a packed RGB565 image
			into another packed RGB565 image.			  
					  
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
#ifndef _PICCROPPER_2B_IMPL_H
#define _PICCROPPER_2B_IMPL_H

#include "PicCropperBase.h"


/** \ingroup ImageLib
 * An RGB565 implementation derived from the general
 * PicCropperBase() class. Crop a packed RGB565 image
 * into another packed RGB565 image. */
class PicCropper2B_Impl: public PicCropperBase
{
public:
	// Construction and destruction.
	PicCropper2B_Impl(void) { }
	PicCropper2B_Impl(int widthOut, int heightOut, int widthIn, int heightIn, int left, int right, int top, int bottom): PicCropperBase(widthOut,heightOut,widthIn,heightIn,left,right,top,bottom) { }
	virtual ~PicCropper2B_Impl(void) {}

	// Interface.
	virtual bool isVideoFormatSupported(const GUID &VideoFormat) const;
	virtual int Crop(const void* pInImg, void* pImg, bool VFlip=false);
}; //end PicCropper2B_Impl.


#endif	// _PICCROPPER_2B_IMPL_H

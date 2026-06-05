/** @file

MODULE:			PicInPic2B_Impl

TAG:			PIPRGB565I

FILE NAME:		PicInPic2B_Impl.h

DESCRIPTION:		A 2 BYTE implementation derived from the general
			PicInPicBase() class. Insert a packed RGB555 or RGB565 image
			into another packed RGB565 image.

COPYRIGHT:		Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:		Software License Agreement (BSD License)


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
#ifndef _PICINPIC_2B_IMPL_H
#define _PICINPIC_2B_IMPL_H

#include "PicInPicBase.h"


/** \ingroup ImageLib
 * An RGB565 implementation derived from the general
 * PicInPicBase() class. Insert a packed RGB565 image
 * into another packed RGB565 image. */
class PicInPic2B_Impl: public PicInPicBase
{
public:
		// Construction and destruction.
	PicInPic2B_Impl(void) {}
	PicInPic2B_Impl(int width, int height, int subWidth, int subHeight, int xPos, int yPos): PicInPicBase(width,height,subWidth,subHeight,xPos,yPos) {}
	virtual ~PicInPic2B_Impl(void) {}

protected:
		// Interface.
	bool isVideoFormatSupported(const GUID &VideoFormat) const;
	void DoInsert(const void* pSubImg, void* pImg, bool VFlip=false) const;
	void DoInsertWithBorder(const void* pSubImg, void* pImg, bool VFlip=false) const;
};


#endif	 //end _PICINPIC_2B_IMPL_H.

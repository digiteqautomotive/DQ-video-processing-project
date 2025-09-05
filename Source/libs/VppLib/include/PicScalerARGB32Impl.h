/** @file

MODULE				: PicScalerARGB32Impl

FILE NAME			: PicScalerARGB32Impl.h

DESCRIPTION			: An ARGB32 implementation derived from the general
					PicScalerBase() class. Scale a packed ARGB32 image
					to the dimensions of another packed ARGB32 image.
					  
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025 Jaroslav Fojtik
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

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

===========================================================================
*/
#ifndef _PICSCALERARGB32IMPL_H
#define _PICSCALERARGB32IMPL_H

#pragma once

#include "PicScalerBase.h"

/**
 * \ingroup ImageLib
 * An ARGB32 implementation derived from the general
 * PicScalerBase() class. Scale a packed ARGB32 image
 * to the dimensions of another packed ARGB32 image.
 */
class PicScalerARGB32Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	PicScalerARGB32Impl(void) { }
	PicScalerARGB32Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~PicScalerARGB32Impl(void) {}

	virtual int GetVideoFormat(void) const {return 32;}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

};//end PicScalerARGB32Impl.


#endif	// _PICSCALERARGB32IMPL_H

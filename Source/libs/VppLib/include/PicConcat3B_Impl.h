/** @file

MODULE:		PicConcat3B_Impl

TAG:		PCRGB24I

FILE NAME:	PicConcat3B_Impl.h

DESCRIPTION:	An RGB24 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB24 
		image to another packed RGB24 image.

COPYRIGHT:	(c)CSIR 2007-2017
		(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:	Software License Agreement (BSD License)

RESTRICTIONS:	Redistribution and use in source and binary forms, with or without 
		modification, are permitted provided that the following conditions 
		are met:

		* Redistributions of source code must retain the above copyright notice, 
		this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright notice, 
		this list of conditions and the following disclaimer in the documentation 
		and/or other materials provided with the distribution.
		* Neither the name of the CSIR nor the names of its contributors may be used 
		to endorse or promote products derived from this software without specific 
		prior written permission.

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
#ifndef _PICCONCAT_3B_IMPL_H
#define _PICCONCAT_3B_IMPL_H

#include "PicConcatPiP.h"
#include "PicInPic3B_Impl.h"


/** \ingroup ImageLib An RGB24 implementation derived from the general
 * PicConcatBase() class. Concatenate a packed RGB24 
 * image	to another packed RGB24 image. */
class PicConcat3B_Impl: public PicConcatPiP, virtual public PicInPic3B_Impl
{
public:
	// Construction and destruction.
	PicConcat3B_Impl(void) {}
	PicConcat3B_Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient);

	// Interface.
	//virtual int Concat(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);

        virtual int GetVideoFormat(void) const {return 24;}
        //virtual void SetOutDimensions(int width, int height);

        //bool isVideoFormatSupported(const GUID & G) const {return PicInPic3B_Impl::isVideoFormatSupported(G);}
        //void DoInsert(const void *P1,void *P2,bool b) const {PicInPic3B_Impl::DoInsert(P1,P2,b);}
        //void DoInsertWithBorder(const void * P1,void *P2,bool b) const {PicInPic3B_Impl::DoInsertWithBorder(P1,P2,b);}

}; //end PicConcat3B_Impl.


#endif	// _PICCONCAT_3B_IMPL_H

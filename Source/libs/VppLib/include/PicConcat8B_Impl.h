/** @file

MODULE:		PicConcat8B_Impl

TAG:		PCRGB24I

FILE NAME:	PicConcat8B_Impl.h

DESCRIPTION:	An RGB24 implementation derived from the general
		PicConcatBase() class. Concatenate a packed RGB24 
		image to another packed RGB24 image.

COPYRIGHT:	(c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICCONCAT_8B_IMPL_H
#define _PICCONCAT_8B_IMPL_H

#include "PicConcatPiP.h"
#include "PicInPic8B_Impl.h"


/** \ingroup ImageLib An RGB24 implementation derived from the general
 * PicConcatBase() class. Concatenate a packed RGB24 
 * image	to another packed RG16 image. */
class PicConcat8B_Impl: public PicConcatPiP, virtual public PicInPic8B_Impl
{
public:
	// Construction and destruction.
	PicConcat8B_Impl(void) {}
	PicConcat8B_Impl(int width, int height, int width1st, int height1st, int width2nd, int height2nd, E_ORIENT orient);

	// Interface.
	//virtual int Concat(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false);

        virtual int GetVideoFormat(void) const {return 64;}
        //virtual void SetOutDimensions(int width, int height);

        //bool isVideoFormatSupported(const GUID & G) const {return PicInPic8B_Impl::isVideoFormatSupported(G);}
        //void DoInsert(const void *P1,void *P2,bool b) const {PicInPic8B_Impl::DoInsert(P1,P2,b);}
        //void DoInsertWithBorder(const void * P1,void *P2,bool b) const {PicInPic8B_Impl::DoInsertWithBorder(P1,P2,b);}

}; //end PicConcat8B_Impl.


#endif // _PICCONCAT_8B_IMPL_H

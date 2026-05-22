/** @file

MODULE:		PicCompareBase

TAG:		PCB

FILE NAME:	PicCompareBase.h

DESCRIPTION:	This class is the base class defining the minimum interface	
		and properties for all derived implementations to compare
		2 images. All memory new/delete must be handled outside 
		of these objects. The compare method is pure virtual and must be 
		implemented by derived classes.

COPYRIGHT:	(c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#ifndef _PICCOMPAREBASE_H
#define _PICCOMPAREBASE_H

/** \ingroup ImageLib
 * This class is the base class defining the minimum interface	
 * and properties for all derived implementations to concatenate
 * a 2nd image on any boundary of the 1st image. The instantiation 
 * process must set the parameters and limits to the images such 
 * that the Concat()implementation only requires source image 
 * data pointers. All memory new/delete must be handled outside 
 * of these objects. The concat method is pure virtual and must be 
 * implemented by derived classes. */
class PicCompareBase
{
public:

		// Construction and destruction.
	PicCompareBase(void):  _width(0), _height(0) {}
	PicCompareBase(int width, int height);
	virtual ~PicCompareBase(void) {}

		// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false) = 0;
        virtual int GetVideoFormat(void) const = 0;

		// Member interface.
	unsigned GetOutWidth(void) const	{ return(_width); }
	unsigned GetOutHeight(void) const 	{ return(_height); }
  
	void SetDimensions(int width, int height);

protected:
		// Private methods.

		// Members.
	unsigned _width;		/// Of total output image that contains both input images.
	unsigned _height;
}; //end PicCompareBase.


#endif	// _PICCOMPAREBASE_H

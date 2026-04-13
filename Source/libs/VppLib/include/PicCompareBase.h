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
	PicCompareBase(void) {_width = 0; _height = 0; _width1st = 0; _height1st = 0; _width2nd = 0; _height2nd = 0;  }
	PicCompareBase(int width, int height, int width1st, int height1st, int width2nd, int height2nd) 
	{	_width = width; _height = height; _width1st = width1st; _height1st = height1st; _width2nd = width2nd; _height2nd = height2nd;}
	virtual ~PicCompareBase(void) {}

		// Interface.
	virtual int Compare(const void* pImg1st, const void* pImg2nd, void* pImg, bool VFlip=false) = 0;
        virtual int GetVideoFormat(void) const = 0;

		// Member interface.
	int GetOutWidth(void) const	{ return(_width); }
	int GetOutHeight(void) const 	{ return(_height); }
	int Get1stWidth(void) const	{ return(_width1st); }
	int Get1stHeight(void) const 	{ return(_height1st); }
	int Get2ndWidth(void) const	{ return(_width2nd); }
	int Get2ndHeight(void) const	{ return(_height2nd); }
  
	void SetOutDimensions(int width, int height)		{_width = width; _height = height; }
	void Set1stDimensions(int width1st, int height1st)	{_width1st = width1st; _height1st = height1st; }
	void Set2ndDimensions(int width2nd, int height2nd)	{_width2nd = width2nd; _height2nd = height2nd; }

		// Private methods.
protected:

protected:
		// Members.
	int _width;		/// Of total output image that contains both input images.
	int _height;
	int _width1st;		/// Of 1st base image.
	int _height1st;
	int _width2nd;		/// Of 2nd image to concat to 1st image.
	int _height2nd;
};//end PicCompareBase.

#endif	// _PICCOMPAREBASE_H

/** @file

MODULE:		PicCompareBase

TAG:		PCB

FILE NAME:	PicCompareBase.cpp

DESCRIPTION:	This class is the base class defining the minimum interface	
		and properties for all derived implementations to compare
		2 images. All memory new/delete must be handled outside 
		of these objects. The compare method is pure virtual and must be 
		implemented by derived classes.

COPYRIGHT:	(c) 2025-2026 Digiteq Automotive, written by Jaroslav Fojtik		All rights reserved.

LICENSE:	Software License Agreement (BSD License)
===========================================================================*/
#include "PicCompareBase.h"

#include <math.h>


PicCompareBase::PicCompareBase(int width, int height): _width(labs(width)), _height(labs(height)) 
{}


void PicCompareBase::SetDimensions(int width, int height)
{
  _width = labs(width);
  _height = labs(height);
}

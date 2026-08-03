/** @file

MODULE:		PicRotateBase

FILE NAME:	PicRotateBase.cpp

DESCRIPTION:
                     
LICENSE: Software License Agreement (BSD License)

(c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.
===========================================================================*/
#include "PicRotateBase.h"

#include <stdlib.h>


void PicRotateBase::SetInDimensions(int nWidth, int nHeight)
{
  m_nWidth = labs(nWidth);
  m_nHeight = labs(nHeight);
}

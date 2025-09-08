/** @file

MODULE                : PicRotateRGB32Impl

FILE NAME             : PicRotateRGB32Impl.cpp

DESCRIPTION           :
                     
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
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
#include "PicRotateRGB32Impl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


PicRotateRGB32Impl::PicRotateRGB32Impl()
{;}

PicRotateRGB32Impl::~PicRotateRGB32Impl()
{;}

int PicRotateRGB32Impl::BytesPerPixel()
{
	return 4;
}


#ifdef USE_ASM
extern "C" {
void Flip32_2(unsigned Width, unsigned Height, const void *ptr_in, void *ptr_out);
void Rotate32_180(unsigned Width, unsigned Height, const void *ptr_in, void *ptr_out);
}
#endif


bool PicRotateRGB32Impl::Rotate(const void* pInImg, void* pOutImg)
{
  if (!pInImg || !pOutImg) return false;
  switch (m_eMode)
  {
    case ROTATE_NONE:
		{
			memcpy(pOutImg, pInImg, labs(m_nWidth*m_nHeight)*4);
			return true;
		}
    case ROTATE_90_DEGREES_CLOCKWISE:
		{			
			const unsigned AbsHeight = labs(m_nHeight);
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg + 4*(m_nWidth-1)*AbsHeight;
			
			for(int i = 0; i < m_nWidth; ++i, pSrc+=4)
			{
				const BYTE* pSrcCol = pSrc;
				unsigned j = AbsHeight;
				while(j-- > 0)				
				{
					*(__int32*)(pDest) = *(__int32*)(pSrcCol);
					pSrcCol += (4*m_nWidth);
					pDest += 4;
				}
				// Copy pixels in same column to their destinations				
				pDest -= 8 * AbsHeight;
			}
			return true;
		}
	case ROTATE_180_DEGREES_CLOCKWISE:
		{
#ifdef USE_ASM
		        Rotate32_180(m_nWidth,labs(m_nHeight),pInImg,pOutImg);
#else
			const BYTE* pSrc = (const BYTE*)pInImg;
			const int nTotalPixels = labs(m_nWidth * m_nHeight);
			const int nLastIndex = (nTotalPixels - 1) * 4;
			BYTE* pDest = (BYTE*)pOutImg + nLastIndex;
			for (int i = 0; i < nTotalPixels; ++i, pSrc += 4, pDest -= 4)
			{
				*(__int32*)(pDest) = *(__int32*)(pSrc);
			}
#endif
			return true;
		}

	case ROTATE_270_DEGREES_CLOCKWISE:
		{
			const unsigned iTargetRowLength = labs(m_nHeight) * 4;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + iTargetRowLength - 4;
			unsigned y = labs(m_nHeight);
			while(y-- > 0)
			{
				BYTE* pDestPixel = pDest;
				for (int x = 0; x < m_nWidth; ++x, pSrc += 4, pDestPixel += iTargetRowLength)
				{				
				  *(__int32*)(pDestPixel) = *(__int32*)(pSrc);
				}
				pDest -= 4;
			}
			return true;
		}
	case ROTATE_FLIP_VERTICAL:
		{
			// Code to flip image
			const unsigned nRowLength = labs(m_nWidth) * 4;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + ((labs(m_nHeight) - 1) * nRowLength);
			unsigned i = labs(m_nHeight);
			while(i-- > 0)
			{
			  memcpy(pDest, pSrc, nRowLength);
			  pSrc += nRowLength;
			  pDest -= nRowLength;
			}
			return true;
		}
	case ROTATE_FLIP_HORIZONTAL:
		{
#ifdef USE_ASM
			Flip32_2(m_nWidth, labs(m_nHeight), pInImg, pOutImg);
#else
			const unsigned iRowLength = labs(m_nWidth) * 4;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + iRowLength - 4;

			unsigned y = labs(m_nHeight);
		        while(y-- > 0)
		        {
			  BYTE* pDestPixel = pDest;
			  for ( int x = 0; x < m_nWidth; x++, pSrc += 4, pDestPixel-=4 )
			  {
			    *(__int32*)(pDestPixel) = *(__int32*)(pSrc);
			  }
			  pDest += iRowLength;
			}
#endif
			return true;
		}
	case ROTATE_FLIP_DIAGONALLY:
		{
			const BYTE* pSrc = (const BYTE*)pInImg;
			const unsigned AbsHeight = labs(m_nHeight);
			for (int i = 0; i < m_nWidth; ++i, pSrc += 4)
			{
				int nNewRow = (m_nWidth - i);//Index Base 1
				// Select target row to be one higher and subtract sizeof RGB block to be at last index of row
				++nNewRow;
				int nNewIndex = ((nNewRow - 1) * m_nHeight);// Base 1
				// Adjust index to previous row
				--nNewIndex;
				const BYTE* pSrcCol = pSrc;
				BYTE* pDest = (BYTE*)pOutImg + (nNewIndex*4);
				for (unsigned j=0; j<AbsHeight; ++j, pSrcCol+=(4*m_nWidth), pDest-=4)
				{
				   *(__int32*)(pDest) = *(__int32*)(pSrcCol);
				}
				// Copy pixels in same column to their destinations
			}
			return true;
		}
	default:
		{
			// Unimplemented
			return false;
		}
	}
}

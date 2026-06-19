/** @file

MODULE                : PicRotateBPP2Impl

FILE NAME             : PicRotateBPP32Impl.cpp

DESCRIPTION           :
                     
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
Copyright (c) 2025-2026 Jaroslav Fojtik		All rights reserved.

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
===========================================================================*/
#include "PicRotateBPP64Impl.h"

#include <string.h>


PicRotateBPP64Impl::PicRotateBPP64Impl()
{}

int PicRotateBPP64Impl::BytesPerPixel(void) const
{
  return 8;
}


#ifdef USE_ASM
extern "C" {
void Flip64_2(unsigned Width, unsigned Height, const void *ptr_in, void *ptr_out);
}
#endif


bool PicRotateBPP64Impl::Rotate(const void* pInImg, void* pOutImg)
{
  if (!pInImg || !pOutImg) return false;
  switch (m_eMode)
  {
    case ROTATE_NONE:
		{
			memcpy(pOutImg, pInImg, m_nWidth*m_nHeight*8);
			return true;
		}
    case ROTATE_90_DEGREES_CLOCKWISE:
		{
//#ifdef USE_ASM
//		        Rotate32_90(m_nWidth,m_nHeight,pInImg,pOutImg);
//#else			
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg + 8*(m_nWidth-1)*m_nHeight;
			
			for(unsigned i=0; i<m_nWidth; ++i, pSrc+=8)
			{
				const BYTE* pSrcCol = pSrc;
				unsigned j = m_nHeight;
				while(j-- > 0)				
				{
					*(__int64*)(pDest) = *(__int64*)(pSrcCol);
					pSrcCol += (8*m_nWidth);
					pDest += 8;
				}
				// Copy pixels in same column to their destinations				
				pDest -= 8 * m_nHeight;
			}
//#endif
			return true;
		}
	case ROTATE_180_DEGREES_CLOCKWISE:
		{
//#ifdef USE_ASM
//		        Rotate32_180(m_nWidth,m_nHeight,pInImg,pOutImg);
//#else
			const BYTE* pSrc = (const BYTE*)pInImg;
			const unsigned nTotalPixels = m_nWidth * m_nHeight;
			const int nLastIndex = (nTotalPixels - 1) * 8;
			BYTE* pDest = (BYTE*)pOutImg + nLastIndex;
			for(unsigned i = 0; i < nTotalPixels; ++i, pSrc+=8, pDest-=8)
			{
				*(__int64*)(pDest) = *(__int64*)(pSrc);
			}
//#endif
			return true;
		}

	case ROTATE_270_DEGREES_CLOCKWISE:	// Continual writing and scaterred reading is faster than continual reading and scaterred writing.
		{
//#ifdef USE_ASM
//		        Rotate32_270(m_nWidth,m_nHeight,pInImg,pOutImg);
//#else			
			const unsigned y_src = m_nHeight;
			unsigned y_dst = m_nWidth;
			const unsigned x_src8 = 8*y_dst;
			const BYTE* pSrc = (const BYTE*)pInImg + 8*(y_src-1)*m_nWidth;
			BYTE* pDest = (BYTE*)pOutImg;
			while(y_dst-- > 0)
			{
				const BYTE* pSrcPos = pSrc;
				int x_dst = y_src;
				while(x_dst-- > 0)
				{				
				  *(__int64*)(pDest) = *(__int64*)(pSrcPos);
				  pDest += 8;
				  pSrcPos -= x_src8;
				}
				pSrc += 8;
			}
//#endif
			return true;
		}
	case ROTATE_FLIP_VERTICAL:
		{
			// Code to flip image
			const unsigned nRowLength = m_nWidth * 8;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + ((m_nHeight - 1) * nRowLength);
			unsigned i = m_nHeight;
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
			Flip64_2(m_nWidth, m_nHeight, pInImg, pOutImg);
#else
			const unsigned iRowLength = m_nWidth * 8;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + iRowLength - 8;

			unsigned y = m_nHeight;
		        while(y-- > 0)
		        {
			  BYTE* pDestPixel = pDest;
			  for ( int x = 0; x < m_nWidth; x++, pSrc+=8, pDestPixel-=8 )
			  {
			    *(__int64*)(pDestPixel) = *(__int64*)(pSrc);
			  }
			  pDest += iRowLength;
			}
#endif
			return true;
		}
	case ROTATE_FLIP_DIAGONALLY:
		{
			const BYTE* pSrc = (const BYTE*)pInImg;
			const unsigned AbsHeight = m_nHeight;
			for(unsigned i=0; i<m_nWidth; ++i, pSrc+=8)
			{
				int nNewRow = (m_nWidth - i);//Index Base 1
				// Select target row to be one higher and subtract sizeof RGB block to be at last index of row
				++nNewRow;
				int nNewIndex = ((nNewRow - 1) * m_nHeight);// Base 1
				// Adjust index to previous row
				--nNewIndex;
				const BYTE* pSrcCol = pSrc;
				BYTE* pDest = (BYTE*)pOutImg + (nNewIndex*8);
				for(unsigned j=0; j<m_nHeight; ++j, pSrcCol+=(8*m_nWidth), pDest-=8)
				{
				   *(__int64*)(pDest) = *(__int64*)(pSrcCol);
				}
				// Copy pixels in same column to their destinations
			}
			return true;
		}

	case ROTATE_90_DEGS_CCK_VFLIP:
		{
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg;		// + 4*(m_nWidth-1)*AbsHeight;
			
			for(int i=0; i<m_nWidth; ++i, pSrc+=8)
			{
				const BYTE* pSrcCol = pSrc;
				unsigned j = m_nHeight;
				while(j-- > 0)				
				{
					*(__int64*)(pDest) = *(__int64*)(pSrcCol);
					pSrcCol += 8*m_nWidth;
					pDest += 8;
				}
				// Copy pixels in same column to their destinations				
				//pDest -= 8 * AbsHeight;
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

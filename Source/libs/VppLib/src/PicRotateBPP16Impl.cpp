/** @file

MODULE                : PicRotateBPP16Impl

FILE NAME             : PicRotateBPP16Impl.cpp

DESCRIPTION           :
                     
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Jaroslav Fojtik		All rights reserved.

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
#include "PicRotateBPP16Impl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int PicRotateBPP16Impl::BytesPerPixel()
{
	return 2;
}


bool PicRotateBPP16Impl::Rotate(const void* pInImg, void* pOutImg)
{
  if (!pInImg || !pOutImg) return false;
  switch (m_eMode)
  {
    case ROTATE_NONE:
		{
			memcpy(pOutImg, pInImg, labs(m_nWidth*m_nHeight)*2);
			return true;
		}
    case ROTATE_90_DEGREES_CLOCKWISE:
		{
			const unsigned AbsHeight = labs(m_nHeight);
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg + 2*(m_nWidth-1)*AbsHeight;
			
			for(int i = 0; i < m_nWidth; ++i, pSrc+=2)
			{
				const BYTE* pSrcCol = pSrc;
				unsigned j = AbsHeight;
				while(j-- > 0)				
				{
					*(__int32*)(pDest) = *(__int32*)(pSrcCol);
					pSrcCol += (2*m_nWidth);
					pDest += 2;
				}
				// Copy pixels in same column to their destinations				
				pDest -= 8 * AbsHeight;
			}
			return true;
		}
	case ROTATE_180_DEGREES_CLOCKWISE:
		{
			const BYTE* pSrc = (const BYTE*)pInImg;
			const int nTotalPixels = labs(m_nWidth * m_nHeight);
			const int nLastIndex = (nTotalPixels - 1) * 2;
			BYTE* pDest = (BYTE*)pOutImg + nLastIndex;
			for (int i = 0; i < nTotalPixels; ++i, pSrc += 2, pDest -= 2)
			{
				*(__int32*)(pDest) = *(__int32*)(pSrc);
			}
			return true;
		}

	case ROTATE_270_DEGREES_CLOCKWISE:	// Continual writing and scaterred reading is faster than continual reading and scaterred writing.
		{
			const unsigned y_src = labs(m_nHeight);
			unsigned y_dst = labs(m_nWidth);
			const unsigned x_src2 = 2*y_dst;
			const BYTE* pSrc = (const BYTE*)pInImg + 2*(y_src-1)*m_nWidth;
			BYTE* pDest = (BYTE*)pOutImg;
			while(y_dst-- > 0)
			{
				const BYTE* pSrcPos = pSrc;
				int x_dst = y_src;
				while(x_dst-- > 0)
				{				
				  *(__int32*)(pDest) = *(__int32*)(pSrcPos);
				  pDest += 2;
				  pSrcPos -= x_src2;
				}
				pSrc += 2;
			}
			return true;
		}
	case ROTATE_FLIP_VERTICAL:
		{
			// Code to flip image
			const unsigned nRowLength = labs(m_nWidth) * 2;
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
			const unsigned iRowLength = labs(m_nWidth) * 2;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + iRowLength - 2;

			unsigned y = labs(m_nHeight);
		        while(y-- > 0)
		        {
			  BYTE* pDestPixel = pDest;
			  for ( int x = 0; x < m_nWidth; x++, pSrc += 2, pDestPixel-=2 )
			  {
			    *(__int32*)(pDestPixel) = *(__int32*)(pSrc);
			  }
			  pDest += iRowLength;
			}
			return true;
		}
	case ROTATE_FLIP_DIAGONALLY:
		{
			const BYTE* pSrc = (const BYTE*)pInImg;
			const unsigned AbsHeight = labs(m_nHeight);
			for (int i = 0; i < m_nWidth; ++i, pSrc += 2)
			{
				int nNewRow = (m_nWidth - i);//Index Base 1
				// Select target row to be one higher and subtract sizeof RGB block to be at last index of row
				++nNewRow;
				int nNewIndex = ((nNewRow - 1) * m_nHeight);// Base 1
				// Adjust index to previous row
				--nNewIndex;
				const BYTE* pSrcCol = pSrc;
				BYTE* pDest = (BYTE*)pOutImg + (nNewIndex*2);
				for (unsigned j=0; j<AbsHeight; ++j, pSrcCol+=(2*m_nWidth), pDest-=2)
				{
				   *(__int32*)(pDest) = *(__int32*)(pSrcCol);
				}
				// Copy pixels in same column to their destinations
			}
			return true;
		}

	case ROTATE_90_DEGS_CCK_VFLIP:
		{
			const unsigned AbsHeight = labs(m_nHeight);
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg;		// + 2*(m_nWidth-1)*AbsHeight;
			
			for(int i = 0; i < m_nWidth; ++i, pSrc+=2)
			{
				const BYTE* pSrcCol = pSrc;
				unsigned j = AbsHeight;
				while(j-- > 0)				
				{
					*(__int32*)(pDest) = *(__int32*)(pSrcCol);
					pSrcCol += (2*m_nWidth);
					pDest += 2;
				}
				// Copy pixels in same column to their destinations				
				//pDest -= 8 * AbsHeight;
			}
		}
	default:
		{
			// Unimplemented
			return false;
		}
	}
}

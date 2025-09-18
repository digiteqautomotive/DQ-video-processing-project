/** @file

MODULE                : PicRotateRGB24Impl

FILE NAME             : PicRotateRGB24Impl.cpp

DESCRIPTION           :
                     
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
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
#include "PicRotateRGB24Impl.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


PicRotateRGB24Impl::PicRotateRGB24Impl()
{
	m_eMode = ROTATE_NONE;
}

PicRotateRGB24Impl::~PicRotateRGB24Impl()
{;}

int PicRotateRGB24Impl::BytesPerPixel()
{
	return 3;
}


#ifdef USE_ASM
extern "C"{
void Flip24_2(unsigned Width, unsigned Height, const void *ptr_in, void *ptr_out);
void Rotate24_180(unsigned Width, unsigned Height, const void *ptr_in, void *ptr_out);
}
#endif


bool PicRotateRGB24Impl::Rotate(const void* pInImg, void* pOutImg)
{
	if (!pInImg || !pOutImg) return false;
	
	switch (m_eMode)
	{
	case ROTATE_NONE:
		{
			memcpy(pOutImg, pInImg, labs(m_nWidth*m_nHeight)*3);
			return true;
		}
	case ROTATE_90_DEGREES_CLOCKWISE:
		{
			const unsigned AbsHeight = labs(m_nHeight);
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg + 3*(m_nWidth-1)*AbsHeight;

			for (int i = 0; i < m_nWidth; ++i, pSrc += 3)
			{
				const BYTE* pSrcCol = pSrc;
				for (int j = 0; j < m_nHeight; ++j)
				{
					pDest[0] = pSrcCol[0];
					pDest[1] = pSrcCol[1];
					pDest[2] = pSrcCol[2];
					pSrcCol += (3*m_nWidth);
					pDest += 3;
				}
				// Copy pixels in same column to their destinations
			        pDest -= 6 * AbsHeight;
			}
			return true;
		}
	case ROTATE_180_DEGREES_CLOCKWISE:
		{
#ifdef USE_ASM
		        Rotate24_180(labs(m_nWidth),labs(m_nHeight),pInImg,pOutImg);
#else
			const BYTE* pSrc = (const BYTE*)pInImg;
			const unsigned nTotalPixels = labs(m_nWidth * m_nHeight);
			const unsigned nLastIndex = (nTotalPixels - 1) * 3;
			BYTE* pDest = (BYTE*)pOutImg + nLastIndex;
			for (unsigned i = 0; i < nTotalPixels; ++i, pSrc += 3, pDest -= 3)
			{
				pDest[0] = pSrc[0];
				pDest[1] = pSrc[1];
				pDest[2] = pSrc[2];
			}
#endif
			return true;
		}

	case ROTATE_270_DEGREES_CLOCKWISE:	// Continual writing and scaterred reading is faster than continual reading and scaterred writing.
		{
			const unsigned y_src = labs(m_nHeight);
			unsigned y_dst = labs(m_nWidth);
			const unsigned x_src3 = 3*y_dst;
			const BYTE* pSrc = (const BYTE*)pInImg + 3*(y_src-1)*m_nWidth;
			BYTE* pDest = (BYTE*)pOutImg;
			while(y_dst-- > 0)
			{
				const BYTE* pSrcPos = pSrc;
				int x_dst = y_src;
				while(x_dst-- > 0)
				{
				  pDest[0] = pSrcPos[0];
			          pDest[1] = pSrcPos[1];
			          pDest[2] = pSrcPos[2];
				  pDest += 3;
				  pSrcPos -= x_src3;
				}
				pSrc += 3;
			}
			return true;
		}
	case ROTATE_FLIP_VERTICAL:
		{
			// Code to flip image
			const unsigned nRowLength = labs(m_nWidth) * 3;
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
			Flip24_2(labs(m_nWidth), labs(m_nHeight), pInImg, pOutImg);
#else
			const unsigned iRowLength = labs(m_nWidth) * 3;
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + iRowLength - 3;
                        for ( int y = 0; y < m_nHeight; y++ )
                        {
			   BYTE* pDestPixel = pDest;
                           for ( int x = 0; x < m_nWidth; x++, pSrc += 3, pDestPixel-=3 )
                           {
			     pDestPixel[0] = pSrc[0];
			     pDestPixel[1] = pSrc[1];
			     pDestPixel[2] = pSrc[2];			
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
			for (int i = 0; i < m_nWidth; ++i, pSrc += 3)
			{
				int nNewRow = (m_nWidth - i);//Index Base 1
				// Select target row to be one higher and subtract sizeof RGB block to be at last index of row
				++nNewRow;
				int nNewIndex = ((nNewRow - 1) * AbsHeight);// Base 1
				// Adjust index to previous row
				--nNewIndex;
				const BYTE* pSrcCol = pSrc;
				BYTE* pDest = (BYTE*)pOutImg + (nNewIndex*3);
				for (unsigned j = 0; j < AbsHeight; ++j, pSrcCol += (3*m_nWidth), pDest -= 3)
				{
					pDest[0] = pSrcCol[0];
					pDest[1] = pSrcCol[1];
					pDest[2] = pSrcCol[2];
				}
				// Copy pixels in same column to their destinations
			}
			return true;
		}

	case ROTATE_90_DEGS_CCK_VFLIP:
		{
			const unsigned AbsHeight = labs(m_nHeight);
			const BYTE* pSrc = (const BYTE*)pInImg;
			BYTE *pDest = (BYTE*)pOutImg;		// + 3*(m_nWidth-1)*AbsHeight;

			for (int i = 0; i < m_nWidth; ++i, pSrc += 3)
			{
				const BYTE* pSrcCol = pSrc;
				for (int j = 0; j < m_nHeight; ++j)
				{
					pDest[0] = pSrcCol[0];
					pDest[1] = pSrcCol[1];
					pDest[2] = pSrcCol[2];
					pSrcCol += 3*m_nWidth;
					pDest += 3;
				}
				// Copy pixels in same column to their destinations
			        //pDest -= 6 * AbsHeight;
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

/** @file

MODULE                : PicRotateRGB24Impl

FILE NAME             : PicRotateRGB24Impl.cpp

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
#include "PicRotateRGB24Impl.h"
#include <stdio.h>
#include <string.h>


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
extern "C" void Flip24_2(unsigned Width, unsigned Height, void *ptr_in, void *ptr_out);
#endif


bool PicRotateRGB24Impl::Rotate(void* pInImg, void* pOutImg)
{
	if (!pInImg || !pOutImg) return false;
	
	switch (m_eMode)
	{
	case ROTATE_NONE:
		{
			memcpy(pOutImg, pInImg, m_nWidth * m_nHeight * 3);
			return true;
		}
	case ROTATE_90_DEGREES_CLOCKWISE:
		{
			BYTE* pSrc = (BYTE*)pInImg;
			BYTE* pDest = NULL;
			BYTE* pSrcCol = NULL;
			for (int i = 0; i < m_nWidth; ++i, pSrc += 3)
			{
				int nNewRow = (m_nWidth - i);//Index Base 1
				int nNewIndex = ((nNewRow - 1) * m_nHeight);// Base 1
				pSrcCol = pSrc;
				pDest = (BYTE*)pOutImg + (nNewIndex*3);
				for (int j = 0; j < m_nHeight; ++j, pSrcCol += (3*m_nWidth), pDest += 3)
				{
					pDest[0] = pSrcCol[0];
					pDest[1] = pSrcCol[1];
					pDest[2] = pSrcCol[2];
				}
				// Copy pixels in same column to their destinations
			}
			return true;
		}
	case ROTATE_180_DEGREES_CLOCKWISE:
		{
			BYTE* pSrc = (BYTE*)pInImg;
			int nTotalPixels = m_nWidth * m_nHeight; 
			int nLastIndex = (nTotalPixels - 1) * 3;
			BYTE* pDest = (BYTE*)pOutImg + nLastIndex;
			for (int i = 0; i < nTotalPixels; ++i, pSrc += 3, pDest -= 3)
			{
				pDest[0] = pSrc[0];
				pDest[1] = pSrc[1];
				pDest[2] = pSrc[2];
			}
			return true;
		}

	case ROTATE_270_DEGREES_CLOCKWISE:
		{
			int iTargetRowLength = m_nHeight * 3;
			BYTE* pSrc = (BYTE*)pInImg;
			BYTE* pDest = (BYTE*)pOutImg + iTargetRowLength - 3;

			for ( int y = 0; y < m_nHeight; y++ )
			{
				BYTE* pDestPixel = pDest;
				for (int x = 0; x < m_nWidth; ++x, pSrc += 3, pDestPixel += iTargetRowLength)
				{
					pDestPixel[0] = pSrc[0];
					pDestPixel[1] = pSrc[1];
					pDestPixel[2] = pSrc[2];
				}
				pDest -= 3;
			}
			return true;
		}
	case ROTATE_FLIP_VERTICAL:
		{
			// Code to flip image
			const int nRowLength = m_nWidth * 3;
			BYTE* pSrc = (BYTE*)pInImg;
			//BYTE* pDest = (BYTE*)pOutImg + ((m_nHeight * nRowLength ) - nRowLength);
			BYTE* pDest = (BYTE*)pOutImg + ((m_nHeight - 1) * nRowLength);
			for (int i = 0; i < m_nHeight; ++i, pSrc += nRowLength, pDest -= nRowLength)
			{
				memcpy(pDest, pSrc, nRowLength);
			}
			return true;
		}
	case ROTATE_FLIP_HORIZONTAL:
		{			
#ifdef USE_ASM
			Flip24_2(m_nWidth, m_nHeight, pInImg, pOutImg);
#else
			const int iRowLength = m_nWidth * 3;
			BYTE* pSrc = (BYTE*)pInImg;
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
			BYTE* pSrc = (BYTE*)pInImg;
			BYTE* pDest = NULL;
			BYTE* pSrcCol = NULL;
			for (int i = 0; i < m_nWidth; ++i, pSrc += 3)
			{
				int nNewRow = (m_nWidth - i);//Index Base 1
				// Select target row to be one higher and subtract sizeof RGB block to be at last index of row
				++nNewRow;
				int nNewIndex = ((nNewRow - 1) * m_nHeight);// Base 1
				// Adjust index to previous row
				--nNewIndex;
				pSrcCol = pSrc;
				pDest = (BYTE*)pOutImg + (nNewIndex*3);
				for (int j = 0; j < m_nHeight; ++j, pSrcCol += (3*m_nWidth), pDest -= 3)
				{
					pDest[0] = pSrcCol[0];
					pDest[1] = pSrcCol[1];
					pDest[2] = pSrcCol[2];
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

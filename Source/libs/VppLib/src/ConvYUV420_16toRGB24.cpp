/** @file

MODULE:			ConvYUV420_16toBGR24
                        
FILE NAME:		ConvYUV420_16toBGR24.cpp

DESCRIPTION:		Integer implementation of YUV420 (16 bpp) to RGB 24 bit 
			colour convertion derived from the YUV420toRGBConverter base class.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
==========================================================================*/
#include "ConvYUV420_16toRGB.h"


#define RYUVRGB24CI2V16_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )


/*===========================================================================
	Private Methods.
==========================================================================*/


void ConvYUV420_16toBGR24::FlipConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const
{
  const int lumX = _width;
  const int tworows  = lumX << 1;
  const int rgb1row  = (lumX * 3);
  const int rgb2rows = (_width * 3) << 1;

  unsigned yy;
  int lumposy;
  int rgbposy;
  int32_t r,b,g;
  int uvpos = 0;

  for(yy=0,lumposy=0,rgbposy=(_height-1)*rgb1row; yy<_height; yy+=2,lumposy+=tworows,rgbposy-=rgb2rows)  
  {
    int lumpos0 = lumposy;
    int lumpos1 = lumpos0 + lumX;
    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbposy - rgb1row;    
    
    unsigned xx = lumX/2;
    while(xx-- > 0)		// this is little bit faster than for(xx=0; xx<lumX; xx+=2)
    {
      const int32_t U = ((int32_t)(u[uvpos])) - 128;
      const int32_t V = ((int32_t)(v[uvpos++])) - 128;		        			

			/// Lum00, u and v.

  		/// Fast calculation intermediate variables. 
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      {
	const int32_t lum00 = (int32_t)(y[lumpos0++]) << 8;
	b = (lum00 + cc) >> 8;
	g = (lum00 + cb) >> 8;
	r = (lum00 + ca) >> 8;
      }  
			/// R, G & B have range 0..255.
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum01.
      {
        const int32_t lum01 = ((int32_t)(y[lumpos0++])) << 8;
        b = (lum01 + cc) >> 8;
        g = (lum01 + cb) >> 8;
        r = (lum01 + ca) >> 8;
      }
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum10.
      {
        const int32_t lum10 = ((int32_t)(y[lumpos1++])) << 8;
        b = (lum10 + cc) >> 8;
        g = (lum10 + cb) >> 8;
        r = (lum10 + ca) >> 8;
      }
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum11.
      {
        const int32_t lum11 = ((int32_t)(y[lumpos1++])) << 8;
	b = (lum11 + cc) >> 8;
	g = (lum11 + cb) >> 8;
	r = (lum11 + ca) >> 8;
      }
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

    } //end for x...

    //rgbpos0 += rgb1row;
    //lumpos0 += lumX;		// 2*lumX. First lumX has been added during x loop.		
  } //end for y...
} //end FlipConvert


void ConvYUV420_16toBGR24::NonRotateConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const
{
  const int lumX = _width;
  const int tworows  = lumX << 1;
  const int rgb1row  = (lumX * 3);
  const int rgb2rows = (_width * 3) << 1;

  unsigned yy;
  int lumposy;
  int rgbposy;
  int32_t r,b,g;
  int uvpos = 0;

  for(yy=0,lumposy=0,rgbposy=0; yy<_height; yy+=2,lumposy+=tworows,rgbposy+=rgb2rows)  
  {
    int lumpos0 = lumposy;
    int lumpos1 = lumpos0 + lumX;
    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbpos0 + rgb1row;
    
    unsigned xx = lumX/2;
    while(xx-- > 0)		// this is little bit faster than for(xx=0; xx<lumX; xx+=2)
    {
      const int32_t U = ((int32_t)(u[uvpos])) - 128;
      const int32_t V = ((int32_t)(v[uvpos++])) - 128;		        			

			/// Lum00, u and v.

  		/// Fast calculation intermediate variables. 
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      {
	const int32_t lum00 = (int32_t)(y[lumpos0++]) << 8;
	b = (lum00 + cc) >> 8;
	g = (lum00 + cb) >> 8;
	r = (lum00 + ca) >> 8;
      }  
			/// R, G & B have range 0..255.
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum01.
      {
        const int32_t lum01 = ((int32_t)(y[lumpos0++])) << 8;
        b = (lum01 + cc) >> 8;
        g = (lum01 + cb) >> 8;
        r = (lum01 + ca) >> 8;
      }
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum10.
      {
        const int32_t lum10 = ((int32_t)(y[lumpos1++])) << 8;
        b = (lum10 + cc) >> 8;
        g = (lum10 + cb) >> 8;
        r = (lum10 + ca) >> 8;
      }
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum11.
      {
        const int32_t lum11 = ((int32_t)(y[lumpos1++])) << 8;
	b = (lum11 + cc) >> 8;
	g = (lum11 + cb) >> 8;
	r = (lum11 + ca) >> 8;
      }
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

    } //end for x...

    //rgbpos0 += rgb1row;
    //lumpos0 += lumX;		// 2*lumX. First lumX has been added during x loop.		
  } //end for y...
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

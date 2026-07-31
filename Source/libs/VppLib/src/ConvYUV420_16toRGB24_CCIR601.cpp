/** @file

MODULE:			ConvYUV420_16toBGR24_CCIR601
                        
FILE NAME:		ConvYUV420_16toBGR24_CCIR601.cpp

DESCRIPTION:		Integer implementation of implementation of YUV420 (16 bpp) to RGB 24 bit 
			colour convertion derived from	the YUV420toRGBConverter base 
			class. Assumes that the colour space of the YUV components
			are defined by the CCIR-601 standard.
					  
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


//  CCIR-601: The defenition assumes limited Y and UV (CbCr) ranges but the inverse
//		  conversion will treat it as full range and only compensate for the shift.
//  r = (y-16) + 1.140(v-128)			~	r*256 = 256*(y-16) + 292*(v-128)
//  g = (y-16) - 0.394(u-128) - 0.581(v-128)	~	g*256 = 256*(y-16) - 101*(u-128) - 149*(v-128)
//  b = (y-16) + 2.033(u-128)			~	b*256 = 256*(y-16) + 520*(u-128)

//#define RYUVRGB24CC_U0	 2.033
//#define RYUVRGB24CC_U1	-0.394
//#define RYUVRGB24CC_V1	-0.581
//#define RYUVRGB24CC_V0	 1.140

#define RYUVRGB24CCV16_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )



/*===========================================================================
	Private Methods.
==========================================================================*/


void ConvYUV420_16toBGR24_CCIR601::FlipConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const
{
  const int lumX = _width;
  const int uvX	= _width >> 1;
  const int tworows  = lumX << 1;
  const int rgb1row  = (_width * 3);
  const int rgb2rows = (_width * 3) << 1;

  unsigned yy, xx;
  int lumposy;
  int rgbposy;
  int uvposy;
  int32_t r,b,g;

  for(yy=0,lumposy=0,uvposy=0,rgbposy=(_height-1)*rgb1row;  yy<_height;  yy+=2,lumposy+=tworows,uvposy+=uvX,rgbposy-=rgb2rows)
  {
    int lumpos0 = lumposy;					// Reset to start of rows.
    int lumpos1 = lumposy + lumX;
    int uvpos	 = uvposy;

    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbposy - rgb1row;

    xx = (_width+1) / 2;
    while(xx-- > 0)
    {
      const int32_t U = (u[uvpos]) - 128;
      const int32_t V = (v[uvpos++]) - 128;
			// Lum00, u and v.

  		// Fast calculation intermediate variables. 
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      {
        const int32_t lum00 = 256 * (y[lumpos0++]) - 16*256;
	b = (lum00 + cc) >> 8;
	g = (lum00 + cb) >> 8;
	r = (lum00 + ca) >> 8;
  
			// R, G & B have range 0..255.
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
      {			// Lum01.
	const int32_t lum01 = 256 * (y[lumpos0++]) - 16*256;

	b = (lum01 + cc) >> 8;
	g = (lum01 + cb) >> 8;
	r = (lum01 + ca) >> 8;
  
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
      {			// Lum10.
	const int32_t lum10 = 256 * (y[lumpos1++]) - 16*256;

	b = (lum10 + cc) >> 8;
	g = (lum10 + cb) >> 8;
	r = (lum10 + ca) >> 8;
  
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
      {			// Lum11.
	const int32_t lum11 = 256 * (y[lumpos1++]) - 16*256;

	b = (lum11 + cc) >> 8;
	g = (lum11 + cb) >> 8;
	r = (lum11 + ca) >> 8;
  
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
    } //end for x...
		
  } //end for y...

} //end FlipConvert


void ConvYUV420_16toBGR24_CCIR601::NonRotateConvert(const int16_t* y, const int16_t *u, const int16_t *v, unsigned char *pRgb) const
{
  const int lumX = _width;
  const int uvX	= _width >> 1;
  const int tworows  = lumX << 1;
  const int rgb1row  = (_width * 3);
  const int rgb2rows = (_width * 3) << 1;

  unsigned yy, xx;
  int lumposy;
  int rgbposy;
  int uvposy;
  int32_t r,b,g;

  for(yy=0,lumposy=0,uvposy=0,rgbposy=0;  yy<_height;  yy+=2,lumposy+=tworows,uvposy+=uvX,rgbposy+=rgb2rows)
  {
    int lumpos0 = lumposy;					// Reset to start of rows.
    int lumpos1 = lumposy + lumX;
    int uvpos	 = uvposy;

    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbposy + rgb1row;

    xx = (_width+1) / 2;
    while(xx-- > 0)
    {
      const int32_t U = (u[uvpos]) - 128;
      const int32_t V = (v[uvpos++]) - 128;
			// Lum00, u and v.

  		// Fast calculation intermediate variables. 
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      {
        const int32_t lum00 = 256 * (y[lumpos0++]) - 16*256;
	b = (lum00 + cc) >> 8;
	g = (lum00 + cb) >> 8;
	r = (lum00 + ca) >> 8;
  
			// R, G & B have range 0..255.
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
      {			// Lum01.
	const int32_t lum01 = 256 * (y[lumpos0++]) - 16*256;

	b = (lum01 + cc) >> 8;
	g = (lum01 + cb) >> 8;
	r = (lum01 + ca) >> 8;
  
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos0++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
      {			// Lum10.
	const int32_t lum10 = 256 * (y[lumpos1++]) - 16*256;

	b = (lum10 + cc) >> 8;
	g = (lum10 + cb) >> 8;
	r = (lum10 + ca) >> 8;
  
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
      {			// Lum11.
	const int32_t lum11 = 256 * (y[lumpos1++]) - 16*256;

	b = (lum11 + cc) >> 8;
	g = (lum11 + cb) >> 8;
	r = (lum11 + ca) >> 8;
  
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(b));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(g));
	pRgb[rgbpos1++] = (unsigned char)(RYUVRGB24CCV16_RANGECHECK_0TO255(r));
      }
    } //end for x...
		
  } //end for y...
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @file

MODULE:			ConvYUV420toRGB24
                        
FILE NAME:		ConvYUV420toRGB24CReal.cpp

DESCRIPTION:		Integer implementation of YUV420 (8 bpp) to RGB 24 bit 
			colour conversion derived from the YUV420toRGBConverter base 	class.
					  
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
#include "ConvYUV420toRGB.h"

#include "typedfs.h"

#include <uuids.h>

DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);

#define RANGECHECK_UINT32_0TO255(__x) ( (((__x) & 0xFFFFFF00)==0 ) ? (__x) : ((~(__x))>>16 ) )

//inline unsigned char RANGECHECK_0TO255(const int32_t x)
//{
//  if(x<=0) return 0;
//  if(x>=255) return 255;
//  return x;
//}



/*===========================================================================
	Private Methods.
==========================================================================*/


bool ConvYUV420toRGB24::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatOut==MEDIASUBTYPE_RGB24) &&
         (VideoFormatIn==MEDIASUBTYPE_YUV420P_S || VideoFormatIn==MEDIASUBTYPE_I420 || VideoFormatIn==MEDIASUBTYPE_YV12);
}


void ConvYUV420toRGB24::FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
int32_t X;
pRgb += 3*_width*(_height-1);

  for(int row=0; row<_height; row+=2)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *pU++ - 128;
      const signed char V = *pV++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *pY;
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;

      Y = 256 * pY[1];

      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[3]=255;
      else if(X<0) pRgb[3]=0;
      else pRgb[3] = X;
	  
	  Y = 256 * pY[_width];
	  pRgb -= 3 * _width;

	  X = (Y + cc) >> 8;		// B: Y + 2.032*U
	  if (X>255) pRgb[2] = 255;
	  else if (X<0) pRgb[2] = 0;
	  else pRgb[2] = X;
	  X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
	  if (X>255) pRgb[1] = 255;
	  else if (X<0) pRgb[1] = 0;
	  else pRgb[1] = X;
	  X = (Y + ca) >> 8;		// R: Y + 1.40*V;
	  if (X>255) pRgb[0] = 255;
	  else if (X<0) pRgb[0] = 0;
	  else pRgb[0] = X;

	  Y = 256 * pY[_width+1];

	  X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
	  if (X>255) pRgb[5] = 255;
	  else if (X<0) pRgb[5] = 0;
	  else pRgb[5] = X;
	  X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
	  if (X>255) pRgb[4] = 255;
	  else if (X<0) pRgb[4] = 0;
	  else pRgb[4] = X;
	  X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
	  if (X>255) pRgb[3] = 255;
	  else if (X<0) pRgb[3] = 0;
	  else pRgb[3] = X;

	  pY += 2;
	  pRgb += 3 * _width + 6;
    }
    pRgb -= 9 *_width;		// 3 lines down. 
    pY += _width;
  }
} //end FlipConvert


#if 1
void ConvYUV420toRGB24::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
const unsigned width3 = _width * 3;
int lineY0 = 0;
int lineRGB0 = 0;
  
  for(unsigned row=0; row<_height; row+=2)
  {
    int lineY1 = lineY0 + _width;
    int lineRGB1 = lineRGB0 + width3;

    unsigned Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *pU++ - 128;
      const signed char V = *pV++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;
      int32_t X;
      
      {
        const int32_t Y00 = (uint32_t)pY[lineY0++] << 8;
        X = (Y00 + ca) >> 8;		// R: Y + 1.40*V;        
        if(X>255) pRgb[lineRGB0]=255;
        else if(X<0) pRgb[lineRGB0]=0;
        else pRgb[lineRGB0] = X;
        lineRGB0++;        
        X = (Y00 + cb) >> 8;		// G: Y - 0.395*U - 0.581*V        
        if(X>255) pRgb[lineRGB0]=255;
        else if(X<0) pRgb[lineRGB0]=0;
        else pRgb[lineRGB0] = X;
        lineRGB0++;
        X = (Y00 + cc) >> 8;		// B: Y + 2.032*U       
        if(X>255) pRgb[lineRGB0]=255;
        else if(X<0) pRgb[lineRGB0]=0;
        else pRgb[lineRGB0] = X;
      }
      lineRGB0++;
      
      {
        const int32_t Y01 = (uint32_t)pY[lineY0++] << 8;
        X = (Y01 + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;        
        if(X>255) pRgb[lineRGB0]=255;
        else if(X<0) pRgb[lineRGB0]=0;
        else pRgb[lineRGB0] = X;
        lineRGB0++;
        X = (Y01 + cb) >> 8;		// G: Y - 0.395*U - 0.581*V        
        if(X>255) pRgb[lineRGB0]=255;
        else if(X<0) pRgb[lineRGB0]=0;
        else pRgb[lineRGB0] = X;
        lineRGB0++;
        X = (Y01 + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
        if(X>255) pRgb[lineRGB0]=255;
        else if(X<0) pRgb[lineRGB0]=0;
        else pRgb[lineRGB0] = X;
      }
      lineRGB0++;

      {
        const int32_t Y10 = (uint32_t)pY[lineY1++] << 8;
        X = (Y10 + ca) >> 8;		// R: Y + 1.40*V;
        if(X>255) pRgb[lineRGB1]=255;
        else if(X<0) pRgb[lineRGB1]=0;
        else pRgb[lineRGB1] = X;
        lineRGB1++;
        X = (Y10 + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
        if(X>255) pRgb[lineRGB1]=255;
        else if(X<0) pRgb[lineRGB1]=0;
        else pRgb[lineRGB1] = X;        
        lineRGB1++;
        X = (Y10 + cc) >> 8;		// B: Y + 2.032*U
        if(X>255) pRgb[lineRGB1]=255;
        else if(X<0) pRgb[lineRGB1]=0;
        else pRgb[lineRGB1] = X;
      }
      lineRGB1++;

      {
        const int32_t Y11 = (uint32_t)pY[lineY1++] << 8;
        X = (Y11 + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
        if(X>255) pRgb[lineRGB1]=255;
        else if(X<0) pRgb[lineRGB1]=0;
        else pRgb[lineRGB1] = X;
        lineRGB1++;
        X = (Y11 + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
        if(X>255) pRgb[lineRGB1]=255;
        else if(X<0) pRgb[lineRGB1]=0;
        else pRgb[lineRGB1] = X;
        lineRGB1++;
        X = (Y11 + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
        if(X>255) pRgb[lineRGB1]=255;
        else if(X<0) pRgb[lineRGB1]=0;
        else pRgb[lineRGB1] = X;
      }
      lineRGB1++;
    }
    lineRGB0 = lineRGB1;
    lineY0 = lineY1;
  }
} //end NonRotateConvert.


#else

/* Alternative: This code is somehow slower. It is faster for 16 bit values, but it seems slower for 8 bits. */
  
void ConvYUV420toRGB24::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
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
      const int32_t U = (signed char)((pU[uvpos]) - 128);
      const int32_t V = (signed char)((pV[uvpos++]) - 128);
			// Lum00, u and v.

  		// Fast calculation intermediate variables. 
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      {
        const int32_t lum00 = 256 * (pY[lumpos0++]);	
	r = (lum00 + ca) >> 8;
	g = (lum00 + cb) >> 8;
	b = (lum00 + cc) >> 8;

			// R, G & B have range 0..255.
	pRgb[rgbpos0++] = (unsigned char)(RANGECHECK_UINT32_0TO255(r));
	pRgb[rgbpos0++] = (unsigned char)(RANGECHECK_UINT32_0TO255(g));
	pRgb[rgbpos0++] = (unsigned char)(RANGECHECK_UINT32_0TO255(b));
      }
      {			// Lum01.
	const int32_t lum01 = 256 * (pY[lumpos0++]);

	r = (lum01 + ca) >> 8;	
	g = (lum01 + cb) >> 8;
	b = (lum01 + cc) >> 8;

	pRgb[rgbpos0++] = (unsigned char)(RANGECHECK_UINT32_0TO255(r));
	pRgb[rgbpos0++] = (unsigned char)(RANGECHECK_UINT32_0TO255(g));
	pRgb[rgbpos0++] = (unsigned char)(RANGECHECK_UINT32_0TO255(b));
      }
      {			// Lum10.
	const int32_t lum10 = 256 * (pY[lumpos1++]);

	r = (lum10 + ca) >> 8;
	g = (lum10 + cb) >> 8;
	b = (lum10 + cc) >> 8;
  
	pRgb[rgbpos1++] = (unsigned char)(RANGECHECK_UINT32_0TO255(r));
	pRgb[rgbpos1++] = (unsigned char)(RANGECHECK_UINT32_0TO255(g));
	pRgb[rgbpos1++] = (unsigned char)(RANGECHECK_UINT32_0TO255(b));
      }
      {			// Lum11.
	const int32_t lum11 = 256 * (pY[lumpos1++]);
	
	r = (lum11 + ca) >> 8;
	g = (lum11 + cb) >> 8;
	b = (lum11 + cc) >> 8;	
  
	pRgb[rgbpos1++] = (unsigned char)(RANGECHECK_UINT32_0TO255(r));
	pRgb[rgbpos1++] = (unsigned char)(RANGECHECK_UINT32_0TO255(g));
	pRgb[rgbpos1++] = (unsigned char)(RANGECHECK_UINT32_0TO255(b));
      }
    } //end for x...
		
  } //end for y...
}

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool YUV420toRGB32Converter::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return (VideoFormatOut==MEDIASUBTYPE_RGB32 || VideoFormatOut==MEDIASUBTYPE_ARGB32) &&
         (VideoFormatIn==MEDIASUBTYPE_YUV420P_S || VideoFormatIn==MEDIASUBTYPE_I420 || VideoFormatIn==MEDIASUBTYPE_YV12);
}


void YUV420toRGB32Converter::FlipConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
int32_t X;
pRgb += 4*_width*(_height-1);

  for(int row=0; row<_height; row++)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *pU++ - 128;
      const signed char V = *pV++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;
      int32_t Y = 256 * *(pY++);

      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;
      pRgb[3] = 0;			// A

      Y = 256 * *(pY++);

      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[6]=255;
      else if(X<0) pRgb[6]=0;
      else pRgb[6] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;
      pRgb[7] = 0;			// A

      pRgb += 8;
    }
    pRgb -= 8*_width;
    if((row&1) == 0)
    {
      pU -= _width/2;
      pV -= _width/2;
    }  
  }
} //end FlipConvert


void YUV420toRGB32Converter::NonRotateConvert(const unsigned char* pY, const signed char* pU, const signed char* pV, unsigned char *pRgb) const
{
int32_t X;
const unsigned width4 = _width * 4;

  for(int row=0; row<_height; row+=2)
  {
    int Pixel2Count = (_width+1) / 2;
    while(Pixel2Count-->0)		//converts to RGB
    {
      const signed char U = *pU++ - 128;
      const signed char V = *pV++ - 128;
      const int32_t cc = U*520 + 127;
      const int32_t ca = V*292 + 127;
      const int32_t cb = -101*U - 149*V + 127;

      int32_t Y = 256 * *pY;
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;

      pRgb[3] = 0;			// A

      Y = 256 * pY[1];
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[6]=255;
      else if(X<0) pRgb[6]=0;
      else pRgb[6] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;

      pRgb[7] = 0;			// A

      pRgb += width4;

      Y = 256 * pY[_width];
      X = (Y + cc) >> 8;		// B: Y + 2.032*U
      if(X>255) pRgb[2]=255;
      else if(X<0) pRgb[2]=0;
      else pRgb[2] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[1]=255;
      else if(X<0) pRgb[1]=0;
      else pRgb[1] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.40*V;
      if(X>255) pRgb[0]=255;
      else if(X<0) pRgb[0]=0;
      else pRgb[0] = X;

      pRgb[3] = 0;			// A

      Y = 256 * pY[_width+1];
      X = (Y + cc) >> 8;		// B: Y + 1.772*Y + 2.034*U;
      if(X>255) pRgb[6]=255;
      else if(X<0) pRgb[6]=0;
      else pRgb[6] = X;
      X = (Y + cb) >> 8;		// G: Y - 0.395*U - 0.581*V
      if(X>255) pRgb[5]=255;
      else if(X<0) pRgb[5]=0;
      else pRgb[5] = X;
      X = (Y + ca) >> 8;		// R: Y + 1.402*V;   Y + 1.137*V;
      if(X>255) pRgb[4]=255;
      else if(X<0) pRgb[4]=0;
      else pRgb[4] = X;

      pRgb[7] = 0;			// A

      pRgb = pRgb - width4 + 8;
      pY += 2;
    }
    pRgb += width4;
    pY += _width;
  }
} //end NonRotateConvert.


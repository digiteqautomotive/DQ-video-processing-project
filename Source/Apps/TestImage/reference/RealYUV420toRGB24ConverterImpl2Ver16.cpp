/** @file

MODULE		: RealYUV420toBGR24ConverterImpl2Ver16

TAG		: RYUVRGB24CI2V16

FILE NAME	: RealYUV420toBGR24ConverterImpl2Ver16.cpp

DESCRIPTION	: Floating point implementation of YUV420 (16 bpp) to RGB 24 bit 
		colour convertion derived from the YUV420toRGBConverter base 
		class.

COPYRIGHT	(c)CSIR 2007-2012 all rights resevered

LICENSE		: Software License Agreement (BSD License)

RESTRICTIONS	: Redistribution and use in source and binary forms, with or without 
		modification, are permitted provided that the following conditions 
		are met:

		* Redistributions of source code must retain the above copyright notice, 
		this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright notice, 
		this list of conditions and the following disclaimer in the documentation 
		and/or other materials provided with the distribution.
		* Neither the name of the CSIR nor the names of its contributors may be used 
		to endorse or promote products derived from this software without specific 
		prior written permission.

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
#include "ConvYUV420toRGB24RealImpl2Ver16.h"

#include "typedfs.h"

#include <uuids.h>

/*===========================================================================
				Macros
===========================================================================*/
#define RYUVRGB24CI2V16_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )

#define RYUVRGB24CI2V16_U0		 2.032
#define RYUVRGB24CI2V16_U1		-0.394
#define RYUVRGB24CI2V16_V1		-0.581
#define RYUVRGB24CI2V16_V0		 1.140

/*===========================================================================
	Constructors.
===========================================================================*/

RealYUV420toBGR24ConverterImpl2Ver16::RealYUV420toBGR24ConverterImpl2Ver16(int width, int height): CVideoConverter(width, height)
{ 
}//end constructor.


bool RealYUV420toBGR24ConverterImpl2Ver16::isVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const
{
  return VideoFormatOut==MEDIASUBTYPE_RGB24;
}


void RealYUV420toBGR24ConverterImpl2Ver16::Convert(const void* pBufferIn, void* pRgb) const
{    
	//set the Y, U and V pointers to point to the correct positions within the byte array
  const unsigned m_nInPixelBytes = 2 * (_width * _height);
  //const yuvType *pY = (const yuvType *)pBufferIn;
  const int16_t *pU = (const int16_t*)((uint8_t*)pBufferIn + m_nInPixelBytes);
  const int16_t *pV = (const int16_t*)((uint8_t*)pU + m_nInPixelBytes/4);
        // let invert take precedence, not catering for both
  if(_flip) 
      FlipConvert(pBufferIn, pU, pV, (unsigned char*)pRgb);
  else				
      NonRotateConvert(pBufferIn, pU, pV, (unsigned char*)pRgb);
}  


/*
===========================================================================
	Private Methods.
===========================================================================
*/
void RealYUV420toBGR24ConverterImpl2Ver16::NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char* pRgb) const
{
  unsigned char *optr	= (unsigned char *)pRgb;
  const int16_t *py	= (int16_t *)pY;
  const int16_t *pu	= (int16_t *)pU;
  const int16_t *pv	= (int16_t *)pV;
  int	lumX	= _width;
  int	lumY	= _height;
  int	uvX	= _width >> 1;

  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;

	for(y = 0,lumposy = 0,uvposy = 0,rgbposy = 0;	y < lumY;	y += 2,lumposy += tworows,uvposy += uvX,rgbposy += rgb2rows)
	{
		int lumpos0 = lumposy;					///< Reset to start of rows.
		int lumpos1 = lumposy + lumX;
		int uvpos	 = uvposy;

		int rgbpos0 = rgbposy;
		int rgbpos1 = rgbposy + rgb1row;

		for(x = 0; x < lumX; x += 2)
		{
			double lum00 = (double)((int)(py[lumpos0++]));
			double u		 = (double)(((int)(pu[uvpos])) - 128);
			double v		 = (double)(((int)(pv[uvpos++])) - 128);

			/// Lum00, u and v.

  		/// Fast calculation intermediate variables. 
			double cc = (RYUVRGB24CI2V16_U0 * u) + 0.5;
			double cb = (RYUVRGB24CI2V16_U1 * u) + (RYUVRGB24CI2V16_V1 * v) + 0.5;
			double ca = (RYUVRGB24CI2V16_V0 * v) + 0.5;

			b = (int)(lum00 + cc);
			g = (int)(lum00 + cb);
			r = (int)(lum00 + ca);
  
			/// R, G & B have range 0..255.
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum01.
			double lum01 = (double)((int)(py[lumpos0++]));

			b = (int)(lum01 + cc);
			g = (int)(lum01 + cb);
			r = (int)(lum01 + ca);
  
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
			optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum10.
			double lum10 = (double)((int)(py[lumpos1++]));

			b = (int)(lum10 + cc);
			g = (int)(lum10 + cb);
			r = (int)(lum10 + ca);
  
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

			/// Lum11.
			double lum11 = (double)((int)(py[lumpos1++]));

			b = (int)(lum11 + cc);
			g = (int)(lum11 + cb);
			r = (int)(lum11 + ca);
  
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
			optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

		}//end for x...
		
	}//end for y...

}//end NonRotateConvert.


void RealYUV420toBGR24ConverterImpl2Ver16::FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char* pRgb) const
{
  unsigned char	*optr	= (unsigned char *)pRgb;
  const short	*py	= (short *)pY;
  const short	*pu	= (short *)pU;
  const short	*pv	= (short *)pV;
  int		lumX	= _width;
  int		lumY	= _height;
  int		uvX	= _width >> 1;

  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;
  int lastrow = _width * (_height - 1) * 3;

  //for(y = 0,lumposy = 0,uvposy = 0,rgbposy = 0;	y < lumY;	y += 2,lumposy += tworows,uvposy += uvX,rgbposy += rgb2rows)
  for(y = 0,lumposy = 0,uvposy = 0,rgbposy = lastrow;	y < lumY;	y += 2,lumposy += tworows,uvposy += uvX,rgbposy -= rgb2rows)
  {
    int lumpos0 = lumposy;					///< Reset to start of rows.
    int lumpos1 = lumposy + lumX;
    int uvpos	 = uvposy;

    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbposy - rgb1row;

    for(x = 0; x < lumX; x += 2)
    {
      double lum00 = (double)((int)(py[lumpos0++]));
      double u		 = (double)(((int)(pu[uvpos])) - 128);
      double v		 = (double)(((int)(pv[uvpos++])) - 128);

      /// Lum00, u and v.

      /// Fast calculation intermediate variables. 
      double cc = (RYUVRGB24CI2V16_U0 * u) + 0.5;
      double cb = (RYUVRGB24CI2V16_U1 * u) + (RYUVRGB24CI2V16_V1 * v) + 0.5;
      double ca = (RYUVRGB24CI2V16_V0 * v) + 0.5;

      b = (int)(lum00 + cc);
      g = (int)(lum00 + cb);
      r = (int)(lum00 + ca);

      /// R, G & B have range 0..255.
      optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

      /// Lum01.
      double lum01 = (double)((int)(py[lumpos0++]));

      b = (int)(lum01 + cc);
      g = (int)(lum01 + cb);
      r = (int)(lum01 + ca);

      optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      optr[rgbpos0++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

      /// Lum10.
      double lum10 = (double)((int)(py[lumpos1++]));

      b = (int)(lum10 + cc);
      g = (int)(lum10 + cb);
      r = (int)(lum10 + ca);

      optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

      /// Lum11.
      double lum11 = (double)((int)(py[lumpos1++]));

      b = (int)(lum11 + cc);
      g = (int)(lum11 + cb);
      r = (int)(lum11 + ca);

      optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(b));
      optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(g));
      optr[rgbpos1++] = (unsigned char)(RYUVRGB24CI2V16_RANGECHECK_0TO255(r));

    }//end for x...

  }//end for y...
}

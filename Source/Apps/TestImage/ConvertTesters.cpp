#include "YUV444toRGBConverter.h"
#include "ConvertTesters.h"


#ifndef BYTE
typedef unsigned char BYTE;
#endif

/*
===========================================================================
	Macros.
===========================================================================
*/
#define RYUVRGB24C_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )


/*
https://www.computerlanguage.com/results.php?definition=YUV%2FRGB+conversion+formulas From YUV to RGB
   R = Y + 1.140V
   G = Y - 0.395U - 0.581V
   B = Y + 2.032U
https://www.pcmag.com/encyclopedia/term/yuvrgb-conversion-formulas
From YUV to RGB
   R = Y + 1.140V
   G = Y - 0.395U - 0.581V
   B = Y + 2.032U

This formula is somehow different, but it uses U' and V':
https://colorfaq.com/article/how-to-correctly-convert-yuv-to-rgb-3ZwZTlC1n
   B = 1.164(Y - 16)  + 2.018(U - 128)
   G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
   R = 1.164(Y - 16) + 1.596(V - 128)
 ===========
   R = Y + 1.403V'
   G = Y - 0.344U' - 0.714V'
   B = Y + 1.770U'
*/

#define RYUVRGB24C_U0		 2.032		// B = Y + 2.032U
#define RYUVRGB24C_U1		-0.395
#define RYUVRGB24C_V1		-0.581		// G = Y - 0.395U - 0.581V
#define RYUVRGB24C_V0		 1.140		// R = Y + 1.140V

/*
===========================================================================
	Private Methods.
===========================================================================
*/

//Note: https://topic.alibabacloud.com/a/about-the-common-ayuv-formats-in-rgb-yuy2-yuyv-yvyu-uyvy-directshow-rgbyuv_8_8_10271129.html
// The ¨AYUV format has an alpha channel, and the YUV component is extracted for each pixel, and the image Data format is as follows:
// A0 Y0 U0 V0 A1 Y1 U1 V1 ... 


void OrigRealYUV444toRGB24Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
    BYTE *optr	=	(unsigned char *)pRgb;
    BYTE *inBuffer	= (BYTE *)pInputBuffer;

    int numberOfPixels = _width * _height;

    for (int i = 0; i < numberOfPixels; i++ )
    {
        BYTE y = inBuffer[ (i * 4 ) + 1 ];	// JFO: Original byte ordering was wrong, fixed.
        BYTE u = inBuffer[ (i * 4 ) + 2 ];
        BYTE v = inBuffer[ (i * 4 ) + 3 ];
        
        
        //drop the 4th byte (= alpha-channel which is always 0 in AYUV) to go from 32bit to 24 bit per pixel

        int r = y + RYUVRGB24C_V0 * (v - 128);
        int g = y + RYUVRGB24C_U1 * (u - 128) + RYUVRGB24C_V1 * (v - 128);
        int b = y + RYUVRGB24C_U0 * (u - 128);

        optr[i * 3] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(b);
        optr[(i * 3) + 1] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(g);
        optr[(i *3) + 2] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(r);
    }
}//end Convert



///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

#define _chrOff			128


void OrigRealYUV420toRGB24Converter::FlipConvert(const void* pY, const void* pU, const void* pV, void* pRgb) const
{
  unsigned char* 	optr	=	(unsigned char *)pRgb;
  const yuvType*	py	= (yuvType *)pY;
  const yuvType*	pu	= (yuvType *)pU;
  const yuvType*	pv	= (yuvType *)pV;
  int		lumX	= _width;
  int		lumY	= _height;
  int		uvX		= _width >> 1;

  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;
  int lastrow = _width * (_height - 1) * 3;

  for(y=0,lumposy=0,uvposy=0,rgbposy=lastrow;	y<lumY;	y+=2,lumposy+=tworows,uvposy+=uvX,rgbposy-=rgb2rows)
  {
	int lumpos0 = lumposy;					// Reset to start of rows.
	int lumpos1 = lumposy + lumX;
	int uvpos	 = uvposy;

	int rgbpos0 = rgbposy;
	int rgbpos1 = rgbposy - rgb1row;

	for(x = 0; x < lumX; x += 2)
	{
		const double dlum00 = (double)((int)(py[lumpos0++]));

		const double du		 = (double)(((int)(pu[uvpos])) );
		const double dv		 = (double)(((int)(pv[uvpos++])) );

		const unsigned char lum00 = dlum00;
		const signed char u		 = (double)(du + _chrOff);
		const signed char v		 = (double)(dv + _chrOff);

			// Lum00, u and v.
  		// Fast calculation intermediate variables. 
		const double cc = (RYUVRGB24C_U0 * u) + 0.5;
		const double cb = (RYUVRGB24C_U1 * u) + (RYUVRGB24C_V1 * v) + 0.5;
		const double ca = (RYUVRGB24C_V0 * v) + 0.5;

		b = (int)(lum00 + cc);
		g = (int)(lum00 + cb);
		r = (int)(lum00 + ca);
  
			// R, G & B have range 0..255.
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum01.
		const double dlum01 = (double)((int)(py[lumpos0++]));
		const unsigned char lum01 = dlum01;
		b = (int)(lum01 + cc);
		g = (int)(lum01 + cb);
		r = (int)(lum01 + ca);
  
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum10.
		const double dlum10 = (double)((int)(py[lumpos1++]));
		const unsigned char lum10 = dlum10;

		b = (int)(lum10 + cc);
		g = (int)(lum10 + cb);
		r = (int)(lum10 + ca);
  
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum11.
		const double dlum11 = (double)((int)(py[lumpos1++]));
		const unsigned char lum11 = dlum11;

		b = (int)(lum11 + cc);
		g = (int)(lum11 + cb);
		r = (int)(lum11 + ca);
  
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

	}//end for x...
		
  }//end for y...

}//end OrigRealYUV420toRGB24Converter::FlipConvert


void OrigRealYUV420toRGB24Converter::NonRotateConvert(const void* pY, const void* pU, const void* pV, void* pRgb) const
{
  unsigned char* 	optr	=	(unsigned char *)pRgb;
  const yuvType*				py		= (yuvType *)pY;
  const yuvType*				pu		= (yuvType *)pU;
  const yuvType*				pv		= (yuvType *)pV;
  int		lumX	= _width;
  int		lumY	= _height;
  int		uvX		= _width >> 1;
  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;

  for(y=0,lumposy=0,uvposy=0,rgbposy=0;	y<lumY;	y+= 2,lumposy+=tworows,uvposy+=uvX,rgbposy+=rgb2rows)
  {
    int lumpos0 = lumposy;					// Reset to start of rows.
    int lumpos1 = lumposy + lumX;
    int uvpos	 = uvposy;

    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbposy + rgb1row;

    for(x = 0; x < lumX; x += 2)
    {
	const double dlum00 = (double)((int)(py[lumpos0++]));

	const double du = (double)(((int)(pu[uvpos])));
	const double dv = (double)(((int)(pv[uvpos++])));

	const unsigned char lum00 = dlum00;
	const signed char u = (double)(du + _chrOff);
	const signed char v = (double)(dv + _chrOff);

			// Lum00, u and v.
  		// Fast calculation intermediate variables. 
	const double cc = (RYUVRGB24C_U0 * u) + 0.5;
	const double cb = (RYUVRGB24C_U1 * u) + (RYUVRGB24C_V1 * v) + 0.5;
	const double ca = (RYUVRGB24C_V0 * v) + 0.5;

	b = (int)(lum00 + cc);
	g = (int)(lum00 + cb);
	r = (int)(lum00 + ca);
  
			// R, G & B have range 0..255.
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum01.
	const double dlum01 = (double)((int)(py[lumpos0++]));
        const unsigned char lum01 = dlum01;
	b = (int)(lum01 + cc);
	g = (int)(lum01 + cb);
	r = (int)(lum01 + ca);
  
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum10.
	const double dlum10 = (double)((int)(py[lumpos1++]));
	const unsigned char lum10 = dlum10;

	b = (int)(lum10 + cc);
	g = (int)(lum10 + cb);
	r = (int)(lum10 + ca);
  
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum11.
	const double dlum11 = (double)((int)(py[lumpos1++]));
        const unsigned char lum11 = dlum11;

	b = (int)(lum11 + cc);
	g = (int)(lum11 + cb);
	r = (int)(lum11 + ca);
  
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

    }//end for x...
		
  }//end for y...

}//end NonRotateConvert.



